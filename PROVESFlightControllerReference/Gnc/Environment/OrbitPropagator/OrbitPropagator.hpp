/**
 * \file OrbitPropagator.hpp
 * \brief SGP4 propagation and time scales.
 */
#ifndef Gnc_Environment_OrbitPropagator_HPP
#define Gnc_Environment_OrbitPropagator_HPP

#include "PROVESFlightControllerReference/Gnc/Environment/OrbitPropagator/OrbitPropagatorComponentAc.hpp"
#include "PROVESFlightControllerReference/Gnc/AstroLib/AstroLib.hpp"

#include <perturb/perturb.hpp>

namespace Gnc {
namespace Environment {

/**
 * \brief Head of the GNC chain: time acquisition and SGP4 propagation.
 *
 * \details Once per tick it reads the clock, builds every time scale,
 * propagates the loaded TLE, derives the ground track, and broadcasts a
 * Gnc::OrbitState. Time validity and position validity are reported
 * separately, so a spacecraft with a good clock and no TLE still drives
 * the solar ephemeris.
 */
class OrbitPropagator final : public OrbitPropagatorComponentBase {
  public:
    /**
     * \brief Construct the component with no TLE loaded.
     * \param compName  F Prime component instance name
     */
    explicit OrbitPropagator(const char* const compName);

    //! Destroy the component. Holds no resources.
    ~OrbitPropagator();

  private:
    /**
     * \brief Rate group tick. Runs the four-stage pipeline.
     *
     * \details Every exit path emits an OrbitState, including the
     * failure paths, so downstream components can distinguish "no time"
     * from "no TLE" from "propagator error" rather than just seeing
     * nothing arrive.
     *
     * \param portNum  Port index, unused (single port)
     * \param context  Rate group context, unused
     */
    void run_handler(FwIndexType portNum, U32 context) override;

    /**
     * \brief Parse and install a new two-line element set.
     *
     * \details Parses into a candidate first and only commits on
     * success, so a bad uplink never leaves the spacecraft worse off
     * than before it arrived.
     *
     * \param opCode  Command opcode
     * \param cmdSeq  Command sequence number
     * \param line1   TLE line 1
     * \param line2   TLE line 2
     */
    void LOAD_TLE_cmdHandler(FwOpcodeType opCode,
                             U32 cmdSeq,
                             const Fw::CmdStringArg& line1,
                             const Fw::CmdStringArg& line2) override;

    /**
     * \brief Invalidate the loaded TLE.
     *
     * \details Position output stops; time output continues. The orbital
     * record itself is left in place and gated off by m_tleValid.
     *
     * \param opCode  Command opcode
     * \param cmdSeq  Command sequence number
     */
    void CLEAR_TLE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    /**
     * \brief Stage 1. Read the time port and build every needed time scale.
     *
     * \details The only place in the GNC chain that reads the clock for computation.
     * Everything downstream receives jdUt1, jdTt, gmstRad
     * and the Fw::Time stamp inside OrbitState.
     *
     * Requires wall-clock time, not uptime.
     *
     * \param ts     [out] UTC / UT1 / TT scales for this instant
     * \param stamp  [out] The raw Fw::Time the scales were built from,
     *               carried downstream as the observation epoch
     * \return false after emitting WARNING_HI TimeMissing when the time
     *         base is not wall clock; true otherwise
     */
    bool acquireTime(Astro::TimeScales& ts, Fw::Time& stamp);

    /**
     * \brief Stage 2. Propagate the loaded TLE to the current epoch (SGP4).
     *
     * \details The caller must have checked m_tleValid. Time since epoch
     * is passed explicitly rather than as an absolute date, because that
     * is the quantity SGP4's drag and secular terms are series in.
     *
     * \param ts       Time scales from acquireTime()
     * \param posTeme  [out] Position in TEME, km
     * \param velTeme  [out] Velocity in TEME, km/s
     * \param ageDays  [out] Time since TLE epoch, days. Written even on
     *                 failure, and may be negative for a future epoch.
     * \return false after emitting WARNING_HI Sgp4Failure on propagator
     *         error; true otherwise
     */
    bool propagate(const Astro::TimeScales& ts,
                   Astro::Vec3& posTeme,
                   Astro::Vec3& velTeme,
                   F64& ageDays);

    /**
     * \brief Stage 3. Write telemetry, including the ground track.
     *
     * \details Reads geodetic position from the state rather than
     * recomputing it. Keeps the ground track and the magnetic field model in agreement.
     *
     * \param state  Fully populated orbit state for this cycle
     */
    void publish(const Gnc::OrbitState& state);

    /**
     * \brief Stage 4. Broadcast the state on orbitOut.
     *
     * \details Fans out synchronously on this thread, so every consumer
     * of a given cycle sees the same state and the CycleUsec budget
     * below covers their execution too.
     *
     * \param state  Orbit state to broadcast, valid or not
     */
    void emit(const Gnc::OrbitState& state);

    /**
     * The SGP4 orbital record. perturb::Satellite has no default
     * constructor, so it is seeded with a value-initialized elsetrec
     * and gated by m_tleValid.
     */
    perturb::Satellite m_sat;  //!< Valid only while m_tleValid is true
    
    bool m_tleValid = false;  //!< Whether m_sat holds a successfully parsed TLE
};

}  // namespace Environment
}  // namespace Gnc

#endif
