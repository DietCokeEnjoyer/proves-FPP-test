/**
 * \file OrbitPropagator.hpp
 * \brief SGP4 propagation and time scales.
 */
#ifndef Gnc_Environment_OrbitPropagator_HPP
#define Gnc_Environment_OrbitPropagator_HPP

#include "PROVESFlightControllerReference/Gnc/Environment/OrbitPropagator/OrbitPropagatorComponentAc.hpp"
#include "PROVESFlightControllerReference/Gnc/Environment/OrbitPropagator/Sgp4Propagator.hpp"

namespace Gnc {
namespace Environment {

/**
 * \brief Time acquisition and SGP4 propagation.
 *
 * \details Once per tick it reads the clock, builds every time scale,
 * propagates the loaded TLE, derives the ground track, and broadcasts a
 * Gnc::OrbitState.
 *
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
     * \brief Rate group tick. Runs the pipeline.
     *
     * \details An OrbitState is always emitted, even on failures,
     * as it contains diagnostic information about the failures.
     *
     * \param portNum  Port index, unused (single port)
     * \param context  Rate group context, unused
     */
    void run_handler(FwIndexType portNum, U32 context) override;

    /**
     * \brief Parse and install a new two-line element set.
     *
     * \details Only replaces old TLE on a successful parse.
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
     * record itself is left in place and gated off by the propagator's
     * validity flag.
     *
     * \param opCode  Command opcode
     * \param cmdSeq  Command sequence number
     */
    void CLEAR_TLE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    /**
     * \brief Read the time port and decide whether the clock is usable.
     *
     * \details The only place in the GNC chain that reads the clock for
     * computation. Everything downstream receives jdUt1, jdTt, gmstRad
     * and the Fw::Time stamp inside OrbitState.
     *
     * Requires wall-clock time, not uptime.
     *
     * \param stamp        [out] The Fw::Time read, carried
     *                     as the observation epoch
     * \param unixSeconds  [out] The same instant as POSIX seconds, before
     *                     UTC_OFFSET_SEC is applied
     * \return false when the time base is invalid, true otherwise
     */
    bool acquireTime(Fw::Time& stamp, F64& unixSeconds);

    /**
     * \brief Build an OrbitConfig from the current parameter values.
     *
     * \details Fields whose parameter is not VALID keep the OrbitConfig
     * default.
     *
     * \return Time-scale offsets and staleness policy for solveOrbit()
     */
    OrbitConfig currentConfig();

    /**
     * \brief Pack the information from an OrbitSolution into an OrbitState to be broadcast.
     *
     * \details Time fields are copied on every path, position fields
     * only when the result carried a position.
     *
     * \param sol     Products of this cycle
     * \param result  How far the cycle got
     * \param stamp   Wall-clock instant the solution describes
     * \param state   [out] State to populate
     */
    static void fillState(const OrbitSolution& sol,
                          OrbitResult result,
                          const Fw::Time& stamp,
                          Gnc::OrbitState& state);

    /**
     * \brief Stage 3. Write telemetry, including the ground track.
     *
     * \details
     *
     * \param state  Fully populated orbit state for this cycle
     */
    void publish(const Gnc::OrbitState& state);

    /**
     * \brief Stage 4. Broadcast the state on orbitOut.
     *
     * \details Fans out synchronously on this thread, so every consumer
     * of a given cycle sees the same state.
     *
     * \param state  Orbit state to broadcast, valid or not
     */
    void emit(const Gnc::OrbitState& state);

    /**
     * \brief Map the propagator's enum to the FPP enum.
     *
     * \param result  Propagator result code
     * \return Equivalent Gnc::OrbitValidity
     */
    static Gnc::OrbitValidity toFppValidity(OrbitResult result);

    //! The loaded TLE and the SGP4 record built from it.
    Sgp4Propagator m_sgp4;
};

}  // namespace Environment
}  // namespace Gnc

#endif
