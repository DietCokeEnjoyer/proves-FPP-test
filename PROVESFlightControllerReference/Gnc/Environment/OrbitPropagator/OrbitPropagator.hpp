/**
 * \file OrbitPropagator.hpp
 * \brief SGP4 propagation and time scales. The navigation product.
 */
#ifndef Gnc_Environment_OrbitPropagator_HPP
#define Gnc_Environment_OrbitPropagator_HPP

#include "Gnc/Environment/OrbitPropagator/OrbitPropagatorComponentAc.hpp"
#include "Gnc/AstroLib/AstroLib.hpp"

#include <perturb/perturb.hpp>

namespace Gnc {
namespace Environment {

class OrbitPropagator final : public OrbitPropagatorComponentBase {
  public:
    explicit OrbitPropagator(const char* const compName);
    ~OrbitPropagator();

  private:
    void run_handler(FwIndexType portNum, U32 context) override;

    void LOAD_TLE_cmdHandler(FwOpcodeType opCode,
                             U32 cmdSeq,
                             const Fw::CmdStringArg& line1,
                             const Fw::CmdStringArg& line2) override;

    void CLEAR_TLE_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    /**
     * Read the time port and build every needed time scale.
     *
     * THE ONLY PLACE IN THE GNC CHAIN THAT READS THE CLOCK FOR
     * COMPUTATION. Everything downstream receives jdUt1, gmstRad and
     * the Fw::Time stamp inside OrbitState.
     */
    bool acquireTime(Astro::TimeScales& ts, Fw::Time& stamp);

    //! SGP4. Returns false on propagator error.
    bool propagate(const Astro::TimeScales& ts,
                   Astro::Vec3& posTeme,
                   Astro::Vec3& velTeme,
                   F64& ageDays);

    //! Telemetry, including the ground track.
    void publish(const Gnc::OrbitState& state);

    //! Broadcast on orbitOut. Fans out synchronously on this thread.
    void emit(const Gnc::OrbitState& state);

    /**
     * The SGP4 orbital record. ~500 bytes, held as a member so it
     * never lands on the task stack. perturb::Satellite has no default
     * constructor, so it is seeded with a value-initialized elsetrec
     * and gated by m_tleValid.
     */
    perturb::Satellite m_sat;

    /**
     * In-class initialiser, matching the rest of the subsystem: the
     * default sits next to the declaration, so adding a member cannot
     * silently leave it uninitialised. (m_sat is the exception --
     * perturb::Satellite has no default constructor, so it must be
     * seeded in the constructor initialiser list.)
     */
    bool m_tleValid = false;
};

}  // namespace Environment
}  // namespace Gnc

#endif
