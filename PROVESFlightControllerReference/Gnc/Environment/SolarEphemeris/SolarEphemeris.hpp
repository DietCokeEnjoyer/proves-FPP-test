/**
 * \file SolarEphemeris.hpp
 *
 * \brief Solar direction, eclipse state and beta angle.
 */
#ifndef Gnc_Environment_SolarEphemeris_HPP
#define Gnc_Environment_SolarEphemeris_HPP

#include "PROVESFlightControllerReference/Gnc/Environment/SolarEphemeris/SolarEphemerisComponentAc.hpp"
#include "PROVESFlightControllerReference/Gnc/Environment/SolarEphemeris/SunModel.hpp"

namespace Gnc {
namespace Environment {

/**
 * \brief Framework wrapper around the solar model.
 *
 * \details Produces a reference sun vector.
 * Driven by the arrival of OrbitState.
 * Framework wrapper: Reads parameters, holds the solar interpolation cache, 
 * and handles events and telemetry. 
 * Solar math lives in SunModel.cpp/hpp
 */
class SolarEphemeris final : public SolarEphemerisComponentBase {
  public:
    /**
     * \brief Construct the component with an empty interpolation cache.
     * \param compName  F Prime component instance name
     */
    explicit SolarEphemeris(const char* const compName);

    //! Destroy the component. Holds no resources.
    ~SolarEphemeris();

  private:
    /**
     * \brief Orbit state arrived. Triggers one evaluation.
     *
     * \details Only an unusable clock stops the evaluation. An
     * unusable position downgrades to a geocentric solution, still valid
     * for attitude determination.
     *
     * \param portNum  Port index, unused (single port)
     * \param state    Orbit state from OrbitPropagator
     */
    void orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) override;

    /**
     * \brief Discard the interpolation cache and force a fresh
     *        evaluation on the next orbit state.
     *
     * \param opCode  Command opcode
     * \param cmdSeq  Command sequence number
     */
    void RESYNC_SUN_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;

    /*
     * ============================================================================
     * Internal helpers
     * ============================================================================
     */

    /**
     * \brief Build a SolarConfig from the current parameter values.
     *
     * \details Falls back to the SolarConfig default when
     * SUN_SEGMENT_SEC is not VALID. 
     * 
     * Default must match the FPP in case the parameter database returns invalid.
     *
     * \return Interpolation tuning to pass to solveSolar()
     */
    SolarConfig currentConfig();

    /**
     * \brief Trigger EclipseEntry / EclipseExit events on illumination changes.
     *
     * \details Edge triggered to prevent log flooding.
     *
     * \param illum    Illumination from this evaluation
     * \param betaRad  Beta angle at the transition, radians
     */
    void reportIllumination(Astro::Illumination illum, F64 betaRad);

    /**
     * \brief Publish on both output ports and update telemetry.
     *
     * \details sunRefOut carries the direction for TRIAD; solarOut
     * carries the full state. Telemetry channels are written only for 
     * the fields the current validity level supports.
     *
     * \param solar  State to publish, valid or not
     * \param stamp  Epoch of the orbit state.
     */
    void emit(const Gnc::SolarState& solar, const Fw::Time& stamp);

    /**
     * \brief Map the model's to the FPP enum.
     *
     * \param result  Solar model result code
     * \return Equivalent Gnc::SolarValidity
     */
    static Gnc::SolarValidity toFppValidity(SolarResult result);

    /*
     * ============================================================================
     * State
     * ============================================================================
     */

    /**
     * Endpoints of the current interpolation segment. Modified by
     * orbitIn and RESYNC_SUN, both of which are guarded.
     */
    SunSegment m_sunSegment;

    //! Previous illumination, for edge-triggered eclipse events.
    Astro::Illumination m_lastIllum = Astro::Illumination::SUNLIT;
    bool m_illumSeeded = false;  //!< Suppresses an event on the first evaluation
};

}  // namespace Environment
}  // namespace Gnc

#endif
