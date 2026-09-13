/**
 * \file SolarEphemeris.cpp
 * \brief FPrime wrapper component for solar ephemeris evaluation.
 *
 * \details
 *   Pipeline on OrbitState arrival:
 *   1. Check the interpolation cache length parameter: currentConfig()
 *   2. Evaluate the solar ephemeris: solveSolar() 
 *   3. Publish the results of the evaluation on output ports and telemetry: emit()
 */

#include "PROVESFlightControllerReference/Gnc/Environment/SolarEphemeris/SolarEphemeris.hpp"

#include "PROVESFlightControllerReference/Gnc/Types/GncConvert.hpp"

namespace Gnc {
namespace Environment {

/**
 * \brief Construct the component with an empty interpolation cache.
 * \param compName  F Prime component instance name
 */
SolarEphemeris::SolarEphemeris(const char* const compName) : SolarEphemerisComponentBase(compName) {}

//! Destroy the component. Holds no resources.
SolarEphemeris::~SolarEphemeris() {}


/**
 * \brief Orbit state arrived. Runs one full solar evaluation.
 *
 * \details Three outcomes, reported through SolarValidity: NO_TIME when
 * the clock is unusable, GEOCENTRIC when there is a time but no
 * position, and VALID when both are available. Every path emits.
 *
 * \param portNum  Port index, unused
 * \param state    Orbit state from OrbitPropagator
 */
void SolarEphemeris::orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) {
    (void)portNum;

    const Fw::Time stamp = state.get_stamp();

    SolarQuery query;
    query.jdUt1 = state.get_jdUt1();
    query.jdTt = state.get_jdTt();
    query.posTemeKm = toAstro(state.get_posTemeKm());
    query.velTemeKmS = toAstro(state.get_velTemeKmS());
    query.timeUsable = state.get_timeUsable();
    query.positionUsable = state.get_positionUsable();

    SolarSolution sol;
    const SolarResult result = solveSolar(query, this->currentConfig(), m_sunSegment, sol);

    Gnc::SolarState solar;
    solar.set_validity(toFppValidity(result));
    solar.set_directionUsable(sol.directionUsable);
    solar.set_geometryUsable(sol.geometryUsable);

    switch (result) {
        case SolarResult::NO_TIME:
            this->log_WARNING_HI_TimeMissing();
            this->emit(solar, stamp);
            return;

        case SolarResult::GEOCENTRIC:
            // Running without an orbit: direction only, no shadow or beta.
            this->log_WARNING_LO_GeocentricFallback();
            solar.set_sunUnitTeme(toVec3f(sol.sunUnitTeme));
            solar.set_sunRangeKm(sol.sunRangeKm);
            this->emit(solar, stamp);
            return;

        case SolarResult::VALID:
        default:
            break;
    }

    this->reportIllumination(sol.illumination, sol.betaRad);

    solar.set_sunUnitTeme(toVec3f(sol.sunUnitTeme));
    solar.set_sunRangeKm(sol.sunRangeKm);
    solar.set_illumination(toFpp(sol.illumination));
    solar.set_betaRad(sol.betaRad);

    this->emit(solar, stamp);
}

/*
 * ============================================================================
 * Configuration and fault reporting
 * ============================================================================
 */

/**
 * \brief Read SUN_SEGMENT_SEC into a SolarConfig.
 *
 * \return Interpolation tuning, defaulted when the parameter is not VALID
 */
SolarConfig SolarEphemeris::currentConfig() {
    SolarConfig cfg;

    Fw::ParamValid valid = Fw::ParamValid::INVALID;
    const F64 rawSeg = this->paramGet_SUN_SEGMENT_SEC(valid);
    if (valid == Fw::ParamValid::VALID) {
        cfg.segmentSec = rawSeg;
    }

    return cfg;
}

/**
 * \brief Trigger an eclipse event on an illumination change.
 *
 * \param illum    Illumination from this evaluation
 * \param betaRad  Beta angle at the transition, radians
 */
void SolarEphemeris::reportIllumination(Astro::Illumination illum, F64 betaRad) {

    if (m_illumSeeded && (illum != m_lastIllum)) {
        const bool wasLit = (m_lastIllum == Astro::Illumination::SUNLIT);
        const bool isLit = (illum == Astro::Illumination::SUNLIT);
        if (wasLit && !isLit) {
            this->log_ACTIVITY_LO_EclipseEntry(static_cast<F32>(betaRad * Astro::RAD2DEG));
        } else if (!wasLit && isLit) {
            this->log_ACTIVITY_LO_EclipseExit(static_cast<F32>(betaRad * Astro::RAD2DEG));
        }
    }
    m_lastIllum = illum;
    m_illumSeeded = true;
}

/**
 * \brief Map SolarResult to the FPP validity enum.
 *
 * \param result  Solar model result code
 * \return Equivalent Gnc::SolarValidity
 */
Gnc::SolarValidity SolarEphemeris::toFppValidity(SolarResult result) {
    switch (result) {
        case SolarResult::VALID:
            return Gnc::SolarValidity::VALID;
        case SolarResult::GEOCENTRIC:
            return Gnc::SolarValidity::GEOCENTRIC;
        case SolarResult::NO_TIME:
        default:
            return Gnc::SolarValidity::NO_TIME;
    }
}

/*
 * ============================================================================
 * Publication
 * ============================================================================
 */

/**
 * \brief Publish on sunRefOut and solarOut, then write telemetry.
 *
 * \param solar  State to publish, valid or not
 * \param stamp  Epoch of the orbit state
 */
void SolarEphemeris::emit(const Gnc::SolarState& solar, const Fw::Time& stamp) {
    if (this->isConnected_sunRefOut_OutputPort(0)) {
        Gnc::VectorSample sample;
        sample.set_vec(solar.get_sunUnitTeme());
        sample.set_frame(Gnc::FrameId::TEME);
        sample.set_stamp(stamp);

        sample.set_valid(solar.get_directionUsable());
        this->sunRefOut_out(0, sample);
    }

    if (this->isConnected_solarOut_OutputPort(0)) {
        Gnc::SolarState copy = solar;
        this->solarOut_out(0, copy);
    }

    this->tlmWrite_Validity(solar.get_validity());
    this->tlmWrite_GeometryUsable(solar.get_geometryUsable());
    if (solar.get_directionUsable()) {
        this->tlmWrite_SunUnitTeme(solar.get_sunUnitTeme());
        this->tlmWrite_SunRangeKm(solar.get_sunRangeKm());
    }
    if (solar.get_geometryUsable()) {
        // Radians in the payload, degrees for telemetry.
        this->tlmWrite_BetaDeg(static_cast<F32>(solar.get_betaRad() * Astro::RAD2DEG));
        this->tlmWrite_Illumination(solar.get_illumination());
    }
}

/*
 * ============================================================================
 * Commands
 * ============================================================================
 */

/**
 * \brief Drop the interpolation cache so the next evaluation re-anchors.
 *
 * \param opCode  Command opcode
 * \param cmdSeq  Command sequence number
 */
void SolarEphemeris::RESYNC_SUN_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_sunSegment.resync();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Environment
}  // namespace Gnc
