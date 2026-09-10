/**
 * \file SolarEphemeris.cpp
 * \brief Per-evaluation pipeline, triggered by OrbitState arrival:
 *
 * \details   
 *   1. solarDirectionTeme()  jdUt1 -> Sun unit vector, TEME (cached). Needs only a valid clock.
 *   2. geometry + position -> parallax, shadow, beta. Needs a valid position.
 *   3. emit() -> sunRefOut, solarOut, telemetry
 */

#include "PROVESFlightControllerReference/Gnc/Environment/SolarEphemeris/SolarEphemeris.hpp"

#include "PROVESFlightControllerReference/Gnc/Types/GncConvert.hpp"

namespace Gnc {
namespace Environment {


/**
 * \brief Construct the component with an empty interpolation cache.
 * \param compName  F Prime component instance name
 */
SolarEphemeris::SolarEphemeris(const char* const compName)
    : SolarEphemerisComponentBase(compName) {}

//! Destroy the component. Holds no resources.
SolarEphemeris::~SolarEphemeris() {}

/*
 * ============================================================================
 * Stage 1: solar direction
 * ============================================================================
 */

/**
 * \brief Evaluate the solar model and rotate MOD -> TEME at one instant.
 *
 * \param tUt1      Julian centuries of UT1 since J2000
 * \param tTt       Julian centuries of TT since J2000
 * \param unitTeme  [out] Geocentric unit vector Earth -> Sun, TEME
 * \param rangeKm   [out] Earth-Sun distance, km
 */
void SolarEphemeris::sunTemeAt(F64 tUt1, F64 tTt, Astro::Vec3& unitTeme, F64& rangeKm) {
    /*
     * Low-precision solar ephemeris. Output is in MOD: mean equator,
     * mean equinox of date.
     */
    const Astro::SunState sun = Astro::sunLowPrecisionMod(tUt1, tTt);

    /*
     * Rotate MOD -> TOD -> TEME so the result shares a frame with the
     * SGP4 output.
     */
    const Astro::Nutation nut = Astro::nutation1980(tTt);

    unitTeme = Astro::modToTeme(sun.unitMod, nut);
    rangeKm = sun.rangeKm;
}

/**
 * \brief Solar direction in TEME, interpolated across a cached segment.
 *
 * \param jdUt1    UT1 Julian date, flattened
 * \param jdTt     TT Julian date, flattened
 * \param rangeKm  [out] Earth-Sun distance, km
 * \return Geocentric unit vector Earth -> Sun, TEME
 */
Astro::Vec3 SolarEphemeris::solarDirectionTeme(F64 jdUt1, F64 jdTt, F64& rangeKm) {
    /*
     * Reconstruct the centuries-from-J2000 arguments from the flattened
     * JDs supplied by the propagator.
     */
    const F64 tUt1 = (jdUt1 - Astro::JD_J2000) / Astro::DAYS_PER_JCENT;
    const F64 tTt  = (jdTt  - Astro::JD_J2000) / Astro::DAYS_PER_JCENT;

    Fw::ParamValid valid = Fw::ParamValid::INVALID;
    const F64 rawSeg = this->paramGet_SUN_SEGMENT_SEC(valid);
    const F64 segmentSec = (valid == Fw::ParamValid::VALID) ? rawSeg : 60.0;

    if (segmentSec <= 0.0) {
        Astro::Vec3 u;
        sunTemeAt(tUt1, tTt, u, rangeKm);
        return u;
    }

    const F64 segmentDays = segmentSec / Astro::SEC_PER_DAY;

    /*
     * Re-anchor if we have no segment, if the clock stepped backwards,
     * or if we have run off the end of the current segment.
     */
    if (!m_sunSeeded || jdUt1 < m_sunPrevJdUt1 || jdUt1 >= m_sunNextJdUt1) {
        const F64 dT = segmentDays / Astro::DAYS_PER_JCENT;

        sunTemeAt(tUt1, tTt, m_sunPrevTeme, m_sunRangePrevKm);
        sunTemeAt(tUt1 + dT, tTt + dT, m_sunNextTeme, m_sunRangeNextKm);

        m_sunPrevJdUt1 = jdUt1;
        m_sunNextJdUt1 = jdUt1 + segmentDays;
        m_sunSeeded = true;
    }

    /*
     * Great-circle interpolation across the segment. The Sun sweeps
     * ~0.00066 deg in 60 s, so SLERP error is far below model error.
     */
    const F64 span = m_sunNextJdUt1 - m_sunPrevJdUt1;
    F64 frac = (span > 0.0) ? ((jdUt1 - m_sunPrevJdUt1) / span) : 0.0;
    if (frac < 0.0) { frac = 0.0; }
    if (frac > 1.0) { frac = 1.0; }

    rangeKm = m_sunRangePrevKm + frac * (m_sunRangeNextKm - m_sunRangePrevKm);
    return Astro::vslerp(m_sunPrevTeme, m_sunNextTeme, frac);
}

/*
 * ============================================================================
 * Orbit state arrival
 * ============================================================================
 */

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

    Gnc::SolarState solar;
    solar.set_validity(Gnc::SolarValidity::NO_TIME);
    solar.set_directionUsable(false);
    solar.set_geometryUsable(false);


    if (!state.get_timeUsable()) {
        this->log_WARNING_HI_TimeMissing();
        this->emit(solar, stamp);
        return;
    }

    // Step 1: Sun direction
    F64 sunRangeKm = 0.0;
    const Astro::Vec3 sunUnitGeo =
        this->solarDirectionTeme(state.get_jdUt1(), state.get_jdTt(), sunRangeKm);
    const Astro::Vec3 sunPosTeme = Astro::vscale(sunUnitGeo, sunRangeKm);

    // Geocentric fallback when position is invalid.
    if (!state.get_positionUsable()) {
        this->log_WARNING_LO_GeocentricFallback();
        solar.set_validity(Gnc::SolarValidity::GEOCENTRIC);
        solar.set_directionUsable(true);
        solar.set_sunUnitTeme(toVec3d(sunUnitGeo));
        solar.set_sunRangeKm(sunRangeKm);
        this->emit(solar, stamp);
        return;
    }

    // Step 2: Geometry
    const Astro::Vec3 posTemeKm = toAstro(state.get_posTemeKm());
    const Astro::Vec3 velTemeKmS = toAstro(state.get_velTemeKmS());

    // Parallax: the Sun vector from the spacecraft.
    F64 scSunRangeKm = 0.0;
    const Astro::Vec3 sunUnitSc =
        Astro::sunUnitFromSpacecraft(posTemeKm, sunPosTeme, scSunRangeKm);

    const Astro::Illumination illum = Astro::shadowConical(posTemeKm, sunPosTeme);
    const F64 betaRad = Astro::betaAngleRad(posTemeKm, velTemeKmS, sunUnitSc);
  
    //Edge-triggered eclipse events.
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

    solar.set_validity(Gnc::SolarValidity::VALID);
    solar.set_directionUsable(true);
    solar.set_geometryUsable(true);
    solar.set_sunUnitTeme(toVec3d(sunUnitSc));
    solar.set_sunRangeKm(scSunRangeKm);
    solar.set_illumination(toFpp(illum));
    solar.set_betaRad(betaRad);

    this->emit(solar, stamp);
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
        // Radians in the payload, degrees at the telemetry boundary.
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
    m_sunSeeded = false;
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Environment
}  // namespace Gnc
