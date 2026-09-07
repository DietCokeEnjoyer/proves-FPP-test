/**
 * \file SolarEphemeris.cpp
 * \brief Per-evaluation pipeline, triggered by OrbitState arrival:
 *
 * \details   1. solarDirectionTeme()  jdUt1 -> Sun unit vector, TEME (cached)
 *   2. geometry              + position -> parallax, shadow, beta
 *   3. emit()                -> sunRefOut, solarOut, telemetry
 *
 * Stage 1 needs only a clock. Stage 2 needs an orbit. That split is
 * what lets this component keep producing a usable attitude reference
 * when no TLE has ever been uploaded.
 *
 * NOTE ON TIME: this file contains no call to getTime() outside of
 * framework timestamping. jdUt1 and the observation stamp both arrive
 * in OrbitState. If you find yourself wanting the clock here, the
 * quantity you want almost certainly belongs in OrbitState instead.
 */
#include "PROVESFlightControllerReference/Gnc/Environment/SolarEphemeris/SolarEphemeris.hpp"

#include "PROVESFlightControllerReference/Gnc/Types/GncConvert.hpp"

namespace Gnc {
namespace Environment {


SolarEphemeris::SolarEphemeris(const char* const compName)
    : SolarEphemerisComponentBase(compName) {}

SolarEphemeris::~SolarEphemeris() {}

/*
 * ============================================================================
 * Stage 1: solar direction
 * ============================================================================
 */

void SolarEphemeris::sunTemeAt(F64 tUt1, F64 tTt, Astro::Vec3& unitTeme, F64& rangeKm) {
    /*
     * Low-precision solar ephemeris. Output is in MOD: mean equator,
     * mean equinox of date.
     */
    const Astro::SunState sun = Astro::sunLowPrecisionMod(tUt1, tTt);

    /*
     * Rotate MOD -> TOD -> TEME so the result shares a frame with the
     * SGP4 output. ~20 arcsec of rotation -- smaller than the solar
     * model's own 0.01 deg error, so strictly optional, but it costs
     * one nutation evaluation per segment rather than per tick.
     */
    const Astro::Nutation nut = Astro::nutation1980(tTt);

    unitTeme = Astro::modToTeme(sun.unitMod, nut);
    rangeKm = sun.rangeKm;
}

Astro::Vec3 SolarEphemeris::solarDirectionTeme(F64 jdUt1, F64 jdTt, F64& rangeKm) {
    /*
     * Reconstruct the centuries-from-J2000 arguments from the flattened
     * JDs supplied by the propagator. A flattened JD carries ~50 us of
     * resolution, which is ~5e-10 deg of solar motion -- utterly below
     * the model error, so nothing is lost by not passing split JDs.
     *
     * BOTH scales come from OrbitState. UT1 drives Earth rotation and
     * the Sun's mean longitude; TT drives the dynamical arguments
     * (nutation, obliquity, mean anomaly). They differ by ~69 s. This
     * used to approximate tTt = tUt1 because the split had dropped TT
     * from the state -- correct to ~0.0008 deg, but silently wrong for
     * no reason when the propagator had already computed it.
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
     * ~0.00066 deg in 60 s, so SLERP error is far below model error
     * while transcendental cost drops by the segment length.
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

void SolarEphemeris::orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) {
    (void)portNum;

    const Fw::Time stamp = state.get_stamp();

    Gnc::SolarState solar;
    solar.set_validity(Gnc::SolarValidity::NO_TIME);
    solar.set_directionUsable(false);
    solar.set_geometryUsable(false);

    /*
     * Only a bad clock stops us. Everything else still carries a usable
     * jdUt1 and jdTt, and the solar model needs nothing more.
     */
    if (!state.get_timeUsable()) {
        this->log_WARNING_HI_TimeMissing();
        this->emit(solar, stamp);
        return;
    }

    // ---- Stage 1: Sun direction (needs only a clock) -----------------
    F64 sunRangeKm = 0.0;
    const Astro::Vec3 sunUnitGeo =
        this->solarDirectionTeme(state.get_jdUt1(), state.get_jdTt(), sunRangeKm);
    const Astro::Vec3 sunPosTeme = Astro::vscale(sunUnitGeo, sunRangeKm);

    if (!state.get_positionUsable()) {
        /*
         * Geocentric fallback. The parallax being skipped is at most
         * 0.0027 deg in LEO, well under the solar model's own 0.01 deg,
         * so this stays fully usable for attitude determination.
         * Shadow and beta genuinely cannot be computed without a
         * position, so geometryUsable stays false rather than shipping
         * a guessed number behind a single "valid" flag.
         */
        this->log_WARNING_LO_GeocentricFallback();
        solar.set_validity(Gnc::SolarValidity::GEOCENTRIC);
        solar.set_directionUsable(true);
        solar.set_sunUnitTeme(toVec3d(sunUnitGeo));
        solar.set_sunRangeKm(sunRangeKm);
        this->emit(solar, stamp);
        return;
    }

    // ---- Stage 2: geometry (needs the orbit) -------------------------
    const Astro::Vec3 posTemeKm = toAstro(state.get_posTemeKm());
    const Astro::Vec3 velTemeKmS = toAstro(state.get_velTemeKmS());

    // Parallax: the Sun vector FROM THE SPACECRAFT.
    F64 scSunRangeKm = 0.0;
    const Astro::Vec3 sunUnitSc =
        Astro::sunUnitFromSpacecraft(posTemeKm, sunPosTeme, scSunRangeKm);

    const Astro::Illumination illum = Astro::shadowConical(posTemeKm, sunPosTeme);
    const F64 betaRad = Astro::betaAngleRad(posTemeKm, velTemeKmS, sunUnitSc);

    /*
     * Edge-triggered eclipse events. Level-triggered would flood the
     * log at 1 Hz; the ground only cares about the transitions.
     */
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

void SolarEphemeris::emit(const Gnc::SolarState& solar, const Fw::Time& stamp) {
    if (this->isConnected_sunRefOut_OutputPort(0)) {
        Gnc::VectorSample sample;
        sample.set_vec(solar.get_sunUnitTeme());
        sample.set_frame(Gnc::FrameId::TEME);
        /*
         * The ORBIT STATE's epoch, not "now". This component never
         * reads the clock, so the attitude chain's staleness check
         * measures the age of the underlying observation.
         */
        sample.set_stamp(stamp);

        /*
         * In eclipse the Sun direction is still geometrically correct;
         * the spacecraft simply cannot MEASURE it. Marking the model
         * output invalid would be wrong -- that is the sun sensor's
         * job, and TRIAD already handles a missing body vector.
         */
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

void SolarEphemeris::RESYNC_SUN_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_sunSeeded = false;
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Environment
}  // namespace Gnc
