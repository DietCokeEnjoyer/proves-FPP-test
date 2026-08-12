// ======================================================================
// \file   SolarEphemeris.cpp
//
// Per-evaluation pipeline, triggered by OrbitState arrival:
//
//   1. solarDirectionTeme()  jdUt1 -> Sun unit vector, TEME (cached)
//   2. geometry              + position -> parallax, shadow, beta
//   3. emit()                -> sunRefOut, solarOut, telemetry
//
// Stage 1 needs only a clock. Stage 2 needs an orbit. That split is
// what lets this component keep producing a usable attitude reference
// when no TLE has ever been uploaded.
//
// NOTE ON TIME: this file contains no call to getTime() outside of
// framework timestamping. jdUt1 and the observation stamp both arrive
// in OrbitState. If you find yourself wanting the clock here, the
// quantity you want almost certainly belongs in OrbitState instead.
// ======================================================================
#include "Gnc/Environment/SolarEphemeris/SolarEphemeris.hpp"

namespace Gnc {
namespace Environment {

namespace {

Gnc::Vector3 toFpp(const Astro::Vec3& v) {
    return Gnc::Vector3(v.x, v.y, v.z);
}

//! Narrow an F64 direction to the F32 attitude-chain type. A unit
//! vector in F32 carries ~2e-5 deg of direction error, four orders of
//! magnitude below the best sun sensor.
Gnc::Vec3f toVec3f(const Astro::Vec3& v) {
    Gnc::Vec3f out;
    out[0] = static_cast<F32>(v.x);
    out[1] = static_cast<F32>(v.y);
    out[2] = static_cast<F32>(v.z);
    return out;
}

Gnc::IlluminationState toFpp(Astro::Illumination i) {
    switch (i) {
        case Astro::Illumination::UMBRA:    return Gnc::IlluminationState::UMBRA;
        case Astro::Illumination::PENUMBRA: return Gnc::IlluminationState::PENUMBRA;
        default:                            return Gnc::IlluminationState::SUNLIT;
    }
}

}  // namespace

SolarEphemeris::SolarEphemeris(const char* const compName)
    : SolarEphemerisComponentBase(compName),
      m_sunPrev { 1.0, 0.0, 0.0 },
      m_sunNext { 1.0, 0.0, 0.0 },
      m_sunRangePrevKm(Astro::AU_KM),
      m_sunRangeNextKm(Astro::AU_KM),
      m_sunPrevJd(0.0),
      m_sunNextJd(0.0),
      m_sunSeeded(false),
      m_lastIllum(Astro::Illumination::SUNLIT),
      m_illumSeeded(false) {}

SolarEphemeris::~SolarEphemeris() {}

// ----------------------------------------------------------------------
// Stage 1: solar direction
// ----------------------------------------------------------------------

void SolarEphemeris::sunTemeAt(F64 tUt1, F64 tTt, Astro::Vec3& unitTeme, F64& rangeKm) {
    // Low-precision solar ephemeris. Output is in MOD: mean equator,
    // mean equinox of date.
    const Astro::SunState sun = Astro::sunLowPrecisionMod(tUt1, tTt);

    // Rotate MOD -> TOD -> TEME so the result shares a frame with the
    // SGP4 output. ~20 arcsec of rotation -- smaller than the solar
    // model's own 0.01 deg error, so strictly optional, but it costs
    // one nutation evaluation per segment rather than per tick.
    const Astro::Nutation nut = Astro::nutation1980(tTt);

    unitTeme = Astro::modToTeme(sun.unitMod, nut);
    rangeKm = sun.rangeKm;
}

Astro::Vec3 SolarEphemeris::solarDirectionTeme(F64 jdUt1, F64& rangeKm) {
    // Reconstruct the centuries-from-J2000 arguments from the flattened
    // JD supplied by the propagator. The flattened JD carries ~50 us of
    // resolution, which is ~5e-10 deg of solar motion -- utterly below
    // the model error, so nothing is lost by not passing a split JD.
    const F64 tUt1 = (jdUt1 - Astro::JD_J2000) / Astro::DAYS_PER_JCENT;

    // TT vs UT1 differ by ~69 s. The Sun moves 0.0417 deg/hour, so that
    // is 0.0008 deg -- below the model error, but free to carry.
    const F64 tTt = tUt1;

    Fw::ParamValid valid = Fw::ParamValid::INVALID;
    const F64 rawSeg = this->paramGet_SUN_SEGMENT_SEC(valid);
    const F64 segmentSec = (valid == Fw::ParamValid::VALID) ? rawSeg : 60.0;

    if (segmentSec <= 0.0) {
        Astro::Vec3 u;
        sunTemeAt(tUt1, tTt, u, rangeKm);
        return u;
    }

    const F64 segmentDays = segmentSec / Astro::SEC_PER_DAY;

    // Re-anchor if we have no segment, if the clock stepped backwards,
    // or if we have run off the end of the current segment.
    if (!m_sunSeeded || jdUt1 < m_sunPrevJd || jdUt1 >= m_sunNextJd) {
        const F64 dT = segmentDays / Astro::DAYS_PER_JCENT;

        sunTemeAt(tUt1, tTt, m_sunPrev, m_sunRangePrevKm);
        sunTemeAt(tUt1 + dT, tTt + dT, m_sunNext, m_sunRangeNextKm);

        m_sunPrevJd = jdUt1;
        m_sunNextJd = jdUt1 + segmentDays;
        m_sunSeeded = true;
    }

    // Great-circle interpolation across the segment. The Sun sweeps
    // ~0.00066 deg in 60 s, so SLERP error is far below model error
    // while transcendental cost drops by the segment length.
    const F64 span = m_sunNextJd - m_sunPrevJd;
    F64 frac = (span > 0.0) ? ((jdUt1 - m_sunPrevJd) / span) : 0.0;
    if (frac < 0.0) { frac = 0.0; }
    if (frac > 1.0) { frac = 1.0; }

    rangeKm = m_sunRangePrevKm + frac * (m_sunRangeNextKm - m_sunRangePrevKm);
    return Astro::vslerp(m_sunPrev, m_sunNext, frac);
}

// ----------------------------------------------------------------------
// Orbit state arrival
// ----------------------------------------------------------------------

void SolarEphemeris::orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) {
    (void)portNum;

    const Gnc::OrbitValidity v = state.get_validity();
    const Fw::Time stamp = state.get_stamp();

    Gnc::SolarState solar;
    solar.set_valid(false);
    solar.set_hasOrbit(false);

    // NO_TIME is the only state that stops us. Everything else carries
    // a usable jdUt1.
    if (v == Gnc::OrbitValidity::NO_TIME) {
        this->log_WARNING_HI_NoTimeAvailable();
        this->emit(solar, stamp);
        return;
    }

    // ---- Stage 1: Sun direction (needs only a clock) -----------------
    F64 sunRangeKm = 0.0;
    const Astro::Vec3 sunUnitGeo = this->solarDirectionTeme(state.get_jdUt1(), sunRangeKm);
    const Astro::Vec3 sunPosTeme = Astro::vscale(sunUnitGeo, sunRangeKm);

    const bool haveOrbit = (v == Gnc::OrbitValidity::VALID) ||
                           (v == Gnc::OrbitValidity::STALE);

    if (!haveOrbit) {
        // Geocentric fallback. In LEO the parallax being skipped is at
        // most 0.0027 deg, well under the solar model's own 0.01 deg,
        // so this remains fully usable for attitude determination.
        // Shadow and beta genuinely cannot be computed without a
        // position, so they are left flagged invalid rather than
        // guessed.
        this->log_WARNING_LO_GeocentricFallback();
        solar.set_valid(true);
        solar.set_hasOrbit(false);
        solar.set_sunUnitTeme(toFpp(sunUnitGeo));
        solar.set_sunRangeKm(sunRangeKm);
        this->emit(solar, stamp);
        return;
    }

    // ---- Stage 2: geometry (needs the orbit) -------------------------
    const Gnc::Vector3 p = state.get_posTeme();
    const Astro::Vec3 posTeme { p.get_x(), p.get_y(), p.get_z() };
    const Gnc::Vector3 vv = state.get_velTeme();
    const Astro::Vec3 velTeme { vv.get_x(), vv.get_y(), vv.get_z() };

    // Parallax: the Sun vector FROM THE SPACECRAFT. At most 0.0027 deg
    // in LEO, but it is three subtractions, and it matters for HEO or
    // cislunar.
    F64 scSunRangeKm = 0.0;
    const Astro::Vec3 sunUnitSc =
        Astro::sunUnitFromSpacecraft(posTeme, sunPosTeme, scSunRangeKm);

    const Astro::Illumination illum = Astro::shadowConical(posTeme, sunPosTeme);
    const F64 betaDeg = Astro::betaAngleRad(posTeme, velTeme, sunUnitSc) * Astro::RAD2DEG;

    // Edge-triggered eclipse events. Level-triggered would flood the
    // log at 1 Hz; the ground only cares about the transitions.
    if (m_illumSeeded && (illum != m_lastIllum)) {
        const bool wasLit = (m_lastIllum == Astro::Illumination::SUNLIT);
        const bool isLit = (illum == Astro::Illumination::SUNLIT);
        if (wasLit && !isLit) {
            this->log_ACTIVITY_LO_EclipseEntry(static_cast<F32>(betaDeg));
        } else if (!wasLit && isLit) {
            this->log_ACTIVITY_LO_EclipseExit(static_cast<F32>(betaDeg));
        }
    }
    m_lastIllum = illum;
    m_illumSeeded = true;

    solar.set_valid(true);
    solar.set_hasOrbit(true);
    solar.set_sunUnitTeme(toFpp(sunUnitSc));
    solar.set_sunRangeKm(scSunRangeKm);
    solar.set_illumination(toFpp(illum));
    solar.set_betaDeg(static_cast<F32>(betaDeg));

    this->emit(solar, stamp);
}

// ----------------------------------------------------------------------
// Publication
// ----------------------------------------------------------------------

void SolarEphemeris::emit(const Gnc::SolarState& solar, const Fw::Time& stamp) {
    if (this->isConnected_sunRefOut_OutputPort(0)) {
        const Gnc::Vector3 s = solar.get_sunUnitTeme();

        Gnc::VectorSample sample;
        sample.set_unitVec(toVec3f(Astro::Vec3 { s.get_x(), s.get_y(), s.get_z() }));
        sample.set_frame(Gnc::FrameId::TEME);
        // Stamped with the ORBIT STATE's epoch, not with "now". This
        // component never reads the clock, so the attitude chain's
        // staleness check is measuring the age of the underlying
        // observation rather than the age of this function call.
        sample.set_stamp(stamp);

        // In eclipse the Sun direction is still geometrically correct;
        // the spacecraft simply cannot MEASURE it. Marking the model
        // output invalid here would be wrong -- it is the sun sensor's
        // job to report no signal, and TRIAD already handles a missing
        // body vector.
        sample.set_valid(solar.get_valid());
        this->sunRefOut_out(0, sample);
    }

    if (this->isConnected_solarOut_OutputPort(0)) {
        Gnc::SolarState copy = solar;
        this->solarOut_out(0, copy);
    }

    this->tlmWrite_HasOrbit(solar.get_hasOrbit());
    if (solar.get_valid()) {
        this->tlmWrite_SunUnitTeme(solar.get_sunUnitTeme());
        this->tlmWrite_SunRangeKm(solar.get_sunRangeKm());
    }
    if (solar.get_hasOrbit()) {
        this->tlmWrite_BetaDeg(solar.get_betaDeg());
        this->tlmWrite_Illum(solar.get_illumination());
    }
}

// ----------------------------------------------------------------------
// Commands
// ----------------------------------------------------------------------

void SolarEphemeris::RESYNC_SUN_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    m_sunSeeded = false;
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Environment
}  // namespace Gnc
