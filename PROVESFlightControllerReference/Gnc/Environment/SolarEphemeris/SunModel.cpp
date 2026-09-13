/**
 * \file SunModel.cpp
 * \brief Solar ephemeris implementation.
 */
#include "PROVESFlightControllerReference/Gnc/Environment/SolarEphemeris/SunModel.hpp"

namespace Gnc {
namespace Environment {

/**
 * \brief Evaluate the solar model and rotate MOD -> TEME at one instant.
 *
 * \details Computationally expensive, so solarDirectionTeme() calls it
 * twice per segment rather than once per evaluation.
 *
 * \param tUt1      Julian centuries of UT1 since J2000
 * \param tTt       Julian centuries of TT since J2000
 * \param unitTeme  [out] Geocentric unit vector Earth -> Sun, TEME
 * \param rangeKm   [out] Earth-Sun distance, km
 */
void sunTemeAt(double tUt1, double tTt, Astro::Vec3& unitTeme, double& rangeKm) {

    // Solar ephemeris. Output in MOD: mean equator, mean equinox of date.
    const Astro::SunState sun = Astro::sunLowPrecisionMod(tUt1, tTt);

    // Rotate MOD -> TOD -> TEME 
    const Astro::Nutation nut = Astro::nutation1980(tTt);

    unitTeme = Astro::modToTeme(sun.unitMod, nut);
    rangeKm = sun.rangeKm;
}

/**
 * \brief Solar direction in TEME, interpolated across a cached segment.
 *
 * \param jdUt1    UT1 Julian date, flattened
 * \param jdTt     TT Julian date, flattened
 * \param cfg      Segment length
 * \param seg      [in,out] Interpolation cache, re-anchored as needed
 * \param rangeKm  [out] Earth-Sun distance, km, interpolated linearly
 * \return Geocentric unit vector Earth -> Sun, TEME
 */
Astro::Vec3 solarDirectionTeme(double jdUt1,
                               double jdTt,
                               const SolarConfig& cfg,
                               SunSegment& seg,
                               double& rangeKm) {
    /*
     * Reconstruct the centuries-from-J2000 arguments from the flattened
     * JDs supplied by the propagator.
     */
    const double tUt1 = (jdUt1 - Astro::JD_J2000) / Astro::DAYS_PER_JCENT;
    const double tTt = (jdTt - Astro::JD_J2000) / Astro::DAYS_PER_JCENT;

    if (cfg.segmentSec <= 0.0) {
        Astro::Vec3 u;
        sunTemeAt(tUt1, tTt, u, rangeKm);
        return u;
    }

    const double segmentDays = cfg.segmentSec / Astro::SECONDS_PER_DAY;

    /*
     * Re-anchor if we have no segment, if the clock stepped backwards,
     * or timestamp is past the segment end.
     */
    if (!seg.seeded || jdUt1 < seg.prevJdUt1 || jdUt1 >= seg.nextJdUt1) {
        const double dT = segmentDays / Astro::DAYS_PER_JCENT;

        sunTemeAt(tUt1, tTt, seg.prevTeme, seg.rangePrevKm);
        sunTemeAt(tUt1 + dT, tTt + dT, seg.nextTeme, seg.rangeNextKm);

        seg.prevJdUt1 = jdUt1;
        seg.nextJdUt1 = jdUt1 + segmentDays;
        seg.seeded = true;
    }

    // Great-circle interpolation across the segment
    const double span = seg.nextJdUt1 - seg.prevJdUt1;
    double frac = (span > 0.0) ? ((jdUt1 - seg.prevJdUt1) / span) : 0.0;
    if (frac < 0.0) {
        frac = 0.0;
    }
    if (frac > 1.0) {
        frac = 1.0;
    }

    rangeKm = seg.rangePrevKm + frac * (seg.rangeNextKm - seg.rangePrevKm);
    return Astro::vslerp(seg.prevTeme, seg.nextTeme, frac);
}

/**
 * \brief Run one full solar evaluation.
 *
 * \param query  Time and, when available, orbit state
 *
 * \param cfg    Interpolation tuning
 *
 * \param seg    [in,out] Interpolation cache, owned by the caller
 *
 * \param out    [out] Solar geometry. Nothing is written when NO_TIME
 *               is returned; only the direction fields are written when
 *               GEOCENTRIC is returned.
 *
 * \return How complete the evaluation was
 */
SolarResult solveSolar(const SolarQuery& query,
                       const SolarConfig& cfg,
                       SunSegment& seg,
                       SolarSolution& out) {
    /*
     * Only a bad clock stops us. Everything else still carries a usable
     * jdUt1 and jdTt, and the solar model needs nothing more.
     */
    if (!query.timeUsable) {
        return SolarResult::NO_TIME;
    }

    // Step 1: Sun direction
    double sunRangeKm = 0.0;
    const Astro::Vec3 sunUnitGeo = solarDirectionTeme(query.jdUt1, query.jdTt, cfg, seg, sunRangeKm);

    // Geocentric fallback when position is invalid.
    if (!query.positionUsable) {
        out.sunUnitTeme = sunUnitGeo;
        out.sunRangeKm = sunRangeKm;
        out.directionUsable = true;
        out.geometryUsable = false;
        return SolarResult::GEOCENTRIC;
    }

    // Step 2: Geometry
    const Astro::Vec3 sunPosTeme = Astro::vscale(sunUnitGeo, sunRangeKm);

    // Parallax: the Sun vector from the spacecraft.
    double scSunRangeKm = 0.0;
    const Astro::Vec3 sunUnitSc = Astro::sunUnitFromSpacecraft(query.posTemeKm, sunPosTeme, scSunRangeKm);

    out.sunUnitTeme = sunUnitSc;
    out.sunRangeKm = scSunRangeKm;
    out.illumination = Astro::shadowConical(query.posTemeKm, sunPosTeme);
    out.betaRad = Astro::betaAngleRad(query.posTemeKm, query.velTemeKmS, sunUnitSc);
    out.directionUsable = true;
    out.geometryUsable = true;

    return SolarResult::VALID;
}

}  // namespace Environment
}  // namespace Gnc
