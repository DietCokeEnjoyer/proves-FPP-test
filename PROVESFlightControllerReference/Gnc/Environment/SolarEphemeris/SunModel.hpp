/**
 * \file SunModel.hpp
 * \brief Solar direction, eclipse state and beta angle.
 *
 * \details No FPrime. Uses the AstroLib solar model and geometry.
 * 
 * Work is in two steps:
 * 1: Determine sun direction in TEME. Needs only a time.
 * 2: Parallax from shadow and beta. Needs a position as well. 
 *
 * Produces a geocentric result when only a time is available. 
 * This is usable for attitude determination as the difference between 
 * it and a result with parallax calcualted is at most 0.0027 deg in LEO,
 * below the solar model's own error of 0.01 deg.
 *
 * Step 1 is expensive relative to how fast the Sun moves, so it's
 * evaluated at both ends of a segment and interpolated across it. 
 * The segment state lives in SunSegment, owned by the caller.
 */
#ifndef Gnc_Environment_SunModel_HPP
#define Gnc_Environment_SunModel_HPP

#include <cstdint>

#include "PROVESFlightControllerReference/Gnc/AstroLib/AstroLib.hpp"

namespace Gnc {
namespace Environment {

/**
 * \brief How complete a solar evaluation was.
 *
 * \details Mirrors the FPP Gnc::SolarValidity.
 */
enum class SolarResult : std::uint8_t {
    NO_TIME = 0,     //!< No usable time, so nothing could be computed
    GEOCENTRIC = 1,  //!< Direction computed, but no orbit: geocentric, no shadow or beta
    VALID = 2        //!< Fully valid
};

/**
 * Everything one evaluation needs, from OrbitState. Not computed here.
 */
struct SolarQuery {
    double jdUt1 = 0.0;                      //!< UT1 Julian date, flattened
    double jdTt = 0.0;                       //!< TT Julian date, flattened
    Astro::Vec3 posTemeKm{0.0, 0.0, 0.0};    //!< Spacecraft position, TEME, km
    Astro::Vec3 velTemeKmS{0.0, 0.0, 0.0};   //!< Spacecraft velocity, TEME, km/s
    bool timeUsable = false;                 //!< Whether jdUt1 and jdTt are meaningful
    bool positionUsable = false;             //!< Whether posTemeKm and velTemeKmS are meaningful
};

/**
 * \brief Interpolation tuning.
 *
 * \details The component overrides segmentSec from the
 * SUN_SEGMENT_SEC parameter.
 */
struct SolarConfig {
    /**
     * Length of one interpolation segment, seconds. 
     * The Sun sweeps ~0.00066 deg in 60 s,
     * below the 0.01 degree model error.
     * 
     * Set <= 0 to evaluate the model on every call.
     */
    double segmentSec = 60.0;
};

/**
 * \brief Endpoints of the current interpolation segment, in TEME.
 *
 * \details Owned by the caller and passed by reference.
 * Re-anchors when it holds no segment, when the clock
 * steps backwards, or when the passed timestamp is past the segment's end.
 */
struct SunSegment {
    Astro::Vec3 prevTeme{1.0, 0.0, 0.0};   //!< Sun direction at segment start
    Astro::Vec3 nextTeme{1.0, 0.0, 0.0};   //!< Sun direction at segment end
    double rangePrevKm = Astro::AU_KM;     //!< Earth-Sun range at segment start
    double rangeNextKm = Astro::AU_KM;     //!< Earth-Sun range at segment end
    double prevJdUt1 = 0.0;                //!< UT1 JD at segment start
    double nextJdUt1 = 0.0;                //!< UT1 JD at segment end
    bool seeded = false;                   //!< Whether the cache holds a real segment

    //! Discard the segment so the next evaluation re-anchors.
    void resync() { seeded = false; }
};

/**
 * \brief Solar geometry for one evaluation.
 *
 * \details illumination and betaRad are meaningless unless
 * geometryUsable.
 */
struct SolarSolution {
    /**
     * Unit vector toward the Sun, TEME. From the spacecraft when
     * geometryUsable, otherwise from the Earth's center.
     */
    Astro::Vec3 sunUnitTeme{0.0, 0.0, 0.0};

    //! Range to the Sun, kilometers, from the same origin as sunUnitTeme
    double sunRangeKm = 0.0;

    //! Shadow state
    Astro::Illumination illumination = Astro::Illumination::SUNLIT;

    //! Sun elevation above the orbit plane, radians
    double betaRad = 0.0;

    bool directionUsable = false;  //!< sunUnitTeme and sunRangeKm are meaningful
    bool geometryUsable = false;   //!< illumination and betaRad are meaningful
};

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
void sunTemeAt(double tUt1, double tTt, Astro::Vec3& unitTeme, double& rangeKm);

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
                               double& rangeKm);

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
                       SolarSolution& out);

}  // namespace Environment
}  // namespace Gnc

#endif  // Gnc_Environment_SunModel_HPP
