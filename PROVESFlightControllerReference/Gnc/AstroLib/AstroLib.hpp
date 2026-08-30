/**
 * \file AstroLib.hpp
 * \brief FPP independent astrodynamics kernel.
 * 
 * \details FPP indepndent / pure CPP for ease of testing and debugging.
 * 
 *  Conventions: 
 *      - Distances in km, angles in radians unless suffixed _deg / Au.
 *      - Frames are "of date": MOD / TOD / TEME / ECEF.
 *      - No dynamic allocation, no exceptions, no recursion, no virtuals.
 */
#ifndef GNC_ASTROLIB_HPP
#define GNC_ASTROLIB_HPP

#include <cmath>
#include <cstdint>

namespace Gnc {
namespace Astro {

/*
 * ============================================================================
 * Constants TODO: Cite sources for constant values, add comments/explanations of constants
 * ============================================================================
 */

constexpr double PI         = 3.14159265358979323846;
constexpr double TWO_PI     = 2.0 * PI;
constexpr double DEG2RAD    = PI / 180.0;
constexpr double RAD2DEG    = 180.0 / PI;
constexpr double ARCSEC2RAD = DEG2RAD / 3600.0;

constexpr double JD_J2000      = 2451545.0;   //!< JD of 2000-01-01T12:00:00 TT
constexpr double JD_UNIX_EPOCH = 2440587.5;   //!< JD of 1970-01-01T00:00:00
constexpr double SEC_PER_DAY   = 86400.0;
constexpr double MIN_PER_DAY   = 1440.0;
constexpr double DAYS_PER_JCENT = 36525.0;

constexpr double AU_KM      = 149597870.7;        //!< IAU 2012 astronomical unit
constexpr double R_EARTH_KM = 6378.137;           //!< WGS-84 equatorial radius
constexpr double F_EARTH    = 1.0 / 298.257223563;//!< WGS-84 flattening
constexpr double R_SUN_KM   = 696000.0;           //!< Solar radius (Vallado)

//! TT - TAI, fixed by definition.
constexpr double TT_MINUS_TAI_SEC = 32.184;

/**
 * Earth shadow cone half-angles (Vallado, Fundamentals of Astrodynamics,
 * Alg. 34). Constant to good approximation over Earth's orbit.
 */
constexpr double ALPHA_UMB_RAD = 0.264121 * DEG2RAD;
constexpr double ALPHA_PEN_RAD = 0.269007 * DEG2RAD;

/**
 * Unitless F64 3-D vector.
 */
struct Vec3 {
    double x;
    double y;
    double z;
};

/**
 * Vector addition
 */
inline Vec3 vadd(const Vec3& a, const Vec3& b) { return Vec3 { a.x + b.x, a.y + b.y, a.z + b.z }; }

/**
 * Vector subtraction
 */
inline Vec3 vsub(const Vec3& a, const Vec3& b) { return Vec3 { a.x - b.x, a.y - b.y, a.z - b.z }; }

/**
 * Vector scalar multiplication
 */
inline Vec3 vscale(const Vec3& a, double s)    { return Vec3 { a.x * s, a.y * s, a.z * s }; }

/**
 * Vector dot product
 */
inline double vdot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/**
 * Vector cross product
 */
inline Vec3 vcross(const Vec3& a, const Vec3& b) {
    return Vec3 { a.y * b.z - a.z * b.y,
                  a.z * b.x - a.x * b.z,
                  a.x * b.y - a.y * b.x };
}
/**
 * Euclidian Norm
 */
inline double vnorm(const Vec3& a) { return std::sqrt(vdot(a, a)); }

/**
 * Returns a unit vector with the same direction as a. Returns the input 
 * unchanged if it's near zero length so NaNs don't propagate into the attitude chain.
 */

Vec3 vunit(const Vec3& a);

/**
 * Clamp a double to [-1, 1]. 
 * Used before acos/asin to guard against domain errors from rounding.
 */
inline double clampUnit(double v) {
    if (v > 1.0) {
        return 1.0;
    } 
    
    if (v < -1.0) {
        return -1.0;
    } 
    
    return v;
}

/**
 * Spherical linear interpolation between two unit vectors, t in [0,1].
 * Falls back to normalized lerp for small separations.
 */
Vec3 vslerp(const Vec3& a, const Vec3& b, double t);

/*
 * ============================================================================
 * Time systems
 * ============================================================================
 */

 /**
  * A Julian date carried as two doubles (whole day + fraction) to allow 
  * for ~microsecond resolution.
  */
struct JulianDate2 {
    double day;   //!< Large part, nominally a whole/half day number
    double frac;  //!< Small part, nominally in [0, 1)
};

/**
 * All the time scales one cycle of the algorithm needs.
 */
struct TimeScales {
    JulianDate2 jdUtc;
    JulianDate2 jdUt1;
    double      tUt1;  //!< Julian centuries of UT1 since J2000
    double      tTt;   //!< Julian centuries of TT  since J2000 (~= TDB)
};

/**
 * Convert POSIX seconds (UTC, no leap seconds encoded) to a split JD.
 */
JulianDate2 jdFromUnixUtc(double unixSecondsUtc);

/**
 * Collapse a split JD. Only use where a few tens of microseconds of
 * resolution loss is acceptable (e.g. telemetry).
 */
inline double jdFlatten(const JulianDate2& jd) { return jd.day + jd.frac; }

/**
 * Difference in days, computed part-wise to avoid catastrophic cancellation.
 */
inline double jdDiffDays(const JulianDate2& a, const JulianDate2& b) {
    return (a.day - b.day) + (a.frac - b.frac);
}

/**
 * Build every time scale from a UTC POSIX timestamp.
 * \param unixSecondsUtc  Seconds since 1970-01-01T00:00:00Z
 * \param dut1Sec         UT1 - UTC, from IERS Bulletin A. |dut1| < 0.9 s.
 * \param taiMinusUtcSec  Current leap second count (37.0 as of 2017).
 */
TimeScales computeTimeScales(double unixSecondsUtc, double dut1Sec, double taiMinusUtcSec);

/**
 * Greenwich Mean Sidereal Time, IAU-1982 model. Returns radians in [0, 2pi).
 */
double gmst1982Rad(double tUt1);

/**
 * 
 * Decimal year (e.g. 2026.5772) from a UT1 Julian date, Gregorian
 * calendar, leap years handled exactly.
 * 
 * Exists because the WMM's secular-variation terms are a series in
 * decimal year. Computed and returned in DOUBLE: the caller may narrow
 * to float for the WMM call itself (an F32 ULP here is 1.22e-4 years,
 * which costs about 0.005 nT of secular drift -- irrelevant), but the
 * same value must never be used to derive Earth rotation, where 2e-4
 * years is 1.07 hours and 16.1 degrees. Use gmst1982Rad for that.
 * 
 */
double decimalYearFromJd(double jdUt1);

/*
 * ============================================================================
 * Precession / nutation
 * ============================================================================
 */

/**
 * Truncated IAU-1980 nutation. Four largest terms only; good to ~1 arcsec,
 * which is two orders of magnitude finer than the Sun model error, so the
 * remaining ~100 terms buy nothing here.
 */
struct Nutation {
    double dPsi;     //!< Nutation in longitude, rad
    double dEps;     //!< Nutation in obliquity, rad
    double meanEps;  //!< Mean obliquity of the ecliptic, rad
    double trueEps;  //!< True obliquity, rad
    double eqEq;     //!< Equation of the equinoxes, rad
};

Nutation nutation1980(double tTt);

/**
 * Frame rotations. rot1/rot3 rotate the *coordinate frame* about the
 * 1- and 3-axes by the given angle (Vallado's ROT1 / ROT3).
 */
Vec3 rot1(const Vec3& v, double angleRad);
Vec3 rot3(const Vec3& v, double angleRad);

/**
 * Mean-of-date -> True-of-date (apply nutation).
 */
Vec3 modToTod(const Vec3& v, const Nutation& n);

/**
 * True-of-date -> TEME (rotate by the equation of the equinoxes).
 */
Vec3 todToTeme(const Vec3& v, const Nutation& n);

/**
 * Convenience: MOD -> TEME in one shot.
 */
Vec3 modToTeme(const Vec3& v, const Nutation& n);

/**
 * TEME -> ECEF (PEF). Ignores polar motion, which is < 0.5 arcsec.
 */
Vec3 temeToEcef(const Vec3& v, double gmstRad);

/**
 * WGS-84 geodetic conversion, Bowring's method (non-iterative).
 */
void ecefToGeodetic(const Vec3& rEcefKm, double& latRad, double& lonRad, double& altKm);

/*
 * ============================================================================
 * Solar ephemeris
 * ============================================================================
 */

/**
 * 
 */
struct SunState {
    Vec3   unitMod;   //!< Unit vector Earth -> Sun, MOD frame
    double rangeKm;   //!< Earth-Sun distance
    double eclLonRad; //!< Apparent ecliptic longitude (useful for checkout)
};

/**
 * Low-precision solar ephemeris (Astronomical Almanac / Vallado Alg. 29).
 * Accuracy is roughly 0.01 deg in direction for 1950-2050.
 */
SunState sunLowPrecisionMod(double tUt1, double tTdb);

/**
 * ============================================================================
 * Illumination and geometry
 * ============================================================================
 */

/**
 * 
 */
enum class Illumination : uint8_t {
    SUNLIT   = 0,
    PENUMBRA = 1,
    UMBRA    = 2
};

/**
 * Dual-cone Earth shadow model (Vallado Alg. 34). Both vectors must be in
 * the same frame, both geocentric, both km.
 */
Illumination shadowConical(const Vec3& rSatKm, const Vec3& rSunKm);

/**
 * Cheaper cylindrical shadow. Sunlit == false means eclipsed.
 */
bool isSunlitCylindrical(const Vec3& rSatKm, const Vec3& sunUnit);

/**
 * Unit vector from the *spacecraft* to the Sun, including parallax.
 */
Vec3 sunUnitFromSpacecraft(const Vec3& rSatKm, const Vec3& rSunKm, double& rangeKm);

/**
 * Beta angle: elevation of the Sun above the orbit plane. Positive when
 * the Sun lies on the +h (angular momentum) side.
 */
double betaAngleRad(const Vec3& rKm, const Vec3& vKmS, const Vec3& sunUnit);

/**
 * Nadir unit vector (spacecraft -> Earth centre).
 */
inline Vec3 nadirUnit(const Vec3& rKm) { return vunit(vscale(rKm, -1.0)); }

} // namespace Astro
} // namespace Gnc

#endif // GNC_ASTROLIB_HPP
