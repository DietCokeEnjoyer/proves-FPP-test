// ======================================================================
// \file   AstroLibTest.cpp
// \brief  Host-side checks for the astro kernel. No F Prime, no gtest --
//         plain asserts so it builds anywhere. Port these into
//         register_fprime_ut() / gtest for CI.
//
//   g++ -std=c++11 -O2 -I<repo-root> AstroLib.cpp test/AstroLibTest.cpp -o t
// ======================================================================
#include "Gnc/AstroLib/AstroLib.hpp"

#include <cstdio>
#include <cstdlib>

using namespace Gnc::Astro;

static int g_failures = 0;

static void check(const char* name, double got, double want, double tol) {
    const double err = std::fabs(got - want);
    const bool ok = (err <= tol);
    if (!ok) {
        g_failures++;
    }
    std::printf("%-46s got=%14.7f want=%14.7f err=%10.3e  %s\n",
                name, got, want, err, ok ? "PASS" : "FAIL");
}

//! Seconds since Unix epoch for a UTC calendar date (proleptic Gregorian).
static double unixFromUtc(int y, int mo, int d, int h, int mi, double s) {
    // Days from civil algorithm (Howard Hinnant).
    y -= (mo <= 2);
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const long long days = static_cast<long long>(era) * 146097 + static_cast<long long>(doe) - 719468;
    return static_cast<double>(days) * 86400.0 + h * 3600.0 + mi * 60.0 + s;
}

static double declinationDeg(const Vec3& u) {
    return std::asin(clampUnit(u.z)) * RAD2DEG;
}

int main() {
    // -----------------------------------------------------------------
    // 1. Julian date. J2000.0 is 2000-01-01T12:00:00 UTC -> JD 2451545.0
    // -----------------------------------------------------------------
    {
        const double u = unixFromUtc(2000, 1, 1, 12, 0, 0.0);
        check("unix seconds at J2000", u, 946728000.0, 0.5);
        const JulianDate2 jd = jdFromUnixUtc(u);
        check("JD at J2000", jdFlatten(jd), 2451545.0, 1e-9);
    }

    // -----------------------------------------------------------------
    // 2. GMST at J2000.0 is 280.46061837 deg (18h 41m 50.55s). This is the
    //    canonical published value, so it pins the sidereal time model.
    // -----------------------------------------------------------------
    {
        const TimeScales ts = computeTimeScales(unixFromUtc(2000, 1, 1, 12, 0, 0.0), 0.0, 32.0);
        check("GMST at J2000 (deg)", gmst1982Rad(ts.tUt1) * RAD2DEG, 280.46061837, 1e-4);
    }

    // -----------------------------------------------------------------
    // 3. Obliquity at J2000.0 is 23.4392911 deg.
    // -----------------------------------------------------------------
    {
        const TimeScales ts = computeTimeScales(unixFromUtc(2000, 1, 1, 12, 0, 0.0), 0.0, 32.0);
        const Nutation n = nutation1980(ts.tTt);
        check("mean obliquity at J2000 (deg)", n.meanEps * RAD2DEG, 23.4392911, 1e-5);
        check("equation of equinoxes magnitude (arcsec)",
              std::fabs(n.eqEq) / ARCSEC2RAD, 8.0, 8.0);  // sanity band: 0-16"
    }

    // -----------------------------------------------------------------
    // 4. Solar declination at the 2026 solstices and equinoxes.
    //    These are physics, not model output, so they are a real check.
    // -----------------------------------------------------------------
    struct Case { const char* name; int y, mo, d, h, mi; double wantDecDeg, tol; };
    const Case cases[] = {
        { "sun dec, Mar equinox 2026-03-20 14:46",  2026,  3, 20, 14, 46,   0.000, 0.02 },
        { "sun dec, Jun solstice 2026-06-21 08:24", 2026,  6, 21,  8, 24,  23.436, 0.02 },
        { "sun dec, Sep equinox 2026-09-23 00:05",  2026,  9, 23,  0,  5,   0.000, 0.02 },
        { "sun dec, Dec solstice 2026-12-21 20:50", 2026, 12, 21, 20, 50, -23.436, 0.02 },
    };
    for (const Case& c : cases) {
        const TimeScales ts = computeTimeScales(unixFromUtc(c.y, c.mo, c.d, c.h, c.mi, 0.0), 0.0, 37.0);
        const SunState s = sunLowPrecisionMod(ts.tUt1, ts.tTt);
        check(c.name, declinationDeg(s.unitMod), c.wantDecDeg, c.tol);
    }

    // -----------------------------------------------------------------
    // 5. Earth-Sun range at perihelion / aphelion 2026.
    // -----------------------------------------------------------------
    {
        TimeScales ts = computeTimeScales(unixFromUtc(2026, 1, 3, 17, 16, 0.0), 0.0, 37.0);
        check("range at perihelion (AU)",
              sunLowPrecisionMod(ts.tUt1, ts.tTt).rangeKm / AU_KM, 0.98330, 0.0005);
        ts = computeTimeScales(unixFromUtc(2026, 7, 6, 17, 31, 0.0), 0.0, 37.0);
        check("range at aphelion (AU)",
              sunLowPrecisionMod(ts.tUt1, ts.tTt).rangeKm / AU_KM, 1.01668, 0.0005);
    }

    // -----------------------------------------------------------------
    // 6. Frame chain must be small and invertible-ish: MOD -> TEME should
    //    move a vector by less than 30 arcsec, never by degrees. This is
    //    the check that catches a sign error in the nutation rotation.
    // -----------------------------------------------------------------
    {
        const TimeScales ts = computeTimeScales(unixFromUtc(2026, 7, 30, 0, 0, 0.0), 0.0, 37.0);
        const Nutation n = nutation1980(ts.tTt);
        const SunState s = sunLowPrecisionMod(ts.tUt1, ts.tTt);
        const Vec3 teme = modToTeme(s.unitMod, n);
        const double sepArcsec = std::acos(clampUnit(vdot(s.unitMod, teme))) * RAD2DEG * 3600.0;
        check("MOD->TEME separation (arcsec)", sepArcsec, 10.0, 20.0);
        check("MOD->TEME preserves norm", vnorm(teme), 1.0, 1e-12);
    }

    // -----------------------------------------------------------------
    // 7. Geodetic round trip for a known point: 1000 km above the equator
    //    at 0 deg longitude.
    // -----------------------------------------------------------------
    {
        double lat, lon, alt;
        ecefToGeodetic(Vec3 { R_EARTH_KM + 1000.0, 0.0, 0.0 }, lat, lon, alt);
        check("geodetic lat (equator)", lat * RAD2DEG, 0.0, 1e-9);
        check("geodetic alt (equator)", alt, 1000.0, 1e-6);

        // North pole: altitude measured from the polar radius.
        const double b = R_EARTH_KM * (1.0 - F_EARTH);
        ecefToGeodetic(Vec3 { 0.0, 0.0, b + 500.0 }, lat, lon, alt);
        check("geodetic lat (pole)", std::fabs(lat) * RAD2DEG, 90.0, 1e-6);
        check("geodetic alt (pole)", alt, 500.0, 1e-3);
    }

    // -----------------------------------------------------------------
    // 8. Shadow model. Place the Sun on +x, then probe the anti-sun axis.
    // -----------------------------------------------------------------
    {
        const Vec3 sun { AU_KM, 0.0, 0.0 };

        // Directly behind Earth at 500 km altitude -> deep umbra.
        check("umbra flag", static_cast<double>(shadowConical(Vec3 { -(R_EARTH_KM + 500.0), 0.0, 0.0 }, sun)),
              static_cast<double>(Illumination::UMBRA), 0.0);

        // Same anti-sun distance but well off-axis -> full sun.
        check("sunlit flag (off-axis)",
              static_cast<double>(shadowConical(Vec3 { -6878.0, 20000.0, 0.0 }, sun)),
              static_cast<double>(Illumination::SUNLIT), 0.0);

        // Sunward side -> always lit.
        check("sunlit flag (sunward)",
              static_cast<double>(shadowConical(Vec3 { 6878.0, 0.0, 0.0 }, sun)),
              static_cast<double>(Illumination::SUNLIT), 0.0);

        // Grazing the terminator at 6878 km must be penumbra somewhere.
        int sawPen = 0;
        for (int i = 0; i < 4000; ++i) {
            const double yOff = 6300.0 + 0.1 * i;
            if (shadowConical(Vec3 { -6878.0, yOff, 0.0 }, sun) == Illumination::PENUMBRA) {
                sawPen = 1;
                break;
            }
        }
        check("penumbra band exists", static_cast<double>(sawPen), 1.0, 0.0);
    }

    // -----------------------------------------------------------------
    // 9. Beta angle. Equatorial prograde orbit, Sun on +z -> beta = +90.
    // -----------------------------------------------------------------
    {
        const Vec3 r { 7000.0, 0.0, 0.0 };
        const Vec3 v { 0.0, 7.5, 0.0 };
        check("beta, sun on orbit normal", betaAngleRad(r, v, Vec3 { 0, 0, 1 }) * RAD2DEG, 90.0, 1e-12);
        check("beta, sun in orbit plane",  betaAngleRad(r, v, Vec3 { 1, 0, 0 }) * RAD2DEG,  0.0, 1e-9);
    }

    // -----------------------------------------------------------------
    // 10. SLERP endpoints and midpoint.
    // -----------------------------------------------------------------
    {
        const Vec3 a { 1, 0, 0 };
        const Vec3 b { 0, 1, 0 };
        check("slerp t=0",   vslerp(a, b, 0.0).x, 1.0, 1e-12);
        check("slerp t=1",   vslerp(a, b, 1.0).y, 1.0, 1e-12);
        check("slerp t=0.5", vslerp(a, b, 0.5).x, std::sqrt(0.5), 1e-12);
        check("slerp unit",  vnorm(vslerp(a, b, 0.37)), 1.0, 1e-12);
    }

    std::printf("\n%s (%d failures)\n", g_failures ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", g_failures);
    return g_failures ? 1 : 0;
}
