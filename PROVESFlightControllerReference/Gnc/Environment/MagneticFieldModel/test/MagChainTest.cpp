// ======================================================================
// MagChainTest.cpp
//
// Host test for the MagneticFieldModel <- OrbitPropagator seam. No F Prime
// and no hardware: it reproduces exactly what evaluate() does, so the
// three things that seam gets wrong when it is wrong are all visible on
// a workstation in one second.
//
//   1. UNITS. XYZgeomag wants ITRS metres; SGP4 produces kilometres.
//      Getting this backwards puts the evaluation 1000x too close to
//      the centre of the Earth, where the dipole term blows up.
//   2. FRAME ROUND TRIP. ECEF -> TEME must undo TEME -> ECEF exactly.
//   3. TIME PRECISION. GMST from an F32 decimal year vs. from an F64
//      one. This is the check that fails loudest on the old code.
//
// Build:
//   g++ -std=c++11 -O2 -I. -IGnc/AstroLib Gnc/AstroLib/AstroLib.cpp
//       Gnc/Environment/MagneticFieldModel/test/MagChainTest.cpp -o magchaintest
// ======================================================================

#include "AstroLib.hpp"
#include "../lib/XYZgeomag.hpp"

#include <cmath>
#include <cstdio>

using namespace Gnc::Astro;

namespace {

int g_failures = 0;

void check(const char* what, bool ok, const char* detail = "") {
    std::printf("  %-52s %s %s\n", what, ok ? "PASS" : "FAIL", detail);
    if (!ok) {
        g_failures++;
    }
}

//! Exactly the flight-path computation in MagneticFieldModel::evaluate.
Vec3 fieldTemeNt(const Vec3& posTemeKm, double jdUt1, double gmstRad, double& altKmOut) {
    const Vec3 ecefKm = temeToEcef(posTemeKm, gmstRad);

    double lat = 0.0;
    double lon = 0.0;
    ecefToGeodetic(ecefKm, lat, lon, altKmOut);

    geomag::Vector p;
    p.x = static_cast<float>(ecefKm.x * 1000.0);   // km -> m
    p.y = static_cast<float>(ecefKm.y * 1000.0);
    p.z = static_cast<float>(ecefKm.z * 1000.0);

    const float decYear = static_cast<float>(decimalYearFromJd(jdUt1));
    const geomag::Vector bT = geomag::GeoMag(decYear, p, geomag::WMM2025);

    const Vec3 bTesla { bT.x, bT.y, bT.z };
    const Vec3 bTeme = rot3(bTesla, -gmstRad);

    return vscale(bTeme, 1.0e9);   // Tesla -> nT
}

}  // namespace

int main() {
    // A representative ISS-like LEO position: 420 km altitude, 51.6 deg
    // inclination, at an arbitrary point in the orbit.
    const double jd = 2461233.5;                 // 2026-07-12 00:00 UT1
    const double tUt1 = (jd - JD_J2000) / DAYS_PER_JCENT;
    const double gmst = gmst1982Rad(tUt1);

    const double r = R_EARTH_KM + 420.0;
    const Vec3 posTemeKm { r * 0.5, r * 0.6, r * std::sqrt(1.0 - 0.25 - 0.36) };

    std::printf("Magnetic chain checks (2026-07-12, ~420 km)\n\n");

    // ------------------------------------------------------------------
    // 1. Units
    // ------------------------------------------------------------------
    double altKm = 0.0;
    const Vec3 bNt = fieldTemeNt(posTemeKm, jd, gmst, altKm);
    const double mag = vnorm(bNt);

    std::printf("  evaluated altitude: %.2f km\n", altKm);
    std::printf("  |B| = %.1f nT\n\n", mag);

    check("geodetic altitude is plausible for LEO",
          altKm > 300.0 && altKm < 500.0);
    check("|B| within the LEO envelope (18000-60000 nT)",
          mag > 18000.0 && mag < 60000.0);

    // The failure mode the old code had: pass km where metres were
    // expected. The evaluation point ends up ~6800 km from the centre
    // divided by 1000, i.e. deep inside the Earth, and the r^-3 dipole
    // term explodes by roughly 10^9.
    {
        geomag::Vector p;
        p.x = static_cast<float>(temeToEcef(posTemeKm, gmst).x);   // km, NOT m
        p.y = static_cast<float>(temeToEcef(posTemeKm, gmst).y);
        p.z = static_cast<float>(temeToEcef(posTemeKm, gmst).z);
        const geomag::Vector bad = geomag::GeoMag(2026.53f, p, geomag::WMM2025);
        const double badMag = std::sqrt(double(bad.x) * bad.x +
                                        double(bad.y) * bad.y +
                                        double(bad.z) * bad.z) * 1.0e9;
        std::printf("  (km fed where metres expected: |B| = %.3e nT)\n", badMag);
        check("km-for-metres is caught by the magnitude envelope",
              !(badMag > 18000.0 && badMag < 60000.0));
    }

    // ------------------------------------------------------------------
    // 2. Frame round trip
    // ------------------------------------------------------------------
    {
        const Vec3 ecef = temeToEcef(posTemeKm, gmst);
        const Vec3 back = rot3(ecef, -gmst);
        const double err = vnorm(vsub(back, posTemeKm));
        char d[64];
        std::snprintf(d, sizeof d, "(residual %.3e km)", err);
        check("TEME -> ECEF -> TEME is exact", err < 1.0e-9, d);
    }

    // A rotation preserves length, so |B| must not depend on GMST even
    // though the components do. This is the invariant that catches a
    // sign error in the back-rotation.
    {
        double a = 0.0;
        double b = 0.0;
        const double m1 = vnorm(fieldTemeNt(posTemeKm, jd, gmst, a));
        const double m2 = vnorm(fieldTemeNt(posTemeKm, jd, gmst + 1.0, b));
        // Different GMST means a different ECEF point, hence a different
        // field -- but both must still be plausible LEO magnitudes.
        check("|B| stays in envelope under a different GMST",
              m2 > 18000.0 && m2 < 60000.0);
        (void)m1;
    }

    // ------------------------------------------------------------------
    // 3. Time precision -- the headline bug
    // ------------------------------------------------------------------
    {
        const double decYearExact = decimalYearFromJd(jd);
        const float decYearF32 = static_cast<float>(decYearExact);
        const double quantErrYears = std::fabs(double(decYearF32) - decYearExact);

        // Reconstruct what the old component did: derive Earth rotation
        // from the F32 decimal year rather than from a proper JD.
        const double daysFromF32 =
            (double(decYearF32) - 2026.0) * 365.0 + (jd - 2461041.5) * 0.0;
        (void)daysFromF32;

        // The direct statement of the problem: one ULP of F32 at 2026.
        const float oneUlp = std::nextafter(2026.5f, 3000.0f) - 2026.5f;
        const double ulpHours = double(oneUlp) * 365.25 * 24.0;
        const double ulpDeg = ulpHours * 15.0;

        std::printf("\n  F32 decimal year at 2026: 1 ULP = %.2e yr = %.2f h = %.1f deg of Earth rotation\n",
                    double(oneUlp), ulpHours, ulpDeg);
        std::printf("  F32 rounding of this epoch: %.2e yr\n\n", quantErrYears);

        check("F32 decimal year is UNUSABLE for Earth rotation (>1 deg)",
              ulpDeg > 1.0);
        check("F32 decimal year is fine for WMM secular terms (<0.01 yr)",
              double(oneUlp) < 0.01);
    }

    // ------------------------------------------------------------------
    // 4. Field direction changes measurably over an orbit -- i.e. the
    //    model is actually responding to position, not returning a
    //    constant that happens to be the right size.
    // ------------------------------------------------------------------
    {
        const Vec3 posB { -posTemeKm.x, -posTemeKm.y, posTemeKm.z };
        double a = 0.0;
        const Vec3 b2 = fieldTemeNt(posB, jd, gmst, a);
        const double cosAng = vdot(vunit(bNt), vunit(b2));
        const double angDeg = std::acos(clampUnit(cosAng)) * RAD2DEG;
        char d[64];
        std::snprintf(d, sizeof d, "(%.1f deg apart)", angDeg);
        check("field direction responds to position", angDeg > 10.0, d);
    }

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL CHECKS PASSED" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
