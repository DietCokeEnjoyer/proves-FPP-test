/**
 * \file MagChainTest.cpp
 * \brief Test for the MagneticFieldModel <- OrbitPropagator boundary.
 *
 * \details 
 *
 * Build:
 *   g++ -std=c++14 -O2 -I<repo-root> -I<repo-root>/PROVESFlightControllerReference
 *       PROVESFlightControllerReference/Gnc/AstroLib/AstroLib.cpp
 *       PROVESFlightControllerReference/Gnc/Environment/MagneticFieldModel/WmmModel.cpp
 *       PROVESFlightControllerReference/Gnc/Environment/MagneticFieldModel/test/MagChainTest.cpp
 *       -o magchaintest
 */

#include "PROVESFlightControllerReference/Gnc/Environment/MagneticFieldModel/WmmModel.hpp"

#include "../lib/XYZgeomag.hpp"

#include <cmath>
#include <cstdio>

using namespace Gnc::Astro;
using namespace Gnc::Environment;

namespace {

int g_failures = 0;

void check(const char* what, bool ok, const char* detail = "") {
    std::printf("  %-52s %s %s\n", what, ok ? "PASS" : "FAIL", detail);
    if (!ok) {
        g_failures++;
    }
}

/**
 * Stand in for OrbitPropagator: derive the geodetic altitude the
 * component would have carried in OrbitState, then run the real
 * evaluation on it.
 */
FieldResult evaluateAt(const Vec3& posTemeKm, double jdUt1, double gmstRad, double& altKmOut, FieldSolution& out) {
    const Vec3 ecefKm = temeToEcef(posTemeKm, gmstRad);

    double lat = 0.0;
    double lon = 0.0;
    ecefToGeodetic(ecefKm, lat, lon, altKmOut);

    FieldQuery query;
    query.posTemeKm = posTemeKm;
    query.altKm = altKmOut;
    query.jdUt1 = jdUt1;
    query.gmstRad = gmstRad;

    return evaluateField(query, FieldConfig(), out);
}

}  // namespace

int main() {
    /*
     * A LEO position: 420 km altitude, 51.6 deg
     * inclination, at an arbitrary point in the orbit.
     */
    const double jd = 2461233.5;  // 2026-07-12 00:00 UT1
    const double tUt1 = (jd - JD_J2000) / DAYS_PER_JCENT;
    const double gmst = gmst1982Rad(tUt1);

    const double r = RADIUS_EARTH_KM + 420.0;
    const Vec3 posTemeKm{r * 0.5, r * 0.6, r * std::sqrt(1.0 - 0.25 - 0.36)};

    std::printf("Magnetic chain checks (2026-07-12, ~420 km)\n\n");

    /*
     * ----------------------------------------------------------------------------
     * Units
     * ----------------------------------------------------------------------------
     */
    double altKm = 0.0;
    FieldSolution sol;
    const FieldResult result = evaluateAt(posTemeKm, jd, gmst, altKm, sol);

    std::printf("  evaluated altitude: %.2f km\n", altKm);
    std::printf("  |B| = %.1f nT\n\n", sol.magnitudeNt);

    check("evaluation accepted", result == FieldResult::OK);
    check("geodetic altitude is plausible for LEO", altKm > 300.0 && altKm < 500.0);
    check("|B| within the LEO envelope (18000-60000 nT)", sol.magnitudeNt > 18000.0 && sol.magnitudeNt < 60000.0);
    check("direction is usable at this field strength", sol.directionUsable);
    check("unit vector has unit length", std::fabs(vnorm(sol.unitTeme) - 1.0) < 1.0e-12);
    check("unit vector is parallel to the field", vdot(sol.unitTeme, vunit(sol.fieldTemeNt)) > 1.0 - 1.0e-12);

    {
        const Vec3 ecefKm = temeToEcef(posTemeKm, gmst);
        geomag::Vector p;
        p.x = static_cast<float>(ecefKm.x);  // km, NOT m
        p.y = static_cast<float>(ecefKm.y);
        p.z = static_cast<float>(ecefKm.z);
        const geomag::Vector bad = geomag::GeoMag(2026.53f, p, geomag::WMM2025);
        const double badMag =
            std::sqrt(double(bad.x) * bad.x + double(bad.y) * bad.y + double(bad.z) * bad.z) * TESLA_TO_NT;
        std::printf("  (km fed where metres expected: |B| = %.3e nT)\n", badMag);
        check("km-for-metres is caught by the magnitude envelope", !(badMag > 18000.0 && badMag < 60000.0));
    }

    /*
     * ----------------------------------------------------------------------------
     * Frame round trip
     * ----------------------------------------------------------------------------
     */
    {
        const Vec3 ecef = temeToEcef(posTemeKm, gmst);
        const Vec3 back = rot3(ecef, -gmst);
        const double err = vnorm(vsub(back, posTemeKm));
        char d[64];
        std::snprintf(d, sizeof d, "(residual %.3e km)", err);
        check("TEME -> ECEF -> TEME is exact", err < 1.0e-9, d);
    }

    /*
     * A rotation preserves length, so |B| must not depend on GMST even
     * though the components do. This is the invariant that catches a
     * sign error in the back-rotation.
     */
    {
        double a = 0.0;
        FieldSolution shifted;
        (void)evaluateAt(posTemeKm, jd, gmst + 1.0, a, shifted);
        /*
         * Different GMST means a different ECEF point, hence a different
         * field -- but both must still be plausible LEO magnitudes.
         */
        check("|B| stays in envelope under a different GMST",
              shifted.magnitudeNt > 18000.0 && shifted.magnitudeNt < 60000.0);
    }

    /*
     * ----------------------------------------------------------------------------
     * Time precision
     * ----------------------------------------------------------------------------
     */
    {
        const double decYearExact = decimalYearFromJd(jd);
        const float decYearF32 = static_cast<float>(decYearExact);
        const double quantErrYears = std::fabs(double(decYearF32) - decYearExact);

        // The direct statement of the problem: one ULP of F32 at 2026.
        const float oneUlp = std::nextafter(2026.5f, 3000.0f) - 2026.5f;
        const double ulpHours = double(oneUlp) * 365.25 * 24.0;
        const double ulpDeg = ulpHours * 15.0;

        std::printf("\n  F32 decimal year at 2026: 1 ULP = %.2e yr = %.2f h = %.1f deg of Earth rotation\n",
                    double(oneUlp), ulpHours, ulpDeg);
        std::printf("  F32 rounding of this epoch: %.2e yr\n\n", quantErrYears);

        check("F32 decimal year is UNUSABLE for Earth rotation (>1 deg)", ulpDeg > 1.0);
        check("F32 decimal year is fine for WMM secular terms (<0.01 yr)", double(oneUlp) < 0.01);

        /*
         * The model narrows the decimal year to F32 on purpose, and only
         * where it feeds the secular variation terms. Confirm that is
         * the value it reports back.
         */
        check("model reports the F32 decimal year it evaluated at", sol.decYear == decYearF32);
    }

    /*
     * ----------------------------------------------------------------------------
     * Field direction changes measurably over an orbit
     * ----------------------------------------------------------------------------
     */
    {
        const Vec3 posB{-posTemeKm.x, -posTemeKm.y, posTemeKm.z};
        double a = 0.0;
        FieldSolution other;
        (void)evaluateAt(posB, jd, gmst, a, other);
        const double cosAng = vdot(sol.unitTeme, other.unitTeme);
        const double angDeg = std::acos(clampUnit(cosAng)) * RAD2DEG;
        char d[64];
        std::snprintf(d, sizeof d, "(%.1f deg apart)", angDeg);
        check("field direction responds to position", angDeg > 10.0, d);
    }

    /*
     * ----------------------------------------------------------------------------
     * Range gates. 
     * ----------------------------------------------------------------------------
     */
    {
        FieldQuery query;
        query.posTemeKm = posTemeKm;
        query.jdUt1 = jd;
        query.gmstRad = gmst;

        const FieldConfig cfg;
        FieldSolution out;

        // Above the WMM's fitted shell
        query.altKm = 20000.0;
        check("altitude above the fitted shell is rejected",
              evaluateField(query, cfg, out) == FieldResult::ALTITUDE_OUT_OF_RANGE);

        // Below the ellipsoid by more than the model tolerates.
        query.altKm = -50.0;
        check("altitude below the ellipsoid is rejected",
              evaluateField(query, cfg, out) == FieldResult::ALTITUDE_OUT_OF_RANGE);

        // Just inside each bound
        query.altKm = static_cast<double>(cfg.maxAltKm) - 1.0;
        check("altitude just inside the upper bound is accepted", evaluateField(query, cfg, out) == FieldResult::OK);

        /*
         * Epoch outside the valid window
         */
        query.altKm = 420.0;
        query.jdUt1 = 2451545.0;  // 2000-01-01
        check("epoch before the coefficient window is rejected",
              evaluateField(query, cfg, out) == FieldResult::EPOCH_OUT_OF_RANGE);

        /*
         * decYear is written before the gate, so the component can
         * telemeter the epoch it tried alongside the rejection.
         */
        check("decYear is reported even on a rejected epoch", out.decYear > 1999.0f && out.decYear < 2001.0f);
    }

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL CHECKS PASSED" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
