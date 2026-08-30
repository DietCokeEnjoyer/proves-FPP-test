/**
 * \file TriadSolverTest.cpp
 * \brief Host-side test for the TRIAD solver. No F Prime, no hardware.
 *
 * \details   c++ -std=c++14 -I.. -I<eigen> TriadSolverTest.cpp ../TriadSolver.cpp
 *
 * Build this WITHOUT EIGEN_NO_DEBUG so Eigen's internal asserts stay
 * live -- the flight build disables them, but you want them here.
 *
 * The interesting test is not "does it return OK". It is the noise
 * sweep at the bottom: it demonstrates the 1/sin(separation) error
 * amplification that drives the MIN_SEPARATION_DEG parameter, and it
 * shows the accuracy asymmetry between the primary and secondary legs.
 */

#include "TriadSolver.hpp"

#include <cmath>
#include <cstdio>
#include <random>

using namespace Gnc::Adcs;

namespace {


int g_failures = 0;

void check(bool condition, const char* what) {
    if (!condition) {
        std::printf("  FAIL: %s\n", what);
        g_failures++;
    }
}

/**
 * Build a synthetic case: rotate reference vectors by a known truth
 * attitude, optionally corrupting the body measurements with noise.
 */
TriadObservation makeCase(const Eigen::Quaternionf& truth,
                          const Eigen::Vector3f& primaryRef,
                          const Eigen::Vector3f& secondaryRef,
                          const Eigen::Vector3f& primaryNoise = Eigen::Vector3f::Zero(),
                          const Eigen::Vector3f& secondaryNoise = Eigen::Vector3f::Zero()) {
    TriadObservation obs;
    obs.primaryRef = primaryRef.normalized();
    obs.secondaryRef = secondaryRef.normalized();
    obs.primaryBody = (truth * obs.primaryRef + primaryNoise).normalized();
    obs.secondaryBody = (truth * obs.secondaryRef + secondaryNoise).normalized();
    return obs;
}

Eigen::Quaternionf axisAngle(float angleRad, const Eigen::Vector3f& axis) {
    return Eigen::Quaternionf(Eigen::AngleAxisf(angleRad, axis.normalized()));
}

// ----------------------------------------------------------------------

void testExactRecovery() {
    std::printf("exact recovery (noise-free)\n");
    const Eigen::Quaternionf truth = axisAngle(0.9f, Eigen::Vector3f(0.2f, 0.7f, -0.4f));
    const TriadObservation obs =
        makeCase(truth, Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0.7f, 0, 0.7f));

    TriadSolution sol;
    check(triadSolve(obs, TriadConfig(), sol) == TriadResult::OK, "solve returned OK");

    const float errDeg = sol.quatBodyFromRef.angularDistance(truth) * RAD2DEG;
    std::printf("  attitude error: %.6f deg\n", errDeg);
    check(errDeg < 1e-3f, "noise-free error below 1e-3 deg");
}

void testPrimaryIsExact() {
    /*
     * The defining property of TRIAD: the primary observation is
     * reproduced exactly even when the secondary is badly corrupted.
     */
    std::printf("primary leg satisfied exactly despite secondary error\n");
    const Eigen::Quaternionf truth = axisAngle(1.2f, Eigen::Vector3f(1, 1, 1));
    const TriadObservation obs = makeCase(truth,
                                          Eigen::Vector3f(0, 0, 1),
                                          Eigen::Vector3f(1, 0, 0),
                                          Eigen::Vector3f::Zero(),
                                          Eigen::Vector3f(0.0f, 0.05f, 0.0f));  // 3 deg on secondary

    TriadSolution sol;
    check(triadSolve(obs, TriadConfig(), sol) == TriadResult::OK, "solve returned OK");

    const Eigen::Vector3f mapped = sol.dcmBodyFromRef * obs.primaryRef;
    const float residualDeg = std::acos(std::fmin(1.0f, mapped.dot(obs.primaryBody))) * RAD2DEG;
    std::printf("  primary residual: %.6f deg\n", residualDeg);
    check(residualDeg < 1e-3f, "A * r1 == b1 to float precision");
}

void testRejections() {
    std::printf("rejection paths\n");
    TriadConfig cfg;
    TriadSolution sol;

    // Zero-length input
    TriadObservation zero = makeCase(Eigen::Quaternionf::Identity(),
                                     Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(1, 0, 0));
    zero.primaryBody = Eigen::Vector3f::Zero();
    check(triadSolve(zero, cfg, sol) == TriadResult::DEGENERATE_INPUT, "zero vector rejected");

    // NaN input
    TriadObservation nan = makeCase(Eigen::Quaternionf::Identity(),
                                    Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(1, 0, 0));
    nan.secondaryBody.x() = std::nanf("");
    check(triadSolve(nan, cfg, sol) == TriadResult::DEGENERATE_INPUT, "NaN rejected");

    // Nearly parallel observations (2 deg apart, below the 5 deg default)
    const Eigen::Vector3f a(0, 0, 1);
    const Eigen::Vector3f b = axisAngle(2.0f / RAD2DEG, Eigen::Vector3f(1, 0, 0)) * a;
    const TriadObservation colinear = makeCase(Eigen::Quaternionf::Identity(), a, b);
    check(triadSolve(colinear, cfg, sol) == TriadResult::BODY_COLINEAR, "colinear rejected");

    /*
     * Antiparallel: sin is also ~0 here, which is why the check uses
     * sin and not the dot product.
     */
    const TriadObservation anti =
        makeCase(Eigen::Quaternionf::Identity(), a, Eigen::Vector3f(0, 0, -0.999f));
    check(triadSolve(anti, cfg, sol) == TriadResult::BODY_COLINEAR, "antiparallel rejected");

    /*
     * Inconsistent geometry: body pair 90 deg apart, reference pair 45.
     * Nothing about the individual vectors is wrong; only the pair is.
     */
    TriadObservation bad;
    bad.primaryRef = Eigen::Vector3f(0, 0, 1);
    bad.secondaryRef = Eigen::Vector3f(0.707f, 0, 0.707f).normalized();
    bad.primaryBody = Eigen::Vector3f(0, 0, 1);
    bad.secondaryBody = Eigen::Vector3f(1, 0, 0);
    check(triadSolve(bad, cfg, sol) == TriadResult::GEOMETRY_MISMATCH, "geometry mismatch caught");
}

void testNoiseVsSeparation() {
    /*
     * Why MIN_SEPARATION_DEG exists. Error about the primary axis grows
     * as roughly 1/sin(separation), so the same sensor noise produces
     * wildly different attitude error depending purely on orbit geometry.
     */
    std::printf("attitude error vs observation separation (0.5 deg sensor noise)\n");

    std::mt19937 rng(12345);
    std::normal_distribution<float> noise(0.0f, 0.0087f);  // ~0.5 deg
    const Eigen::Quaternionf truth = axisAngle(0.4f, Eigen::Vector3f(0.1f, 0.9f, 0.3f));

    TriadConfig cfg;
    cfg.minSinSeparation = 0.0f;        // disable the gate so we can see the trend
    cfg.maxGeometryErrorRad = 10.0f;

    for (float sepDeg : {90.0f, 45.0f, 20.0f, 10.0f, 5.0f, 2.0f, 1.0f}) {
        const Eigen::Vector3f ref1(0, 0, 1);
        const Eigen::Vector3f ref2 = axisAngle(sepDeg / RAD2DEG, Eigen::Vector3f(1, 0, 0)) * ref1;

        float sumSq = 0.0f;
        const int N = 500;
        for (int i = 0; i < N; i++) {
            const Eigen::Vector3f n1(noise(rng), noise(rng), noise(rng));
            const Eigen::Vector3f n2(noise(rng), noise(rng), noise(rng));
            const TriadObservation obs = makeCase(truth, ref1, ref2, n1, n2);

            TriadSolution sol;
            if (triadSolve(obs, cfg, sol) == TriadResult::OK) {
                const float e = sol.quatBodyFromRef.angularDistance(truth) * RAD2DEG;
                sumSq += e * e;
            }
        }
        std::printf("  separation %5.1f deg -> RMS attitude error %6.3f deg\n",
                    sepDeg, std::sqrt(sumSq / N));
    }
}

}  // namespace

int main() {
    testExactRecovery();
    testPrimaryIsExact();
    testRejections();
    testNoiseVsSeparation();

    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "PASS" : "FAIL", g_failures);
    return g_failures == 0 ? 0 : 1;
}
