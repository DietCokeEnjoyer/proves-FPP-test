/**
 * \file TriadSolver.cpp
 * \brief TRIAD implementation. See TriadSolver.hpp for the derivation.
 */
#include "TriadSolver.hpp"

#include <cmath>

namespace Gnc {
namespace Adcs {

namespace {

/**
 * Normalize with a length + finiteness guard.
 * Returns false for zero-length, NaN, or Inf input. The !(n > min)
 * form (rather than n <= min) is deliberate: it is also false when n
 * is NaN, so a single comparison catches both failure modes.
 */
bool safeNormalize(const Eigen::Vector3f& in, float minNorm, Eigen::Vector3f& out) {
    const float n = in.norm();
    if (!(n > minNorm) || !std::isfinite(n)) {
        return false;
    }
    out = in / n;
    return true;
}

/**
 * Angle between two unit vectors, 0..pi, computed with atan2 rather
 * than acos(dot). acos loses precision badly for near-parallel
 * vectors -- exactly the regime where we need an accurate answer in
 * order to decide whether to reject the solution.
 */
float angleBetweenUnit(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return std::atan2(a.cross(b).norm(), a.dot(b));
}

}  // namespace

TriadResult triadSolve(const TriadObservation& obs,
                       const TriadConfig& cfg,
                       TriadSolution& out) {
    /*
     * ----------------------------------------------------------------------------
     * STEP 1: Normalize. TRIAD is a direction-only algorithm -- the
     * magnitudes of the sun vector and the B-field carry no attitude
     * information, so we discard them immediately. This also rejects
     * unpopulated (all-zero) and NaN inputs before they can poison the
     * rest of the computation.
     * ----------------------------------------------------------------------------
     */
    Eigen::Vector3f b1, b2, r1, r2;
    if (!safeNormalize(obs.primaryBody,   cfg.minVectorNorm, b1) ||
        !safeNormalize(obs.secondaryBody, cfg.minVectorNorm, b2) ||
        !safeNormalize(obs.primaryRef,    cfg.minVectorNorm, r1) ||
        !safeNormalize(obs.secondaryRef,  cfg.minVectorNorm, r2)) {
        return TriadResult::DEGENERATE_INPUT;
    }

    /*
     * ----------------------------------------------------------------------------
     * STEP 2: Conditioning check.
     *
     * The second triad axis is (v1 x v2)/|v1 x v2|. For unit vectors
     * |v1 x v2| == sin(angle). As the pair approaches parallel or
     * antiparallel that norm goes to zero, the division blows up, and
     * the rotation about the primary axis becomes unobservable. This is
     * a real operational case: the sun-Earth-B-field geometry genuinely
     * lines up over parts of an orbit, and TRIAD must decline rather
     * than emit a confident garbage answer.
     * ----------------------------------------------------------------------------
     */
    const Eigen::Vector3f crossBody = b1.cross(b2);
    const Eigen::Vector3f crossRef  = r1.cross(r2);
    const float sinBody = crossBody.norm();
    const float sinRef  = crossRef.norm();

    out.separationRad = angleBetweenUnit(b1, b2);

    if (sinBody < cfg.minSinSeparation) {
        return TriadResult::BODY_COLINEAR;
    }
    if (sinRef < cfg.minSinSeparation) {
        return TriadResult::REFERENCE_COLINEAR;
    }

    /*
     * ----------------------------------------------------------------------------
     * STEP 3: Free consistency check.
     *
     * A rotation preserves angles. The angle between the two measured
     * body vectors must therefore equal the angle between the two
     * modelled reference vectors. TRIAD never uses this fact, which
     * means it costs nothing to spend it on fault detection instead:
     * a large residual means a magnetometer that needs recalibration,
     * a sun sensor seeing Earth albedo, a stale TLE, or swapped axes.
     * Catching that here is far cheaper than debugging it downstream.
     * ----------------------------------------------------------------------------
     */
    const float sepRef = angleBetweenUnit(r1, r2);
    out.geometryErrorRad = std::fabs(out.separationRad - sepRef);
    if (out.geometryErrorRad > cfg.maxGeometryErrorRad) {
        return TriadResult::GEOMETRY_MISMATCH;
    }

    /*
     * ----------------------------------------------------------------------------
     * STEP 4: Build the two triads.
     *
     * Each becomes a proper (right-handed, orthonormal) rotation matrix
     * whose COLUMNS are the triad axes.
     *   col0 = the primary direction itself
     *   col1 = normal to the plane containing both observations
     *   col2 = completes the right-handed set
     * ----------------------------------------------------------------------------
     */
    const Eigen::Vector3f t1b = b1;
    const Eigen::Vector3f t2b = crossBody / sinBody;
    const Eigen::Vector3f t3b = t1b.cross(t2b);

    const Eigen::Vector3f t1r = r1;
    const Eigen::Vector3f t2r = crossRef / sinRef;
    const Eigen::Vector3f t3r = t1r.cross(t2r);

    Eigen::Matrix3f Mb;
    Mb.col(0) = t1b;
    Mb.col(1) = t2b;
    Mb.col(2) = t3b;

    Eigen::Matrix3f Mr;
    Mr.col(0) = t1r;
    Mr.col(1) = t2r;
    Mr.col(2) = t3r;

    /*
     * ----------------------------------------------------------------------------
     * STEP 5: The TRIAD solution itself.
     *
     * Both bases describe the SAME physical triad, expressed in two
     * different frames. The attitude is whatever maps one to the other:
     *     A * Mr = Mb   ->   A = Mb * Mr^-1 = Mb * Mr^T
     * The transpose is exact (no inverse is ever computed) because Mr
     * is orthonormal by construction. That is the whole trick, and it
     * is why TRIAD is a handful of flops with no iteration.
     * ----------------------------------------------------------------------------
     */
    const Eigen::Matrix3f A = Mb * Mr.transpose();

    /*
     * ----------------------------------------------------------------------------
     * STEP 6: Verify the result is a valid rotation.
     *
     * Cheap insurance against accumulated float error, a mis-wired
     * frame convention, or an FPU/compiler-flag problem on the target.
     * determinant() is computed by hand as the scalar triple product to
     * avoid pulling in the Eigen LU module.
     * ----------------------------------------------------------------------------
     */
    const float orthErr =
        (A * A.transpose() - Eigen::Matrix3f::Identity()).cwiseAbs().maxCoeff();
    const float det = A.col(0).dot(A.col(1).cross(A.col(2)));
    if (orthErr > cfg.orthonormalityTol || det < 0.0f) {
        return TriadResult::NOT_ORTHONORMAL;
    }

    /*
     * ----------------------------------------------------------------------------
     * STEP 7: Convert DCM -> quaternion.
     *
     * Eigen's matrix-to-quaternion conversion uses the branch-selecting
     * (Shepperd) method: it picks whichever of the trace or the three
     * diagonal elements is largest and derives the quaternion from that
     * branch, avoiding the catastrophic cancellation the naive
     * sqrt(1 + trace) formula suffers near 180 deg rotations.
     *
     * Canonicalize to w >= 0 so a given physical attitude always has one
     * representation (q and -q are the same rotation). The component
     * layer additionally enforces sign continuity against the previous
     * published solution, which is what downstream filters care about.
     * ----------------------------------------------------------------------------
     */
    Eigen::Quaternionf q(A);
    q.normalize();
    if (q.w() < 0.0f) {
        q.coeffs() *= -1.0f;
    }

    out.dcmBodyFromRef = A;
    out.quatBodyFromRef = q;
    return TriadResult::OK;
}

}  // namespace Adcs
}  // namespace Gnc
