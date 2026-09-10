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
 * Normalize a vector.
 * Returns false for zero-length, NaN, or Inf input. 
 * !(n > min) catches zero-length and NaN inputs.
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
 * Angle between two unit vectors, radians in [0, pi]. Computed with atan2 
 * rather than acos(dot) to stay precise with vectors close to parallel.
 */
float angleBetweenUnit(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return std::atan2(a.cross(b).norm(), a.dot(b));
}

}  // namespace

TriadResult triadSolve(const TriadObservation& obs, const TriadConfig& cfg, TriadSolution& out) {
    /*
     * ----------------------------------------------------------------------------
     * Step 1: Normalize
     * TRIAD only needs directions, so this is done first. 
     * Rejects any zero or NaN inputs.
     * ----------------------------------------------------------------------------
     */
    Eigen::Vector3f b1, b2, r1, r2;
    if (!safeNormalize(obs.primaryBody, cfg.minVectorNorm, b1) ||
        !safeNormalize(obs.secondaryBody, cfg.minVectorNorm, b2) ||
        !safeNormalize(obs.primaryRef, cfg.minVectorNorm, r1) ||
        !safeNormalize(obs.secondaryRef, cfg.minVectorNorm, r2)) {
        return TriadResult::DEGENERATE_INPUT;
    }

    /*
     * ----------------------------------------------------------------------------
     * Step 2: Conditioning check.
     * 
     * TRIAD can't be computed when a pair of vectors approach (anti)parallel.
     *
     * The second triad axis is (v1 x v2)/|v1 x v2|, and for unit vectors
     * |v1 x v2| == sin(angle). When the pair approaches (anti)parallel,
     * the norm goes to zero and the division blows up, so the rotation about 
     * the primary axis becomes unobservable.
     * ----------------------------------------------------------------------------
     */
    const Eigen::Vector3f crossBody = b1.cross(b2);
    const Eigen::Vector3f crossRef = r1.cross(r2);
    const float sinBody = crossBody.norm();
    const float sinRef = crossRef.norm();

    out.separationRad = angleBetweenUnit(b1, b2);

    if (sinBody < cfg.minSinSeparation) {
        return TriadResult::BODY_COLINEAR;
    }
    if (sinRef < cfg.minSinSeparation) {
        return TriadResult::REFERENCE_COLINEAR;
    }

    /*
     * ----------------------------------------------------------------------------
     * Step 3: Consistency check
     * 
     * The angle between the measured body vectors should equal the angle between the
     * modeled reference vectors, because they are the same angle in different 
     * frames of reference. If they aren't, there's a sensor or model error.
     * ----------------------------------------------------------------------------
     */
    const float sepRef = angleBetweenUnit(r1, r2);
    out.geometryErrorRad = std::fabs(out.separationRad - sepRef);
    if (out.geometryErrorRad > cfg.maxGeometryErrorRad) {
        return TriadResult::GEOMETRY_MISMATCH;
    }

    /*
     * ----------------------------------------------------------------------------
     * Step 4: Build the two triads
     *
     * Each becomes a proper (right-handed, orthonormal) rotation matrix
     * whose columns are the triad axes.
     *   col0 = the primary direction
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
     * Step 5: The TRIAD solution
     *
     * Both bases describe the same physical triad, expressed in two
     * different frames. The attitude is what maps one to the other:
     *     A * Mr = Mb   ->   A = Mb * Mr^-1 = Mb * Mr^T
     * ----------------------------------------------------------------------------
     */
    const Eigen::Matrix3f A = Mb * Mr.transpose();

    /*
     * ----------------------------------------------------------------------------
     * Step 6: Check for a valid rotation
     * ----------------------------------------------------------------------------
     */
    const float orthErr = (A * A.transpose() - Eigen::Matrix3f::Identity()).cwiseAbs().maxCoeff();
    const float det = A.col(0).dot(A.col(1).cross(A.col(2)));
    if (orthErr > cfg.orthonormalityTol || det < 0.0f) {
        return TriadResult::NOT_ORTHONORMAL;
    }

    /*
     * ----------------------------------------------------------------------------
     * Step 7: Convert DCM -> quaternion.
     *
     * Canonicalize to w >= 0 so a given physical attitude always has one
     * representation (q and -q are the same rotation).
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
