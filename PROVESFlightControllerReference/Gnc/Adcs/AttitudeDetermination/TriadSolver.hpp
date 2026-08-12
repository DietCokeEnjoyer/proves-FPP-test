// ======================================================================
// \title  TriadSolver.hpp
// \brief  TRIAD (TRI-axial Attitude Determination) solver.
//
// Deliberately contains ZERO F Prime dependencies. This lets the whole
// algorithm be unit tested on a workstation with nothing but Eigen, and
// keeps the numerics reviewable independently of the framework wiring.
//
// THE ALGORITHM
// -------------
// Given two non-parallel directions measured in the body frame
// (b1, b2) and the same two directions known in a reference frame
// (r1, r2), TRIAD builds an orthonormal basis ("triad") out of each
// pair and reads the attitude straight off the two bases.
//
//   Body triad                      Reference triad
//     t1b = b1                        t1r = r1
//     t2b = (b1 x b2)/|b1 x b2|       t2r = (r1 x r2)/|r1 x r2|
//     t3b = t1b x t2b                 t3r = t1r x t2r
//
//   Mb = [t1b t2b t3b]  (columns)   Mr = [t1r t2r t3r]
//
// Both Mb and Mr are proper rotation matrices, and by construction the
// attitude A maps one into the other: A * Mr = Mb. Since Mr is
// orthonormal, Mr^-1 == Mr^T, giving the closed-form solution
//
//   A = Mb * Mr^T           (v_body = A * v_reference)
//
// Note the asymmetry: A * r1 == b1 EXACTLY, while r2 is only honoured
// in the plane sense. That is why the more accurate sensor must be the
// primary leg -- TRIAD throws away part of the secondary measurement.
// (QUEST / ESOQ / SVD instead weight both optimally; see README.)
// ======================================================================
#ifndef Adcs_TriadSolver_HPP
#define Adcs_TriadSolver_HPP

#include <cstdint>

// Core + Geometry only. Do NOT include <Eigen/Dense>: it drags in LU,
// QR, SVD, Cholesky and Eigenvalues, which massively inflates compile
// time and flash for a 3x3 problem.
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Gnc {
namespace Adcs {

//! Why a solve succeeded or failed. Mirrors Adcs::TriadStatus in FPP,
//! but declared here so this file stays framework-free.
enum class TriadResult : std::uint8_t {
    OK = 0,
    DEGENERATE_INPUT = 4,
    BODY_COLINEAR = 5,
    REFERENCE_COLINEAR = 6,
    GEOMETRY_MISMATCH = 7,
    NOT_ORTHONORMAL = 8
};

//! The four vectors TRIAD needs. "primary" is the leg that will be
//! satisfied exactly; "secondary" only fixes the roll about it.
struct TriadObservation {
    Eigen::Vector3f primaryBody;     //!< e.g. sun vector from the sun sensors
    Eigen::Vector3f secondaryBody;   //!< e.g. B-field from the magnetometer
    Eigen::Vector3f primaryRef;      //!< same direction from the ephemeris model
    Eigen::Vector3f secondaryRef;    //!< same direction from the IGRF model
};

//! Tuning / fault-detection thresholds.
struct TriadConfig {
    //! Reject vectors shorter than this (catches zeroed or unpopulated data)
    float minVectorNorm = 1.0e-6f;
    //! Reject if sin(angle between the pair) is below this.
    //! 0.0872 == 5 deg. Attitude error scales roughly as 1/sin(angle),
    //! so at 5 deg separation the secondary sensor's noise is amplified
    //! ~11x about the primary axis.
    float minSinSeparation = 0.0872f;
    //! Max allowed disagreement, in radians, between the angle separating
    //! the two BODY vectors and the angle separating the two REFERENCE
    //! vectors. These should be identical (rotation preserves angles), so
    //! any excess is a sensor, calibration, or ephemeris fault.
    float maxGeometryErrorRad = 0.0873f;  // 5 deg
    //! Max element-wise deviation of A*A^T from identity
    float orthonormalityTol = 1.0e-3f;
};

//! Everything the solver produces, including diagnostics worth telemetering.
struct TriadSolution {
    Eigen::Matrix3f dcmBodyFromRef = Eigen::Matrix3f::Identity();
    Eigen::Quaternionf quatBodyFromRef = Eigen::Quaternionf::Identity();
    //! Angle between the two observations, radians, 0..pi. Watch this in
    //! telemetry: it is the single best predictor of solution quality.
    float separationRad = 0.0f;
    //! |body separation - reference separation|, radians
    float geometryErrorRad = 0.0f;
};

//! Run TRIAD. Pure function: no allocation, no state, no I/O, bounded
//! execution time. Safe to call from any thread or from an ISR context.
//!
//! \param obs  the four input vectors (need not be normalized)
//! \param cfg  thresholds
//! \param out  populated only when the return value is OK
//! \return TriadResult::OK on success, otherwise the first check that failed
TriadResult triadSolve(const TriadObservation& obs,
                       const TriadConfig& cfg,
                       TriadSolution& out);

}  // namespace Adcs
}  // namespace Gnc

#endif  // Adcs_TriadSolver_HPP
