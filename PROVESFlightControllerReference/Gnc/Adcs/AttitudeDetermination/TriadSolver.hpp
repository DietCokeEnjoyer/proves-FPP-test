/**
 * \file TriadSolver.hpp
 * \brief TRIAD (TRI-axial Attitude Determination) solver.
 *
 * \details No FPrime. Uses the Eigen linear algebra library.
 *
 * Algorithim:
 * 
 * Given two non-parallel directions measured in the body frame
 * (b1, b2) and the same two directions known in a reference frame
 * (r1, r2), TRIAD builds an orthonormal basis from each
 * pair and reads the attitude straight off the two bases.
 *
 *   Body triad                      Reference triad
 *     t1b = b1                        t1r = r1
 *     t2b = (b1 x b2)/|b1 x b2|       t2r = (r1 x r2)/|r1 x r2|
 *     t3b = t1b x t2b                 t3r = t1r x t2r
 *
 *   Mb = [t1b t2b t3b]  (columns)   Mr = [t1r t2r t3r]
 *
 * Both Mb and Mr are proper rotation matrices, and by construction the
 * attitude A maps one into the other: A * Mr = Mb. Since Mr is
 * orthonormal, Mr^-1 == Mr^T.
 *
 *   A = Mb * Mr^T           (v_body = A * v_reference)
 *
 * TRIAD assumes one measurement and reference pair (b1 and r1) to be more accurate than the other,
 * so the more accurate sensor reading should be used for this.
 */
#ifndef Adcs_TriadSolver_HPP
#define Adcs_TriadSolver_HPP

#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Gnc {
namespace Adcs {

/*
 * ============================================================================
 * Constants
 * ============================================================================
 */
constexpr float DEG2RAD = 0.017453292519943295f;
constexpr float RAD2DEG = 57.29577951308232f;


/**
 * \brief Why a solve succeeded or failed.
 *
 * \details Mirrors the FPP Adcs::TriadStatus
 */
enum class TriadResult : std::uint8_t {
    OK = 0,                  //!< Solution valid and written to the output
    DEGENERATE_INPUT = 4,    //!< An input was zero length, NaN, or Inf
    BODY_COLINEAR = 5,       //!< Measured pair too close to (anti)parallel
    REFERENCE_COLINEAR = 6,  //!< Modelled pair too close to (anti)parallel
    GEOMETRY_MISMATCH = 7,   //!< Body and reference separations disagree
    NOT_ORTHONORMAL = 8      //!< Result failed the rotation-matrix check
};

/**
 * The four vectors TRIAD needs. primary is the leg that will be
 * satisfied exactly; "secondary" only fixes the roll about it.
 */
struct TriadObservation {
    Eigen::Vector3f primaryBody;     //!< e.g. sun vector from the sun sensors
    Eigen::Vector3f secondaryBody;   //!< e.g. B-field from the magnetometer
    Eigen::Vector3f primaryRef;      //!< same direction from the ephemeris model
    Eigen::Vector3f secondaryRef;    //!< same direction from the IGRF model
};

/**
 * \brief Tuning and fault-detection thresholds.
 *
 * \details Defaults are flight-safe. The component overrides
 * minSinSeparation and maxGeometryErrorRad from FPP parameters; see
 * AttitudeDetermination::currentConfig().
 */
struct TriadConfig {
    //! Reject vectors shorter than this to catch zeroed/uninitalized values
    float minVectorNorm = 1.0e-6f;
   
    /**
     * Reject if sin(angle between the pair) is below this.
     * Attitude error scales roughly as 1/sin(angle), so at 5 deg separation 
     * the secondary sensor's noise is amplified 11x about the primary axis.
     */
    float minSinSeparation = 0.0872f; // 0.0872 rads == 5 deg

    /**
     * Max allowed disagreement between the angle separating the two body 
     * vectors and the angle separating the two reference vectors. They should
     * be identical, so any excess is a sensor, calibration, or ephemeris fault.
     */
    float maxGeometryErrorRad = 0.0872f;  // 0.0872 rads == 5 deg

    //! Max element-wise deviation of A*A^T from identity
    float orthonormalityTol = 1.0e-3f;
};

/**
 * \brief The attitude solution + diagnostics.
 *
 * \details separationRad and geometryErrorRad are populated even on some
 * failure paths, so the component can telemeter them alongside the
 * rejection reason.
 */
struct TriadSolution {
    //! Attitude as a DCM: v_body = dcmBodyFromRef * v_reference
    Eigen::Matrix3f dcmBodyFromRef = Eigen::Matrix3f::Identity();

    //! Same rotation as a unit quaternion, canonicalized to w >= 0
    Eigen::Quaternionf quatBodyFromRef = Eigen::Quaternionf::Identity();

    /**
     * Angle between the two observations, radians
     */
    float separationRad = 0.0f;

    //! |body separation - reference separation|, radians
    float geometryErrorRad = 0.0f;
};

/**
 * \brief Run TRIAD on one observation pair.
 *
 * \details
 *
 * \param obs  The four input vectors. Don't need to be unit, magnitudes
 *             are discarded because TRIAD is direction-only.
 * 
 * \param cfg  Threshold configuration
 * 
 * \param out  [out] Attitude and diagnostics. dcmBodyFromRef and
 *             quatBodyFromRef are written only when OK is returned;
 *             separationRad and geometryErrorRad are written when
 *             they are computable.
 * 
 * \return TriadResult::OK on success, otherwise the first check that failed
 */
TriadResult triadSolve(const TriadObservation& obs,
                       const TriadConfig& cfg,
                       TriadSolution& out);

}  // namespace Adcs
}  // namespace Gnc

#endif  // Adcs_TriadSolver_HPP
