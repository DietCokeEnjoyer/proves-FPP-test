# ======================================================================
# TriadTypes.fpp
#
# Attitude-domain types.
# ======================================================================

module Gnc {
module Adcs {

  @ Attitude quaternion, stored [x, y, z, w].
  @ Scalar-LAST, Hamilton convention. This matches Eigen's 
  @ Quaternionf::coeffs() memory order, so no reordering is needed. 
  @ Identity is [0,0,0,1].
  array Quatf = [4] F32 default [0.0, 0.0, 0.0, 1.0] format "{.6f}"

  @ Which observation TRIAD trusts exactly. Use the most accurate measurement.
  enum PrimaryVector: U8 {
    @ Sun sensor is primary
    SUN = 0
    @ Magnetometer is primary
    MAG = 1
  }

  @ Outcome of one TRIAD evaluation
  enum TriadStatus: U8 {
    @ Valid attitude produced
    OK = 0
    @ Sun observation missing, invalid, or stale.
    SUN_UNAVAILABLE = 1
    @ Magnetic observation missing, invalid, or stale
    MAG_UNAVAILABLE = 2
    @ A reference vector was missing or stale
    REFERENCE_UNAVAILABLE = 3
    @ Input vector had zero length, NaN, or Inf
    DEGENERATE_INPUT = 4
    @ The observed vectors are too close to (anti)parallel
    @ for TRIAD's cross product computations.
    BODY_COLINEAR = 5
    @ The reference vectors are too close to (anti)parallel
    @ for TRIAD's cross product computations.
    REFERENCE_COLINEAR = 6
    @ Angle between the body pair disagrees with the angle between the
    @ reference pair
    GEOMETRY_MISMATCH = 7
    @ Resulting matrix failed the orthonormality / handedness check
    NOT_ORTHONORMAL = 8
    @ A producer tagged its sample with a frame this component doesn't accept
    FRAME_MISMATCH = 9
  }

  @ The attitude estimate to be broadcast
  struct AttitudeSolution {
    @ Rotation from the reference (inertial) frame to the body frame
    q: Quatf
    @ Which inertial frame q is referenced to.
    refFrame: Gnc.FrameId
    @ Why the solution is or isn't usable
    status: TriadStatus
    @ Timestamp of the oldest observation used.
    stamp: Fw.Time
    @ True only when status == OK
    valid: bool
  }

  @ Port used to publish the attitude estimate
  port AttitudeSolutionSend(ref solution: AttitudeSolution)

}
}
