# ======================================================================
# TriadTypes.fpp
#
# Attitude-domain types. Everything that is NOT attitude-specific --
# Vec3f, VectorSample, FrameId -- now lives in Gnc/Types/GncTypes.fpp
# and is shared with the ephemeris and magnetic field models. That is
# what lets the four producers connect to this component at all.
#
# All floating point here is F32 on purpose: the RP2350 Cortex-M33 has
# a hardware single-precision FPU. TRIAD does not need more -- sensor
# error dominates by orders of magnitude.
# ======================================================================

module Gnc {
module Adcs {

  @ Attitude quaternion, stored [x, y, z, w].
  @ Scalar-LAST, Hamilton convention -- this matches Eigen's
  @ Quaternionf::coeffs() memory order exactly, so no reordering is
  @ needed anywhere in the flight code. Identity is [0,0,0,1].
  array Quatf = [4] F32 default [0.0, 0.0, 0.0, 1.0] format "{.6f}"

  @ Which observation TRIAD trusts exactly (the "primary" leg)
  enum PrimaryVector: U8 {
    @ Sun sensor is primary (normal daylight case: sun sensors are
    @ typically 10x more accurate than a magnetometer + WMM model)
    SUN = 0
    @ Magnetometer is primary (only sensible if the sun sensor is degraded)
    MAG = 1
  }

  @ Outcome of one TRIAD evaluation
  enum TriadStatus: U8 {
    @ Valid attitude produced
    OK = 0
    @ Sun observation missing, invalid, or stale (eclipse, saturation)
    SUN_UNAVAILABLE = 1
    @ Magnetic observation missing, invalid, or stale
    MAG_UNAVAILABLE = 2
    @ A reference (inertial) vector was missing or stale
    REFERENCE_UNAVAILABLE = 3
    @ Input vector had ~zero length, NaN, or Inf
    DEGENERATE_INPUT = 4
    @ The two body observations are too close to (anti)parallel:
    @ the cross product that builds the triad is ill-conditioned
    BODY_COLINEAR = 5
    @ Same problem in the reference frame
    REFERENCE_COLINEAR = 6
    @ Angle between the body pair disagrees with the angle between the
    @ reference pair -> a sensor, a calibration, or the ephemeris is wrong
    GEOMETRY_MISMATCH = 7
    @ Resulting matrix failed the orthonormality / handedness check
    NOT_ORTHONORMAL = 8
    @ A producer tagged its sample with a frame this component was not
    @ configured to accept. Body inputs must be BODY; reference inputs
    @ must match REFERENCE_FRAME. This catches the failure mode that
    @ used to be invisible: mixing TEME and J2000 reference vectors is
    @ 0.36 deg of precession error in 2026 and TRIAD cannot detect it
    @ from the numbers alone (both vectors rotate together, so the
    @ geometry consistency check still passes).
    FRAME_MISMATCH = 9
  }

  @ Result published to the controller / estimator
  struct AttitudeSolution {
    @ Rotation from the reference (inertial) frame to the body frame
    q: Quatf
    @ Which inertial frame q is referenced to. Carried so the
    @ controller cannot silently assume the wrong one.
    refFrame: Gnc.FrameId
    @ Why the solution is or is not usable
    status: TriadStatus
    @ Timestamp of the OLDEST observation used. A solution is only as
    @ fresh as its stalest input, and a downstream filter needs to know
    @ which instant this attitude belongs to.
    stamp: Fw.Time
    @ True only when status == OK
    valid: bool
  }

  @ Port used to publish the attitude estimate
  port AttitudeSolutionSend(ref solution: AttitudeSolution)

}
}
