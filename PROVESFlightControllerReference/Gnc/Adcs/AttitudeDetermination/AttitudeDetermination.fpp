# ======================================================================
# AttitudeDetermination.fpp
#
# Passive, guarded inputs.
# ======================================================================

module Gnc {
module Adcs {

  @ TRIAD, deterministic attitude determination from a pair of vector
  @ observations (sun + magnetic field)
  passive component AttitudeDetermination {

    # ------------------------------------------------------------------
    # Data ports
    # ------------------------------------------------------------------

    @ Measured sun direction, body frame, from the sun sensors
    guarded input port sunBodyIn: Gnc.VectorSampleSend

    @ Measured magnetic field direction, body frame, from the magnetometer
    guarded input port magBodyIn: Gnc.VectorSampleSend

    @ Modeled sun direction, inertial frame, from SolarEphemeris
    guarded input port sunRefIn: Gnc.VectorSampleSend

    @ Modeled magnetic field direction, inertial frame, from
    @ MagneticFieldModel
    guarded input port magRefIn: Gnc.VectorSampleSend

    @ Rate group tick. One tick == one TRIAD evaluation.
    guarded input port run: Svc.Sched

    @ Publishes the attitude estimate to the controller / estimator
    output port attitudeOut: AttitudeSolutionSend

    # ------------------------------------------------------------------
    # Special ports
    # ------------------------------------------------------------------
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    event port logOut
    text event port logTextOut
    telemetry port tlmOut
    time get port timeCaller

    param get port prmGetOut
    param set port prmSetOut

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    @ Which observation TRIAD satisfies exactly.
    @
    @ MAG by default.
    @ Typically the sun vector is used as primary, but we have VEML6031 
    @ ambient-light photodiodes, not a dedicated sun sensor. A dedicated sun 
    @ sensor is accurate to about 0.1-1 deg, a weighted-face-normal sun vector 
    @ from ambient-light photodiodes is accurate to ~5-15 deg. Magnetometer + 
    @ WMM is about 2-5 deg, so it's probably more accurate in our case.
    @
    param PRIMARY_VECTOR: PrimaryVector default PrimaryVector.MAG id 0x00

    @ The inertial frame the reference vectors must be tagged with.
    @ TEME, because it's what SGP4 produces.
    param REFERENCE_FRAME: Gnc.FrameId default Gnc.FrameId.TEME id 0x01

    @ Reject a solve if the two observations are separated by less than
    @ this angle (or more than 180 minus this). Attitude error about the
    @ primary axis scales as 1/sin(separation).
    param MIN_SEPARATION_DEG: F32 default 5.0 id 0x02

    @ Reject if the body-pair separation and reference-pair separation
    @ disagree by more than this.
    param MAX_GEOMETRY_ERR_DEG: F32 default 5.0 id 0x03

    @ Discard a body (sensor) sample older than this. Sensors are
    @ should run at or above the ADCS rate.
    param MAX_BODY_AGE_MS: U32 default 500 id 0x04

    @ Discard a reference (model) sample older than this.
    param MAX_REF_AGE_MS: U32 default 3000 id 0x05

    @ Fallback age limit, in rate group cycles, used only when the
    @ sample timestamp cannot be differenced against the current time
    @ (no valid clock yet, mismatched time base, or a clock step).
    param MAX_SAMPLE_AGE_CYCLES: U32 default 20 id 0x06

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    @ Force one TRIAD evaluation immediately using the latched samples
    sync command SOLVE_NOW \
      opcode 0x10

    @ Run TRIAD against a built-in synthetic case with a known answer.
    @ Verifies the math, FPU, and compiler flags on the target.
    sync command SELF_TEST \
      opcode 0x11

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    @ Attitude estimate, [x,y,z,w], reference frame -> body frame
    telemetry AttQuat: Quatf id 0x00

    @ Result of the most recent evaluation
    telemetry Status: TriadStatus id 0x01

    @ Angle between the two observations. The key health channel: watch
    @ this to predict when geometry will force TRIAD to drop out.
    telemetry SeparationDeg: F32 id 0x02 format "{.2f}" 

    @ Body-vs-reference angle disagreement. Should close to the sensor noise. 
    @ A persistent bias means indicates calibration or model error.
    telemetry GeometryErrDeg: F32 id 0x03 format "{.3f}" 

    @ Count of successful solves
    telemetry SolutionCount: U32 id 0x04

    @ Count of rejected solves.
    telemetry SolveRejectCount: U32 id 0x05

    @ Age of the oldest input used in the last attempt, ms.
    @ Look here when the status is *_UNAVAILABLE, tells us if a producer stopped or is slower than expected.
    telemetry OldestInputAgeMs: U32 id 0x06

    @ True when staleness is being judged by cycle count because the
    @ clock is not usable. Expected briefly after boot.
    telemetry UsingCycleFallback: bool id 0x07

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    @ Emitted when TRIAD stops producing solutions
    event SolutionLost(
                        status: TriadStatus @< reason for stopping
                      ) \
      severity warning high \
      id 0x00 \
      format "TRIAD solution lost: {}"

    @ Emitted when solutions resume
    event SolutionRestored \
      severity activity high \
      id 0x01 \
      format "TRIAD solution restored"

    @ Observation geometry too close to (anti)parallel
    event VectorsColinear(
                           separationDeg: F32 @< measured separation
                         ) \
      severity warning low \
      id 0x02 \
      format "TRIAD geometry poor: separation {.2f} deg" \
      throttle 5

    @ Measured and Modeled geometry disagree
    event GeometryMismatch(
                            errorDeg: F32 @< angle residual
                          ) \
      severity warning high \
      id 0x03 \
      format "TRIAD geometry mismatch: {.3f} deg residual" \
      throttle 5

    @ An input has not been refreshed recently enough to use
    event SampleStale(
                       name: string size 16 @< which input
                       ageMs: U32           @< age, ms (0 if unknown)
                     ) \
      severity warning low \
      id 0x04 \
      format "TRIAD input {} stale ({} ms)" \
      throttle 5

    @ A producer tagged a sample with an unexpected frame.
    event FrameMismatch(
                         name: string size 16   @< which input
                         got: Gnc.FrameId       @< frame received
                         expected: Gnc.FrameId  @< frame required
                       ) \
      severity warning high \
      id 0x05 \
      format "TRIAD input {} in frame {}, expected {}" \
      throttle 3

    @ Self test result
    event SelfTestResult(
                          errorDeg: F32 @< recovered attitude error
                          passed: bool  @< true if within tolerance
                        ) \
      severity activity high \
      id 0x06 \
      format "TRIAD self test error {.5f} deg, passed={}"

  }

}
}
