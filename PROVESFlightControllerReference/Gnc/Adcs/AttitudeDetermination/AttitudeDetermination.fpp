# ======================================================================
# AttitudeDetermination.fpp
#
# PASSIVE on purpose.
#
# The solve is a few dozen floating-point operations -- microseconds on
# a 150 MHz M33 with a hardware FPU. Making this active would buy a
# thread and its stack (~2-4 KB of the RP2350's 520 KB SRAM) plus a
# message queue, to hide latency that does not exist. Instead it runs
# directly on the rate group thread.
#
# The data-input ports are GUARDED rather than sync: sensor components
# and the model chain call them from their own threads, so the latched
# state needs a mutex. Guarded ports give exactly that, with no queue
# and no thread.
#
# WHAT CHANGED AND WHY
# --------------------
# * The four inputs now take Gnc.VectorSample, the same struct the
#   ephemeris and the WMM already emit, instead of a private Adcs copy
#   of the same idea. This is what actually connects the subsystem.
# * Staleness is measured in MILLISECONDS against the sample timestamp,
#   with cycle counting kept only as the fallback for a spacecraft that
#   has booted without a valid clock. The old cycle-only scheme was a
#   latent showstopper: with the ephemeris at 1 Hz, this component at
#   10 Hz, and MAX_SAMPLE_AGE_CYCLES defaulting to 5, the reference
#   vectors were stale on 9 ticks out of 10 and TRIAD would have
#   produced a solution roughly never.
# * Body and reference inputs get SEPARATE age limits, because they
#   legitimately update at different rates -- a magnetometer at 10 Hz
#   and an orbit propagator at 1 Hz are both healthy.
# * Every input's frame tag is checked. See TriadStatus.FRAME_MISMATCH.
# ======================================================================

module Gnc {
module Adcs {

  @ TRIAD deterministic attitude determination from a pair of vector
  @ observations (sun + magnetic field)
  passive component AttitudeDetermination {

    # ------------------------------------------------------------------
    # Data ports
    #
    # Four inputs, deliberately symmetric. The two BODY vectors come
    # from hardware; the two REFERENCE vectors come from on-board models
    # (solar ephemeris and the WMM evaluated at the propagated orbit
    # position). Keeping the reference vectors as ports rather than
    # subscribing to OrbitState or SolarState here keeps this
    # component purely about TRIAD, and lets the models be swapped or
    # stubbed for test without an orbit propagator in the loop.
    # ------------------------------------------------------------------

    @ Measured sun direction, body frame, from the sun sensor component
    guarded input port sunBodyIn: Gnc.VectorSampleSend

    @ Measured magnetic field direction, body frame, from the magnetometer
    guarded input port magBodyIn: Gnc.VectorSampleSend

    @ Modelled sun direction, inertial frame, from SolarEphemeris
    guarded input port sunRefIn: Gnc.VectorSampleSend

    @ Modelled magnetic field direction, inertial frame, from
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
    #
    # Everything tunable is a parameter, not a constant, so the geometry
    # rejection thresholds can be adjusted on orbit once real sensor
    # performance is known -- without a software load.
    # ------------------------------------------------------------------

    @ Which observation TRIAD satisfies exactly.
    @
    @ MAG by default, which is the opposite of the usual advice. The
    @ usual advice assumes a dedicated sun sensor at 0.1-1 deg against
    @ a magnetometer + model at 2-5 deg. This spacecraft's sun sensor
    @ is an array of VEML6031 ambient-light photodiodes, one per face;
    @ a weighted-face-normal sun vector from those is realistically
    @ 5-15 deg, and degrades further near face boundaries and under
    @ Earth albedo. The LIS2MDL plus the WMM at the propagated orbit
    @ position is the better of the two.
    @
    @ TRIAD reproduces the PRIMARY observation exactly and uses the
    @ secondary only to fix rotation about it, so the primary must be
    @ the more accurate sensor. Flip this to SUN if you fly a real sun
    @ sensor, or if magnetometer calibration turns out worse than the
    @ photodiode array in flight.
    param PRIMARY_VECTOR: PrimaryVector default PrimaryVector.MAG id 0x00

    @ The inertial frame the reference vectors must be tagged with.
    @ TEME, because that is what SGP4 produces and the whole chain was
    @ built around not converting it. Any producer that disagrees is
    @ rejected loudly rather than averaged in silently.
    param REFERENCE_FRAME: Gnc.FrameId default Gnc.FrameId.TEME id 0x01

    @ Reject a solve if the two observations are separated by less than
    @ this angle (or more than 180 minus this). Attitude error about the
    @ primary axis scales as 1/sin(separation).
    param MIN_SEPARATION_DEG: F32 default 5.0 id 0x02

    @ Reject if the body-pair separation and reference-pair separation
    @ disagree by more than this. A rotation preserves angles, so a
    @ nonzero residual is always a fault somewhere.
    param MAX_GEOMETRY_ERR_DEG: F32 default 5.0 id 0x03

    @ Discard a BODY (sensor) sample older than this. Sensors are
    @ expected to run at or above the ADCS rate, so this is tight.
    param MAX_BODY_AGE_MS: U32 default 500 id 0x04

    @ Discard a REFERENCE (model) sample older than this. The orbit
    @ propagator runs at 1 Hz and the underlying quantities move
    @ slowly -- the Sun by 0.0000042 deg per second, the modelled
    @ field by well under a degree -- so several seconds of tolerance
    @ costs almost nothing and prevents a rate mismatch from
    @ suppressing the solution entirely.
    param MAX_REF_AGE_MS: U32 default 3000 id 0x05

    @ Fallback age limit, in rate group cycles, used ONLY when the
    @ sample timestamp cannot be differenced against the current time
    @ (no valid clock yet, mismatched time base, or a clock step).
    @ Counting cycles avoids any dependence on time base validity at
    @ power-on, which is exactly the regime where a coarse sun search
    @ needs an attitude most.
    param MAX_SAMPLE_AGE_CYCLES: U32 default 20 id 0x06

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    @ Force one TRIAD evaluation immediately using the latched samples
    sync command SOLVE_NOW \
      opcode 0x10

    @ Run TRIAD against a built-in synthetic case with a known answer.
    @ Verifies the math, the FPU, and the compiler flags on the target.
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

    @ Body-vs-reference angle disagreement. Should hover near sensor
    @ noise; a persistent bias means a calibration or model error.
    telemetry GeometryErrDeg: F32 id 0x03 format "{.3f}" 

    @ Count of successful solves
    telemetry SolutionCount: U32 id 0x04

    @ Count of rejected solves. Named for this component: the ground
    @ flattens the namespace and a bare RejectCount would collide with
    @ the magnetic field model's.
    telemetry SolveRejectCount: U32 id 0x05

    @ Age of the oldest input used in the last attempt, ms. The channel
    @ to look at when the status is *_UNAVAILABLE: it tells you whether
    @ a producer stopped or is merely slower than you assumed.
    telemetry OldestInputAgeMs: U32 id 0x06

    @ True when staleness is being judged by cycle count because the
    @ clock is not usable. Expected briefly after boot; if it stays
    @ true, the time source never came up.
    telemetry UsingCycleFallback: bool id 0x07

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    @ Emitted when TRIAD stops producing solutions
    event SolutionLost(
                        status: TriadStatus @< reason for the dropout
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

    @ Measured and modelled geometry disagree -- suspect a sensor,
    @ a calibration, or the reference model
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

    @ A producer tagged a sample with an unexpected frame. This is a
    @ wiring or configuration fault, not a transient: it will not clear
    @ on its own, and until it does the attitude solution is suspended.
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
