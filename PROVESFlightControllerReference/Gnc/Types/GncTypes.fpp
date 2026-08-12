# ======================================================================
# GncTypes.fpp
#
# SINGLE SOURCE OF TRUTH for every data type crossing a component
# boundary in the GNC chain.
#
# Before this refactor there were three parallel type systems for the
# same physical quantities:
#
#   Components::EciPosition / MagFieldEci   (F32, "ECI")
#   Adcs::Vec3f / VectorSample              (F32, frame implicit)
#   Gnc::Vector3 / EphemerisSolution        (F64, TEME; now split
#                                            into OrbitState and
#                                            SolarState)
#
# Nothing connected, and the two that *could* have connected disagreed
# about units (km vs m) and about which inertial frame "ECI" meant.
# Both are silent failures: the code runs and produces a plausible,
# wrong answer.
#
# Two rules are now enforced by the type system rather than by comment:
#
#   1. Every vector on a port carries an explicit FrameId. Consumers
#      check it. Mixing TEME and J2000 is a 0.36 deg error in 2026 and
#      is the single most likely bug in this subsystem.
#   2. Units are in the field name or the doc comment, always, and
#      the km/m boundary exists in exactly one place (the WMM call).
# ======================================================================

module Gnc {

  # --------------------------------------------------------------------
  # Frames
  # --------------------------------------------------------------------

  @ Identifies the reference frame a vector is expressed in.
  @
  @ TEME is the system-wide inertial frame, because that is what SGP4
  @ natively produces and converting it is extra code that can be wrong.
  @ The Sun model (MOD) and the magnetic model (ECEF) both rotate INTO
  @ TEME at their own boundary. Nothing downstream converts anything.
  enum FrameId: U8 {
    @ Unset. Treated as a fault by every consumer.
    UNKNOWN = 0
    @ True Equator, Mean Equinox of date. The system inertial frame.
    TEME    = 1
    @ Mean Equator, Mean Equinox of date. Solar model output only.
    MOD     = 2
    @ True Equator, True Equinox of date. Intermediate only.
    TOD     = 3
    @ Earth-fixed (ITRS/PEF, polar motion neglected). WMM input only.
    ECEF    = 4
    @ Mean equator and equinox of J2000.0. NOT used in the flight path;
    @ present so that a star tracker or a ground comparison can be
    @ tagged correctly and rejected loudly instead of silently mixed.
    J2000   = 5
    @ Spacecraft body frame. Sensor outputs.
    BODY    = 6
  }

  # --------------------------------------------------------------------
  # Vectors
  #
  # Two precisions on purpose:
  #
  #   Vector3 (F64) for the orbit/ephemeris domain. SGP4 is a double
  #   algorithm and positions of 7000 km with metre-level meaning need
  #   the mantissa.
  #
  #   Vec3f (F32) for the attitude domain, which is direction-only.
  #   A unit vector in F32 is good to ~1e-7, i.e. ~2e-5 deg, which is
  #   four orders of magnitude below the best sun sensor. The RP2350's
  #   Cortex-M33 has a hardware SINGLE precision FPU, so this is the
  #   difference between native instructions and soft-float emulation.
  #
  # The F64 -> F32 narrowing happens exactly once, at the point a model
  # publishes a direction for the attitude chain.
  # --------------------------------------------------------------------

  @ Cartesian 3-vector, F64. Units depend on the containing field.
  struct Vector3 {
    x: F64
    y: F64
    z: F64
  }

  @ Cartesian 3-vector, F32. Used for directions in the attitude chain.
  array Vec3f = [3] F32 default [0.0, 0.0, 0.0] format "{.6f}"

  # --------------------------------------------------------------------
  # Vector observations
  # --------------------------------------------------------------------

  @ One vector observation, from a sensor or from an on-board model.
  @
  @ This is the common currency of the attitude chain: sun sensor,
  @ magnetometer, solar ephemeris and the WMM all emit exactly this.
  struct VectorSample {
    @ The direction. Consumers normalize defensively, so producers are
    @ not required to.
    unitVec: Vec3f
    @ Which frame unitVec is expressed in. Consumers MUST check this.
    frame: FrameId
    @ When the observation was taken (not when it was sent). Drives
    @ staleness. A producer with no valid clock should send a Fw.Time
    @ with time base TB_NONE; consumers then fall back to cycle
    @ counting instead of wall clock.
    stamp: Fw.Time
    @ Producer's own quality flag. False short-circuits any consumer.
    valid: bool
  }

  @ Port used by every vector producer: sun sensor, magnetometer,
  @ solar ephemeris, magnetic field model.
  port VectorSampleSend(ref sample: VectorSample)

  # --------------------------------------------------------------------
  # Orbit
  # --------------------------------------------------------------------

  @ Confidence in the current orbit solution
  enum OrbitValidity: U8 {
    @ No TLE has been loaded since boot. Time and GMST are still valid.
    NO_TLE      = 0
    @ Time source is not usable. NOTHING in the state is meaningful.
    NO_TIME     = 1
    @ SGP4 returned a non-zero error code
    PROP_ERROR  = 2
    @ Solution is good but the TLE is older than MAX_TLE_AGE_DAYS
    STALE       = 3
    @ Fully valid
    VALID       = 4
  }

  @ Spacecraft orbit state for one cycle. TEME throughout.
  @
  @ This is the spacecraft-wide navigation product. Comms wants it for
  @ pass prediction and antenna pointing, power and thermal want it for
  @ eclipse forecasting, the magnetic model wants it to evaluate the
  @ WMM, and the solar ephemeris wants it for parallax and shadow. It
  @ is deliberately NOT bundled with the Sun vector: those are
  @ different products with different audiences.
  struct OrbitState {
    validity: OrbitValidity

    @ Wall-clock instant this state describes.
    @
    @ Carried so that downstream components can stamp their own
    @ outputs with the OBSERVATION epoch without calling the time port
    @ themselves. Only OrbitPropagator reads the clock; see the note on
    @ gmstRad below for what happens when that rule is broken.
    stamp: Fw.Time

    @ UT1 Julian date of the solution
    jdUt1: F64

    @ Greenwich Mean Sidereal Time at jdUt1, radians, [0, 2pi).
    @
    @ Computed ONCE, here, in F64, and carried downstream. GMST is the
    @ most error-sensitive quantity in the subsystem -- 15 deg per hour
    @ of clock error -- and it used to be derived independently in two
    @ components, the second of which quantized it to 16.1 deg by
    @ deriving it from an F32 decimal year. Downstream components must
    @ treat this as read-only input and must never recompute it.
    gmstRad: F64

    @ Spacecraft position, TEME, km. Meaningless unless validity is
    @ VALID or STALE.
    posTeme: Vector3

    @ Spacecraft velocity, TEME, km/s. Same caveat.
    velTeme: Vector3

    @ Unit vector from spacecraft to Earth centre, TEME
    nadirTeme: Vector3

    @ Age of the loaded TLE, days. SGP4 degrades roughly 1-3 km/day.
    tleAgeDays: F32
  }

  @ Broadcasts the orbit state. Consumed by the solar ephemeris, the
  @ magnetic field model, and anything else that needs to know where
  @ the spacecraft is.
  port OrbitUpdate(ref state: OrbitState)

  # --------------------------------------------------------------------
  # Solar geometry
  # --------------------------------------------------------------------

  @ Illumination state with respect to the Earth's shadow
  enum IlluminationState: U8 {
    SUNLIT   = 0
    PENUMBRA = 1
    UMBRA    = 2
  }

  @ Solar geometry for one cycle. TEME throughout.
  struct SolarState {
    @ True when sunUnitTeme is usable as a direction. Note this can be
    @ true while hasOrbit is false: the Sun direction only needs a
    @ clock, not an ephemeris.
    valid: bool

    @ True when the spacecraft position was known, so that parallax was
    @ applied and illumination / betaDeg are meaningful. When false,
    @ sunUnitTeme is GEOCENTRIC -- which in LEO differs from the
    @ topocentric direction by at most 0.0027 deg, well under the solar
    @ model's own 0.01 deg, so it remains perfectly usable for attitude
    @ determination and for a safe-mode sun search.
    hasOrbit: bool

    @ Unit vector toward the Sun, TEME. From the spacecraft if
    @ hasOrbit, otherwise from the Earth's centre.
    sunUnitTeme: Vector3

    @ Range to the Sun, km
    sunRangeKm: F64

    @ Shadow state. Only meaningful when hasOrbit.
    illumination: IlluminationState

    @ Sun elevation above the orbit plane, degrees. Only meaningful
    @ when hasOrbit.
    betaDeg: F32
  }

  @ Broadcasts solar geometry. Consumed by power (eclipse and beta
  @ drive array output and thermal load) and by ADCS.
  port SolarUpdate(ref state: SolarState)

  # --------------------------------------------------------------------
  # Magnetic field
  # --------------------------------------------------------------------

  @ Magnetic field vector, nanotesla. Frame is carried alongside.
  @
  @ F32: the field is 20000-65000 nT in LEO and the WMM itself is only
  @ good to ~150 nT RMS, so F32's ~7 digits are five orders of
  @ magnitude finer than the model error.
  struct MagFieldVec {
    x: F32
    y: F32
    z: F32
  }

  @ Field plus an explicit validity flag.
  @
  @ The old design returned a bare vector, so "model declined to
  @ evaluate" and "field is genuinely near zero" were the same three
  @ zeros on the wire. They are not the same thing.
  struct MagFieldResult {
    field: MagFieldVec
    @ Frame the field vector is expressed in (TEME on the flight path)
    frame: FrameId
    @ Epoch the model was evaluated at
    stamp: Fw.Time
    valid: bool
  }

  @ Push the full field vector, magnitude included. The magnetorquer
  @ controller needs |B|, not just its direction, so it takes this
  @ rather than the VectorSample that TRIAD takes.
  port MagFieldSend(ref result: MagFieldResult)

  @ Synchronous request/response: field at an arbitrary position and
  @ time. Kept as a pull interface for ground checkout and for a
  @ magnetorquer allocator that wants B somewhere other than "now".
  @ The normal flight path does NOT use this -- see magRefOut.
  @
  @ posTemeKm is F64 and in KILOMETRES, matching SGP4. The conversion
  @ to the metres that the WMM wants happens inside the component, in
  @ exactly one place. Passing metres here used to be assumed and never
  @ stated, and nothing in the system produced metres, so the model
  @ would have been evaluated 1000x too close to the Earth's centre.
  port MagFieldRequest(
                        ref posTemeKm: Vector3  @< position, TEME, km
                        jdUt1: F64              @< UT1 Julian date
                        gmstRad: F64            @< GMST at jdUt1, rad
                      ) -> MagFieldResult

}
