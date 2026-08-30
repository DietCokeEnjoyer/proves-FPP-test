# ======================================================================
# GncTypes.fpp
#
# SINGLE SOURCE OF TRUTH for every data type crossing a component
# boundary in the GNC chain.
#
# ----------------------------------------------------------------------
# NAMING RULES. Follow these when adding anything here.
#
#   1. UNITS AND FRAMES ARE PART OF THE NAME:
#        <quantity><Frame><Unit>
#      e.g. posTemeKm, velTemeKmS, gmstRad, sunRangeKm, altKm.
#      Omit the unit only when the value is genuinely dimensionless
#      (sunUnitTeme). Omit the frame only when it cannot vary.
#
#      A name must not change as a value crosses a boundary: what is
#      posTemeKm in a struct is posTemeKm in every function that
#      handles it.
#
#   2. ANGLES ARE RADIANS in every port payload. Degrees appear only
#      at the telemetry boundary, where the channel name says Deg.
#
#   3. PAYLOAD NOUNS carry meaning and are not interchangeable:
#        State    - a propagated or measured condition of the vehicle
#        Sample   - one observation, from a sensor or a model
#        Solution - an estimate produced by solving something
#      Do not add a fourth.
#
#   4. PUSH PORT TYPES end in Send. The suffix describes the port, not
#      the payload.
#
#   5. USABILITY is reported as one reason enum plus one boolean per
#      independent capability the struct gates. See OrbitState.
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
    @ Unset. Treated as a fault by consumers.
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
    @ present so a star tracker or a ground comparison can be tagged
    @ correctly and rejected loudly instead of silently mixed.
    J2000   = 5
    @ Spacecraft body frame. Sensor outputs.
    BODY    = 6
  }

  # --------------------------------------------------------------------
  # Vectors
  #
  # Two precisions on purpose, and PRECISION IS THE ONLY DIFFERENCE --
  # both are struct {x, y, z} so nothing else reads as different.
  #
  #   Vec3d (F64) for the orbit domain. SGP4 is a double algorithm and
  #   positions of 7000 km with metre-level meaning need the mantissa.
  #
  #   Vec3f (F32) for the attitude and field domains. A unit vector in
  #   F32 is good to ~1e-7, i.e. ~2e-5 deg, four orders of magnitude
  #   below the best sun sensor; a field in nT is good to ~0.005 nT
  #   against a WMM only good to ~150 nT RMS. The RP2350's Cortex-M33
  #   has a hardware SINGLE precision FPU, so this is the difference
  #   between native instructions and soft-float emulation.
  #
  # --------------------------------------------------------------------

  @ Cartesian 3-vector, F64. Units and frame are named by the field
  @ that holds it.
  struct Vec3d {
    x: F64
    y: F64
    z: F64
  } default { x = 0.0, y = 0.0, z = 0.0 }

  @ Cartesian 3-vector, F32. Units and frame are named by the field
  @ that holds it.
  struct Vec3f {
    x: F32 format "{.6f}"
    y: F32 format "{.6f}"
    z: F32 format "{.6f}"
  } default { x = 0.0, y = 0.0, z = 0.0 }

  # --------------------------------------------------------------------
  # Vector observations
  # --------------------------------------------------------------------

  @ One vector observation from a sensor or model, ex: magnetometer, WMM
  @
  @ Not required to be a unit vector, so consumers that need unit vectors 
  @ must normalize the sample.
  struct VectorSample {
    @ The vector. Units and frame come from the producer; see the port
    @ it arrived on.
    vec: Vec3f
    @ Which frame vec is expressed in. Consumers MUST check this. TODO: WHY
    frame: FrameId
    @ When the observation was taken, not when it was sent. Drives
    @ staleness. A producer with no valid clock sends time base
    @ TB_NONE; consumers then fall back to cycle counting.
    stamp: Fw.Time
    @ Producer's own quality flag. False short-circuits any consumer.
    valid: bool
  }

  @ Push port for every vector producer: sun sensor, magnetometer,
  @ solar ephemeris, magnetic field model.
  port VectorSampleSend(ref sample: VectorSample)

  # --------------------------------------------------------------------
  # Orbit
  # --------------------------------------------------------------------

  @ Why the orbit state is / isn't usable. Diagnostic detail; branch
  @ on timeUsable / positionUsable rather than comparing this.
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

  @ Spacecraft orbit state for one cycle, TEME Frame.
  @
  @ The spacecraft-wide navigation product: comms wants it for pass
  @ prediction and antenna pointing, power and thermal for eclipse
  @ forecasting, the magnetic model to evaluate the WMM, the solar
  @ ephemeris for parallax and shadow.
  @
  @ TWO INDEPENDENT CAPABILITY FLAGS, not one. Time can be good while
  @ position is not (NO_TLE, PROP_ERROR), and a consumer that checks a
  @ single "valid" and then reads posTemeKm would get garbage with no
  @ warning. Branch on the flag for the data you are about to read.
  struct OrbitState {
    @ Diagnostic reason. For events and telemetry, not for branching.
    validity: OrbitValidity

    @ True when stamp, jdUt1, jdTt and gmstRad are meaningful.
    timeUsable: bool

    @ True when posTemeKm, velTemeKmS, latRad, lonRad, altKm and
    @ tleAgeDays are meaningful.
    @
    @ Computed HERE rather than by each consumer testing
    @ (validity == VALID || validity == STALE). Two consumers
    @ previously encoded that policy independently and could have
    @ diverged if a sixth validity state were added.
    positionUsable: bool

    @ Wall-clock instant this state describes.
    @
    @ Carried so downstream components can stamp their own outputs with
    @ the OBSERVATION epoch without reading the clock themselves. Only
    @ OrbitPropagator reads the time port for computation.
    stamp: Fw.Time

    @ UT1 Julian date. Drives Earth rotation and the Sun's mean
    @ longitude.
    jdUt1: F64

    @ Terrestrial Time Julian date. Drives the dynamical arguments:
    @ nutation, obliquity, the Sun's mean anomaly.
    @
    @ Carried because the solar ephemeris needs it and receives only
    @ this struct. It was previously approximated downstream as
    @ jdTt = jdUt1, a silent ~69 s error, even though the propagator
    @ had already computed the correct value and discarded it.
    jdTt: F64

    @ Greenwich Mean Sidereal Time at jdUt1, radians, [0, 2pi).
    @
    @ Computed once for the whole subsystem and carried. GMST is the
    @ most error-sensitive quantity here -- 15 deg per hour of clock
    @ error -- and it used to be derived independently in two
    @ components, the second of which quantized it to 16.1 deg by
    @ deriving it from an F32 decimal year. Never recompute this.
    gmstRad: F64

    @ Spacecraft position, TEME, kilometres
    posTemeKm: Vec3d

    @ Spacecraft velocity, TEME, kilometres per second
    velTemeKmS: Vec3d

    @ Sub-satellite geodetic latitude, radians (WGS-84)
    @
    @ Computed here so the WGS-84 conversion runs once per cycle. It
    @ previously ran twice on the same position -- once for the ground
    @ track, once for the magnetic model's altitude gate -- which on a
    @ soft-float M33 is several wasted transcendentals.
    latRad: F64

    @ Sub-satellite geodetic longitude, radians (WGS-84)
    lonRad: F64

    @ Geodetic altitude above the WGS-84 ellipsoid, kilometres
    altKm: F64

    @ Age of the loaded TLE, days. SGP4 degrades roughly 1-3 km/day.
    tleAgeDays: F32
  }

  @ Broadcasts the orbit state.
  port OrbitStateSend(ref state: OrbitState)

  # --------------------------------------------------------------------
  # Solar geometry
  # --------------------------------------------------------------------

  @ Illumination state with respect to the Earth's shadow
  enum IlluminationState: U8 {
    SUNLIT   = 0
    PENUMBRA = 1
    UMBRA    = 2
  }

  @ Why the solar state is / isn't usable. Diagnostic detail; branch
  @ on directionUsable / geometryUsable.
  enum SolarValidity: U8 {
    @ No usable time, so nothing could be computed
    NO_TIME    = 0
    @ Direction computed, but no orbit was available: the vector is
    @ geocentric and shadow / beta are unavailable
    GEOCENTRIC = 1
    @ Fully valid
    VALID      = 2
  }

  @ Solar geometry for one cycle. TEME throughout.
  @
  @ Same two-capability shape as OrbitState, for the same reason: the
  @ Sun direction needs only a clock, while shadow and beta need an
  @ orbit. These previously shared a single "valid" plus an unexplained
  @ "hasOrbit", so a consumer could read betaDeg on a valid struct and
  @ get an uninitialized number.
  struct SolarState {
    @ Diagnostic reason. For events and telemetry, not for branching.
    validity: SolarValidity

    @ True when sunUnitTeme and sunRangeKm are meaningful.
    directionUsable: bool

    @ True when illumination and betaRad are meaningful. Implies
    @ directionUsable.
    geometryUsable: bool

    @ Unit vector toward the Sun, TEME. From the spacecraft when
    @ geometryUsable, otherwise from the Earth's centre -- a difference
    @ of at most 0.0027 deg in LEO, well under the solar model's own
    @ 0.01 deg, so it stays usable for attitude determination.
    sunUnitTeme: Vec3d

    @ Range to the Sun, kilometres
    sunRangeKm: F64

    @ Shadow state
    illumination: IlluminationState

    @ Sun elevation above the orbit plane, RADIANS. Converted to
    @ degrees only at the telemetry boundary.
    betaRad: F64
  }

  @ Broadcasts solar geometry. Consumed by power (eclipse and beta
  @ drive array output and thermal load) and by ADCS.
  port SolarStateSend(ref state: SolarState)

}
