# ======================================================================
# OrbitPropagator.fpp
#
# SGP4 propagation and the time scales it needs.
#
# The only component in the GNC subsystem that reads the time port
# for computation. jdUt1, gmstRad and the Fw::Time stamp flow downstream
# inside OrbitState. Downstream components use their time ports to 
# timestamp events and telemetry, not for computation.
# ======================================================================

module Gnc {
module Environment {

  active component OrbitPropagator {

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    @ Rate group tick.
    async input port run: Svc.Sched

    @ Broadcasts the orbit state.
    @
    @ This call is synchronous into whatever it is connected
    @ to, on this component's thread. Connected to the magnetic field model
    @ and the solar ephemeris so both evaluate with the same position and epoch
    @ from this component.
    output port orbitOut: Gnc.OrbitStateSend

    # ------------------------------------------------------------------
    # Standard framework ports
    # ------------------------------------------------------------------

    command recv  port cmdIn
    command reg   port cmdRegOut
    command resp  port cmdResponseOut

    event      port logOut
    text event port logTextOut
    telemetry  port tlmOut
    time get   port timeCaller

    param get port prmGetOut
    param set port prmSetOut

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    @ Upload a two-line element set. Both lines must be exactly 69
    @ characters.
    async command LOAD_TLE(
                            line1: string size 80 @< TLE line 1
                            line2: string size 80 @< TLE line 2
                          ) \
      opcode 0x10

    @ Discard the current TLE.
    async command CLEAR_TLE \
      opcode 0x11

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    @ TAI - UTC, seconds. Bump when a leap second is announced.
    param LEAP_SEC: F64 default 37.0 id 0x00

    @ UT1 - UTC, seconds, from IERS Bulletin A. Always < 0.9 s.
    @ Leaving this at 0 is at most 0.004 deg of error in Earth's rotation.
    param DUT1_SEC: F64 default 0.0 id 0x01

    @ TLE age in days beyond which the state is reported STALE.
    param MAX_TLE_AGE_DAYS: F32 default 7.0 id 0x02

    @ Offset added to the Fw::Time seconds field to obtain UTC POSIX
    @ seconds. Use if time source is GPS time or TAI.
    param UTC_OFFSET_SEC: F64 default 0.0 id 0x03

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    @ Spacecraft position, TEME, kilometers
    telemetry PosTemeKm: Gnc.Vec3d id 0x00

    @ Spacecraft velocity, TEME, kilometers per second
    telemetry VelTemeKmS: Gnc.Vec3d id 0x01

    @ Sub-satellite geodetic latitude, deg
    telemetry LatDeg: F32 id 0x02

    @ Sub-satellite geodetic longitude, deg
    telemetry LonDeg: F32 id 0x03

    @ Geodetic altitude, km
    telemetry AltKm: F32 id 0x04

    @ Age of the loaded TLE, days
    telemetry TleAgeDays: F32 id 0x05

    @ Validity of the current state
    telemetry Validity: Gnc.OrbitValidity id 0x06

    @ Greenwich Mean Sidereal Time, deg.
    telemetry GmstDeg: F32 id 0x07

    @ Wall time consumed by the last tick, microseconds.
    telemetry CycleUsec: U32 id 0x08

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    @ TLE parsed and the propagator initialized successfully
    event TleAccepted(
                       satnum: U32  @< NORAD catalog number
                       epochJd: F64 @< TLE epoch, Julian date
                     ) \
      severity activity high \
      id 0x00 \
      format "TLE accepted for object {} at epoch JD {f}"

    @ TLE rejected. Code is the perturb::Sgp4Error value. The
    @ previously loaded TLE is left untouched.
    event TleRejected(
                       code: U8
                     ) \
      severity warning high \
      id 0x01 \
      format "TLE rejected, SGP4 init error {}"

    @ SGP4 propagation failed this tick
    event Sgp4Failure(
                       code: U8
                       minsFromEpoch: F64
                     ) \
      severity warning high \
      id 0x02 \
      format "SGP4 error {} at {f} min from epoch" \
      throttle 5

    @ TLE older than MAX_TLE_AGE_DAYS
    event TleStale(
                    ageDays: F32
                  ) \
      severity warning low \
      id 0x03 \
      format "TLE is {f} days old, uplink a fresh one" \
      throttle 3

    @ No TLE loaded.
    event TleMissing \
      severity warning low \
      id 0x04 \
      format "No TLE loaded, spacecraft state unavailable" \
      throttle 3

    @ Time source reported invalid.
    event TimeMissing \
      severity warning high \
      id 0x05 \
      format "Time source invalid, navigation suspended" \
      throttle 3

    @ TLE cleared by command
    event TleCleared \
      severity activity high \
      id 0x06 \
      format "TLE cleared"

  }

}
}
