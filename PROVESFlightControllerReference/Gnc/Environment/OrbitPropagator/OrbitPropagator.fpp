# ======================================================================
# OrbitPropagator.fpp
#
# SGP4 propagation and the time scales it needs. Publishes the
# spacecraft-wide navigation product.
#
# WHY THIS IS ITS OWN COMPONENT
# -----------------------------
# Orbit state and solar geometry have different audiences. Position,
# velocity, GMST and ground track are wanted by comms (pass prediction,
# antenna pointing), power and thermal (eclipse forecasting), payload
# scheduling, and the magnetic field model. The Sun direction is wanted
# by ADCS and power. Bundling them meant the more widely-used product
# lived inside a component named for the narrower one, and meant the
# magnetic field model took a dependency on solar ephemeris, eclipse
# state and beta angle purely to obtain a position.
#
# ACTIVE, and the only active component in the chain. Two threads would
# otherwise touch the SGP4 elsetrec: the rate group tick and the
# LOAD_TLE command. The message queue serializes them, which removes
# the need for a mutex around propagator state.
#
# THE ONE INVARIANT
# -----------------
# This is the ONLY component in the GNC chain that reads the time port
# for computation. jdUt1, gmstRad and the Fw::Time stamp flow downstream
# inside OrbitState and are read-only there. Downstream components keep
# a time port solely because F Prime requires one to timestamp events
# and telemetry -- they must not use it to derive epochs.
#
# This is not stylistic. Two independent time acquisitions produce two
# epochs that differ, and the moment a component computes its own GMST
# you are back to the failure that quantized Earth rotation to 16.1 deg.
# ======================================================================

module Gnc {
module Environment {

  active component OrbitPropagator {

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    @ Rate group tick. Nominally 1 Hz -- SGP4 in soft-float double is
    @ the expensive part of the subsystem and orbital position does not
    @ need more.
    async input port run: Svc.Sched

    @ Broadcasts the orbit state.
    @
    @ ORDERING: this call is synchronous into whatever it is connected
    @ to, on THIS component's thread. Connect the magnetic field model
    @ and the solar ephemeris here and both evaluate at exactly this
    @ position and epoch -- no interpolation, no skew, no separate time
    @ acquisition.
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
    @ characters. Requires FW_CMD_STRING_MAX_SIZE >= 80 in your config.
    async command LOAD_TLE(
                            line1: string size 80 @< TLE line 1
                            line2: string size 80 @< TLE line 2
                          ) \
      opcode 0x00

    @ Discard the current TLE. State reverts to NO_TLE, but time and
    @ GMST keep flowing so the solar ephemeris stays alive.
    async command CLEAR_TLE \
      opcode 0x01

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    @ TAI - UTC, seconds. Bump when a leap second is announced.
    @ THE ONLY PLACE THIS IS CONFIGURED. Splitting the chain must not
    @ mean two components with their own leap second setting to drift
    @ apart.
    param LEAP_SEC: F64 default 37.0 id 0x00

    @ UT1 - UTC, seconds, from IERS Bulletin A. Always < 0.9 s.
    @ Leaving this at 0 costs at most ~0.004 deg of Earth rotation.
    param DUT1_SEC: F64 default 0.0 id 0x01

    @ TLE age in days beyond which the state is reported STALE.
    param MAX_TLE_AGE_DAYS: F32 default 7.0 id 0x02

    @ Offset added to the Fw::Time seconds field to obtain UTC POSIX
    @ seconds. Use if your time source is GPS time or TAI.
    param UTC_OFFSET_SEC: F64 default 0.0 id 0x03

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    @ Spacecraft position, TEME, kilometres
    telemetry PosTemeKm: Gnc.Vec3d id 0x00

    @ Spacecraft velocity, TEME, kilometres per second
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

    @ Greenwich Mean Sidereal Time, deg. The shared quantity for the
    @ whole subsystem; if anything downstream ever disagrees with this
    @ number, something is recomputing it that should not be.
    telemetry GmstDeg: F32 id 0x07

    @ Wall time consumed by the last tick, microseconds. Covers the
    @ ENTIRE synchronous fan-out, including the WMM evaluation and the
    @ solar model, since both run on this thread.
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

    @ No TLE loaded. Time and GMST still publish, so the solar
    @ ephemeris continues to produce a usable geocentric Sun vector.
    event TleMissing \
      severity warning low \
      id 0x04 \
      format "No TLE loaded, spacecraft state unavailable" \
      throttle 3

    @ Time source reported invalid. NOTHING downstream is usable.
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
