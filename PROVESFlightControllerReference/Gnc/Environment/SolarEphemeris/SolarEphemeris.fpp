# ======================================================================
# SolarEphemeris.fpp
#
# Solar direction, eclipse state and beta angle.
#
# PASSIVE, driven by OrbitState. Everything it does that needs an orbit
# -- parallax correction, the dual-cone shadow test, the beta angle --
# needs position AND Sun together, so it sits downstream of the
# propagator rather than beside it.
#
# IT DOES NOT READ THE CLOCK. jdUt1 and the Fw::Time stamp arrive in
# OrbitState. The time port below exists solely because F Prime needs
# one to timestamp events and telemetry. Deriving an epoch from it
# would reintroduce exactly the skew the split was made to prevent.
#
# It still produces a usable Sun vector with NO ORBIT AT ALL: the solar
# model needs only a clock. That is why OrbitPropagator publishes time
# and GMST even on the NO_TLE path -- a safe-mode sun search must not
# depend on having a TLE.
# ======================================================================

module Gnc {
module Environment {

  passive component SolarEphemeris {

    @ Orbit state. Arrival drives one evaluation.
    guarded input port orbitIn: Gnc.OrbitStateSend

    @ Sun direction as a plain vector observation, TEME, for the
    @ attitude chain. Narrow contract on purpose: it lets the TRIAD
    @ component be tested and stubbed without an orbit propagator.
    output port sunRefOut: Gnc.VectorSampleSend

    @ Full solar geometry, for power (array output, eclipse budgeting)
    @ and thermal.
    output port solarOut: Gnc.SolarStateSend

    command recv  port cmdIn
    command reg   port cmdRegOut
    command resp  port cmdResponseOut

    event      port logOut
    text event port logTextOut
    telemetry  port tlmOut
    time get   port timeCaller

    param get port prmGetOut
    param set port prmSetOut

    @ Force a full recomputation on the next tick, bypassing the
    @ interpolation cache. GUARDED, not sync: it writes cache state that
    @ orbitIn also touches, and on a passive component a sync command
    @ would run on the dispatcher thread without the component mutex.
    guarded command RESYNC_SUN \
      opcode 0x00

    @ Length of a solar interpolation segment, seconds. The Sun moves
    @ ~1.1e-5 deg/s, so a 60 s segment costs < 0.0001 deg of SLERP error
    @ while cutting solar model evaluations by ~30x. Set 0 to disable.
    param SUN_SEGMENT_SEC: F64 default 60.0 id 0x00

    @ Unit vector toward the Sun, TEME
    telemetry SunUnitTeme: Gnc.Vec3f id 0x00

    @ Range to the Sun, km
    telemetry SunRangeKm: F64 id 0x01

    @ Sun elevation above the orbit plane, deg
    telemetry BetaDeg: F32 id 0x02

    @ Current illumination state
    telemetry Illumination: Gnc.IlluminationState id 0x03

    @ False when running without an orbit: the Sun vector is geocentric
    @ and illumination / beta are not meaningful.
    telemetry GeometryUsable: bool id 0x04

    @ Diagnostic reason for the current solar state
    telemetry Validity: Gnc.SolarValidity id 0x05

    @ Entered the Earth's shadow
    event EclipseEntry(
                        betaDeg: F32
                      ) \
      severity activity low \
      id 0x00 \
      format "Eclipse entry at beta {f} deg"

    @ Exited the Earth's shadow
    event EclipseExit(
                       betaDeg: F32
                     ) \
      severity activity low \
      id 0x01 \
      format "Eclipse exit at beta {f} deg"

    @ Running without an orbit: Sun direction is geocentric (within
    @ 0.0027 deg in LEO) and shadow / beta are unavailable.
    event GeocentricFallback \
      severity warning low \
      id 0x02 \
      format "No orbit: publishing geocentric Sun vector" \
      throttle 3

    @ Orbit state carried no usable time, so nothing can be computed.
    event TimeMissing \
      severity warning high \
      id 0x03 \
      format "No valid time in orbit state, solar ephemeris suspended" \
      throttle 3

  }

}
}
