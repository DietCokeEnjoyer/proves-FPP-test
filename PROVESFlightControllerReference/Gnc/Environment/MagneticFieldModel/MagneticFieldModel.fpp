# ======================================================================
# MagneticFieldModel.fpp
#
# WMM2025 evaluated at the propagated orbit position, published as a
# reference vector for TRIAD.
#
# WHAT CHANGED AND WHY
# --------------------
# The original component was unreachable. It exposed only a synchronous
# request/response port taking (EciPosition, F32 decYear), and nothing
# in the system produced either argument: the ephemeris publishes an
# an orbit state in TEME km, and nothing anywhere computed a decimal
# year. Its rate-group handler only telemetered whatever the last call
# had left behind, which for a system with no callers was zeros forever.
#
# It is now a CONSUMER of the ephemeris. One input port, one output
# port, and the position and epoch it evaluates at are by construction
# the ones the propagator just produced. Three classes of bug are
# structurally eliminated rather than fixed:
#
#   * Frame skew. It used to be documented as taking J2000 while doing
#     a GMST rotation, which is the TEME->PEF rotation. Feeding it real
#     J2000 would have been ~0.36 deg of precession error in 2026, and
#     the number would have looked entirely plausible.
#   * Time quantization. It derived GMST from an F32 decimal year. At
#     2026 an F32 ULP is 1.22e-4 years, i.e. 1.07 hours, so Earth
#     rotation was quantized to roughly 30 degrees. GMST now arrives in
#     F64 from the ephemeris.
#   * Unit mismatch. XYZgeomag wants ITRS METRES; SGP4 produces KM.
#     The one conversion now lives in this component's .cpp.
#
# Passive, with guarded ports: the evaluation is a degree-12 spherical
# harmonic sum in F32 -- hundreds of native FPU operations on the M33,
# tens of microseconds. That does not justify a thread. But two threads
# do touch this component (the propagator thread via orbitIn, the rate
# group via run), so the latched state needs the component mutex, which
# is exactly what guarded ports provide.
# ======================================================================

module Gnc {
module Environment {

  passive component MagneticFieldModel {

    # ------------------------------------------------------------------
    # Data ports
    # ------------------------------------------------------------------

    @ Orbit state from OrbitPropagator. Arrival triggers an evaluation.
    @
    @ This component uses exactly three fields of it -- posTeme, jdUt1
    @ and gmstRad -- and has no interest in the Sun whatsoever. Before
    @ the propagator was split out it consumed a combined ephemeris
    @ solution, which meant the magnetic model took a dependency on the
    @ solar model, the eclipse state and the beta angle purely to
    @ obtain a position.
    @
    @ It does NOT read the clock. jdUt1 and gmstRad arrive here; the
    @ time port below is for framework event/telemetry timestamping
    @ only. Deriving an epoch from it would reintroduce exactly the
    @ skew this structure prevents.
    guarded input port orbitIn: Gnc.OrbitUpdate

    @ Magnetic field DIRECTION in TEME, for attitude determination.
    @ Normalized here: TRIAD is direction-only and normalizing at the
    @ producer keeps the magnitude available in telemetry without
    @ making every consumer repeat the division.
    output port magRefOut: Gnc.VectorSampleSend

    @ Full field vector with magnitude, TEME, nT. For the magnetorquer
    @ controller, which needs |B| as well as its direction.
    output port fieldOut: Gnc.MagFieldSend

    @ Synchronous request/response for an off-nominal query. Not used
    @ on the flight path; kept for ground checkout and for planning
    @ tools that want B at a hypothetical position.
    guarded input port getField: Gnc.MagFieldRequest

    @ Rate group tick. Telemetry heartbeat only -- it does NOT compute
    @ a new field. Connect it to a slow rate group (0.2-1 Hz); the
    @ field itself updates whenever the ephemeris does.
    guarded input port run: Svc.Sched

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    @ Last computed field vector, TEME, nT
    telemetry FieldTeme: Gnc.MagFieldVec

    @ Field magnitude, nT. In LEO this should sit between roughly
    @ 20000 (equatorial) and 50000 (polar). A number outside that band
    @ means the position, the epoch, or the units are wrong.
    telemetry FieldMagnitude: F32 format "{.1f} nT"

    @ Geodetic altitude the model was last evaluated at, km. The
    @ cheapest single check that the km/m boundary is being crossed
    @ correctly: this should read a few hundred, not a few hundred
    @ thousand and not a fraction.
    telemetry EvalAltKm: F32 format "{.2f} km"

    @ Decimal year the model was last evaluated at
    telemetry EvalDecYear: F32 format "{.4f}"

    @ True if the last evaluation produced a usable field
    telemetry FieldValid: bool

    @ Count of evaluations rejected as out of range
    telemetry RejectCount: U32

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    @ Emitted if a field request is outside the model's validated
    @ altitude or epoch range. Throttled: an orbit that is out of range
    @ is out of range every single cycle, and at 1 Hz that is 86400
    @ identical events per day.
    event OutOfRangeWarning(
                             altitudeKm: F32
                             decYear: F32
                           ) \
      severity warning low \
      format "WMM query outside validated range: alt={f} km, year={f}" \
      throttle 5

    @ The orbit state carries no usable position, so no field can be
    @ computed. Unlike the solar ephemeris, this component has no
    @ position-free fallback: the WMM is a function of location.
    event OrbitUnusable(
                         validity: Gnc.OrbitValidity
                       ) \
      severity warning low \
      format "No field: orbit validity {}" \
      throttle 5

    @ Field evaluation recovered after a dropout
    event FieldRestored \
      severity activity high \
      format "Magnetic field evaluation restored"

    ###############################################################################
    # Standard AC Ports                                                           #
    ###############################################################################

    time get port timeCaller

    command reg port cmdRegOut
    command recv port cmdIn
    command resp port cmdResponseOut

    text event port logTextOut
    event port logOut
    telemetry port tlmOut

    param get port prmGetOut
    param set port prmSetOut

  }

}
}
