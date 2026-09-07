/**
 * \file MagneticFieldModel.cpp
 * \brief WMM2025 evaluated at the propagated orbit position.
 *
 * \details THE UNIT AND FRAME BOUNDARY LIVES HERE, IN ONE FUNCTION
 * -------------------------------------------------------
 * Everything upstream of evaluate() is TEME kilometres in F64, because
 * that is what SGP4 produces. Everything inside the geomag call is ITRS
 * metres in F32, because that is what XYZgeomag consumes. Everything
 * downstream is TEME nanotesla. Those three conversions happen once,
 * adjacently, where they can be read together and checked against each
 * other. They used to be spread across a port signature, a comment,
 * and an implicit assumption.
 */

#include "PROVESFlightControllerReference/Gnc/Environment/MagneticFieldModel/MagneticFieldModel.hpp"

#include <cmath>

#include "PROVESFlightControllerReference/Gnc/Types/GncConvert.hpp"

#include "lib/XYZgeomag.hpp"

namespace Gnc {
namespace Environment {

namespace {

/**
 * WMM2025 validated epoch range. The coefficients carry linear secular
 * variation terms fitted over this window; outside it the model is an
 * extrapolation that degrades quickly.
 */
constexpr F32 WMM_MIN_YEAR = 2025.0f;
constexpr F32 WMM_MAX_YEAR = 2030.0f;

/**
 * WMM validated altitude range, km above the WGS-84 ellipsoid. The
 * spherical harmonic sum will happily evaluate outside this; the
 * result just stops meaning anything, because the model omits the
 * external (magnetospheric and ionospheric) field that dominates
 * higher up.
 */
constexpr F32 WMM_MIN_ALT_KM = -1.0f;
constexpr F32 WMM_MAX_ALT_KM = 850.0f;

constexpr F32 TESLA_TO_NT = 1.0e9f;
constexpr F64 KM_TO_M     = 1000.0;

}  // namespace

/*
 * ============================================================================
 * Construction
 * ============================================================================
 */

MagneticFieldModel::MagneticFieldModel(const char* const compName)
    : MagneticFieldModelComponentBase(compName),
      m_lastFieldTemeNtTemeNt(0.0f, 0.0f, 0.0f) {}

MagneticFieldModel::~MagneticFieldModel() {}

/*
 * ============================================================================
 * The model evaluation
 * ============================================================================
 */

bool MagneticFieldModel::evaluate(const Astro::Vec3& posTemeKm,
                                  F64 altKm,
                                  F64 jdUt1,
                                  F64 gmstRad,
                                  Gnc::Vec3f& outTemeNt,
                                  F32& decYearOut) {
    /*
     * --- 1. TEME -> ECEF ------------------------------------------------
     *
     * A single rotation about the 3-axis by GMST. GMST is NOT recomputed
     * here: it arrives from the ephemeris in F64. The old code derived
     * it from an F32 decimal year, which quantized this rotation to
     * 16.1 degrees and would have made the field direction useless
     * for attitude determination while still looking like a plausible
     * 30000 nT vector.
     */
    const Astro::Vec3 ecefKm = Astro::temeToEcef(posTemeKm, gmstRad);

    /*
     * --- 2. Range gate --------------------------------------------------
     *
     * WGS-84 geodetic altitude arrives in OrbitState. It is NOT
     * recomputed here: the propagator ran Bowring's method on this exact
     * position one call earlier, and duplicating several F64
     * transcendentals per cycle on a soft-float M33 buys nothing.
     */
    const F64 decYear = Astro::decimalYearFromJd(jdUt1);
    decYearOut = static_cast<F32>(decYear);

    const F32 altKmF = static_cast<F32>(altKm);
    if (altKmF < WMM_MIN_ALT_KM || altKmF > WMM_MAX_ALT_KM ||
        decYearOut < WMM_MIN_YEAR || decYearOut > WMM_MAX_YEAR) {
        this->log_WARNING_LO_EvaluationOutOfRange(altKmF, decYearOut);
        return false;
    }

    /*
     * --- 3. Evaluate ----------------------------------------------------
     *
     * XYZgeomag takes ITRS METRES and returns TESLA. SGP4 produced
     * KILOMETRES. This multiply is the entire km/m boundary of the
     * subsystem; if OrbitPropagator.AltKm ever reads in the hundreds of
     * thousands, this line is the first place to look.
     */
    geomag::Vector p;
    p.x = static_cast<float>(ecefKm.x * KM_TO_M);
    p.y = static_cast<float>(ecefKm.y * KM_TO_M);
    p.z = static_cast<float>(ecefKm.z * KM_TO_M);

    /*
     * Narrowing the decimal year to float is safe HERE and only here:
     * it feeds the secular variation terms, where an F32 ULP of 1.22e-4
     * years costs well under a nanotesla.
     */
    const geomag::Vector bEcefTesla = geomag::GeoMag(decYearOut, p, geomag::WMM2025);

    /*
     * --- 4. ECEF -> TEME, and Tesla -> nanotesla ------------------------
     *
     * Same GMST, opposite sign. Using the identical angle for both legs
     * is what makes the round trip exact regardless of how good the
     * clock is -- a clock error rotates the field with the spacecraft,
     * so the ATTITUDE solution stays consistent even when the ground
     * track does not.
     */
    const Astro::Vec3 bTesla { static_cast<F64>(bEcefTesla.x),
                               static_cast<F64>(bEcefTesla.y),
                               static_cast<F64>(bEcefTesla.z) };
    const Astro::Vec3 bTeme = Astro::rot3(bTesla, -gmstRad);

    outTemeNt.set_x(static_cast<F32>(bTeme.x) * TESLA_TO_NT);
    outTemeNt.set_y(static_cast<F32>(bTeme.y) * TESLA_TO_NT);
    outTemeNt.set_z(static_cast<F32>(bTeme.z) * TESLA_TO_NT);

    return true;
}

/*
 * ============================================================================
 * Ephemeris arrival: the flight path
 * ============================================================================
 */

void MagneticFieldModel::orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) {
    (void)portNum;

    const Fw::Time stamp = state.get_stamp();

    /*
     * The producer decides whether position is usable, including the
     * policy that a STALE TLE still counts -- a week-old TLE is 10-20 km
     * of position error, under 0.2 degrees of field direction change at
     * LEO, an order of magnitude better than the magnetometer plus
     * residual spacecraft dipole. That test used to be spelled out here
     * AND in the solar ephemeris, so the two could have diverged.
     */
    if (!state.get_positionUsable()) {
        this->log_WARNING_LO_OrbitUnusable(state.get_validity());
        m_lastValid = false;
        m_rejectCount++;
        this->emit(m_lastFieldTemeNt, false, stamp);
        return;
    }

    const Astro::Vec3 posTemeKm = toAstro(state.get_posTemeKm());

    Gnc::Vec3f fieldTemeNt;
    F32 decYear = 0.0f;
    const bool ok = this->evaluate(posTemeKm,
                                   state.get_altKm(),
                                   state.get_jdUt1(),
                                   state.get_gmstRad(),
                                   fieldTemeNt,
                                   decYear);

    m_lastDecYear = decYear;

    if (!ok) {
        m_lastValid = false;
        m_rejectCount++;
        this->emit(m_lastFieldTemeNt, false, stamp);
        return;
    }

    if (!m_lastValid && m_everValid) {
        this->log_ACTIVITY_HI_FieldRestored();
    }

    m_lastFieldTemeNt = fieldTemeNt;
    m_lastValid = true;
    m_everValid = true;

    this->emit(fieldTemeNt, true, stamp);
}

/*
 * ============================================================================
 * Publication
 * ============================================================================
 */

void MagneticFieldModel::emit(const Gnc::Vec3f& fieldTemeNt,
                              bool valid,
                              const Fw::Time& stamp) {
    const F32 magNt = std::sqrt(fieldTemeNt.get_x() * fieldTemeNt.get_x() +
                                fieldTemeNt.get_y() * fieldTemeNt.get_y() +
                                fieldTemeNt.get_z() * fieldTemeNt.get_z());

    /*
     * 1 nT is far below any real geomagnetic field, so this only trips
     * on a genuinely degenerate result.
     */
    const bool usable = valid && (magNt > 1.0f);

    // Direction, for TRIAD.
    if (this->isConnected_magRefOut_OutputPort(0)) {
        Gnc::Vec3f unit;
        if (usable) {
            unit.set_x(fieldTemeNt.get_x() / magNt);
            unit.set_y(fieldTemeNt.get_y() / magNt);
            unit.set_z(fieldTemeNt.get_z() / magNt);
        }

        Gnc::VectorSample sample;
        sample.set_vec(unit);
        sample.set_frame(Gnc::FrameId::TEME);
        /*
         * The ORBIT STATE's epoch, not "now". This component never reads
         * the clock, so the attitude chain's staleness check measures
         * the age of the underlying observation.
         */
        sample.set_stamp(stamp);
        sample.set_valid(usable);
        this->magRefOut_out(0, sample);
    }

    /*
     * Full vector in nanotesla, for the magnetorquer controller, which
     * needs |B| and not just its direction.
     *
     * Same port type as magRefOut. VectorSample does not require a unit
     * vector -- see its doc comment -- so there is no reason for a
     * near-identical second struct to exist just to carry a magnitude.
     */
    if (this->isConnected_fieldOut_OutputPort(0)) {
        Gnc::VectorSample sample;
        sample.set_vec(fieldTemeNt);
        sample.set_frame(Gnc::FrameId::TEME);
        sample.set_stamp(stamp);
        sample.set_valid(valid);
        this->fieldOut_out(0, sample);
    }
}

/*
 * ============================================================================
 * Telemetry heartbeat
 * ============================================================================
 */

void MagneticFieldModel::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;

    /*
     * Only report a field once one has actually been computed. The old
     * version telemetered a zero-initialized member from boot, which is
     * indistinguishable on the ground from a genuine model failure.
     */
    if (m_everValid) {
        this->tlmWrite_FieldTemeNt(m_lastFieldTemeNt);

        const F32 magNt = std::sqrt(m_lastFieldTemeNt.get_x() * m_lastFieldTemeNt.get_x() +
                                    m_lastFieldTemeNt.get_y() * m_lastFieldTemeNt.get_y() +
                                    m_lastFieldTemeNt.get_z() * m_lastFieldTemeNt.get_z());
        this->tlmWrite_FieldMagnitudeNt(magNt);
        this->tlmWrite_EvalDecYear(m_lastDecYear);
    }

    /*
     * Evaluation altitude is not telemetered here. It is
     * OrbitPropagator.AltKm, computed once and shared -- see OrbitState.
     */
    this->tlmWrite_FieldValid(m_lastValid);
    this->tlmWrite_FieldRejectCount(m_rejectCount);
}

}  // namespace Environment
}  // namespace Gnc
