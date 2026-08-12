// ======================================================================
// \title  MagneticFieldModel.cpp
// \brief  WMM2025 evaluated at the propagated orbit position.
//
// THE UNIT AND FRAME BOUNDARY LIVES HERE, IN ONE FUNCTION
// -------------------------------------------------------
// Everything upstream of evaluate() is TEME kilometres in F64, because
// that is what SGP4 produces. Everything inside the geomag call is ITRS
// metres in F32, because that is what XYZgeomag consumes. Everything
// downstream is TEME nanotesla. Those three conversions happen once,
// adjacently, where they can be read together and checked against each
// other. They used to be spread across a port signature, a comment,
// and an implicit assumption.
// ======================================================================

#include "Gnc/Environment/MagneticFieldModel/MagneticFieldModel.hpp"

#include <cmath>

#include "lib/XYZgeomag.hpp"

namespace Gnc {
namespace Environment {

namespace {

//! WMM2025 validated epoch range. The coefficients carry linear secular
//! variation terms fitted over this window; outside it the model is an
//! extrapolation that degrades quickly.
constexpr F32 WMM_MIN_YEAR = 2025.0f;
constexpr F32 WMM_MAX_YEAR = 2030.0f;

//! WMM validated altitude range, km above the WGS-84 ellipsoid. The
//! spherical harmonic sum will happily evaluate outside this; the
//! result just stops meaning anything, because the model omits the
//! external (magnetospheric and ionospheric) field that dominates
//! higher up.
constexpr F32 WMM_MIN_ALT_KM = -1.0f;
constexpr F32 WMM_MAX_ALT_KM = 850.0f;

constexpr F32 TESLA_TO_NT = 1.0e9f;
constexpr F64 KM_TO_M     = 1000.0;

}  // namespace

// ----------------------------------------------------------------------
// Construction
// ----------------------------------------------------------------------

MagneticFieldModel::MagneticFieldModel(const char* const compName)
    : MagneticFieldModelComponentBase(compName),
      m_lastField(0.0f, 0.0f, 0.0f),
      m_lastAltKm(0.0f),
      m_lastDecYear(0.0f),
      m_lastValid(false),
      m_everValid(false),
      m_rejectCount(0) {}

MagneticFieldModel::~MagneticFieldModel() {}

// ----------------------------------------------------------------------
// The model evaluation
// ----------------------------------------------------------------------

bool MagneticFieldModel::evaluate(const Astro::Vec3& posTemeKm,
                                  F64 jdUt1,
                                  F64 gmstRad,
                                  Gnc::MagFieldVec& outNt,
                                  F32& altKmOut,
                                  F32& decYearOut) {
    // --- 1. TEME -> ECEF ------------------------------------------------
    //
    // A single rotation about the 3-axis by GMST. GMST is NOT recomputed
    // here: it arrives from the ephemeris in F64. The old code derived
    // it from an F32 decimal year, which quantized this rotation to
    // 16.1 degrees and would have made the field direction useless
    // for attitude determination while still looking like a plausible
    // 30000 nT vector.
    const Astro::Vec3 ecefKm = Astro::temeToEcef(posTemeKm, gmstRad);

    // --- 2. Range gate --------------------------------------------------
    //
    // Proper WGS-84 geodetic altitude, not the spherical approximation.
    // We already own Bowring's method in AstroLib and the WMM's own
    // altitude limit is defined against the ellipsoid, so using the
    // sphere here would put the gate up to 21 km off at the poles.
    F64 lat = 0.0;
    F64 lon = 0.0;
    F64 altKm = 0.0;
    Astro::ecefToGeodetic(ecefKm, lat, lon, altKm);

    const F64 decYear = Astro::decimalYearFromJd(jdUt1);

    altKmOut   = static_cast<F32>(altKm);
    decYearOut = static_cast<F32>(decYear);

    if (altKmOut < WMM_MIN_ALT_KM || altKmOut > WMM_MAX_ALT_KM ||
        decYearOut < WMM_MIN_YEAR || decYearOut > WMM_MAX_YEAR) {
        this->log_WARNING_LO_OutOfRangeWarning(altKmOut, decYearOut);
        return false;
    }

    // --- 3. Evaluate ----------------------------------------------------
    //
    // XYZgeomag takes ITRS METRES and returns TESLA. SGP4 produced
    // KILOMETRES. This multiply is the entire km/m boundary of the
    // subsystem; if EvalAltKm telemetry ever reads in the hundreds of
    // thousands, this line is the first place to look.
    geomag::Vector p;
    p.x = static_cast<float>(ecefKm.x * KM_TO_M);
    p.y = static_cast<float>(ecefKm.y * KM_TO_M);
    p.z = static_cast<float>(ecefKm.z * KM_TO_M);

    // Narrowing the decimal year to float is safe HERE and only here:
    // it feeds the secular variation terms, where an F32 ULP of 1.22e-4
    // years costs well under a nanotesla.
    const geomag::Vector bEcefTesla = geomag::GeoMag(decYearOut, p, geomag::WMM2025);

    // --- 4. ECEF -> TEME, and Tesla -> nanotesla ------------------------
    //
    // Same GMST, opposite sign. Using the identical angle for both legs
    // is what makes the round trip exact regardless of how good the
    // clock is -- a clock error rotates the field with the spacecraft,
    // so the ATTITUDE solution stays consistent even when the ground
    // track does not.
    const Astro::Vec3 bTesla { static_cast<F64>(bEcefTesla.x),
                               static_cast<F64>(bEcefTesla.y),
                               static_cast<F64>(bEcefTesla.z) };
    const Astro::Vec3 bTeme = Astro::rot3(bTesla, -gmstRad);

    outNt.set_x(static_cast<F32>(bTeme.x) * TESLA_TO_NT);
    outNt.set_y(static_cast<F32>(bTeme.y) * TESLA_TO_NT);
    outNt.set_z(static_cast<F32>(bTeme.z) * TESLA_TO_NT);

    return true;
}

// ----------------------------------------------------------------------
// Ephemeris arrival: the flight path
// ----------------------------------------------------------------------

void MagneticFieldModel::orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) {
    (void)portNum;

    const Gnc::OrbitValidity v = state.get_validity();
    const Fw::Time stamp = state.get_stamp();

    // STALE is accepted deliberately. A week-old TLE is 10-20 km of
    // position error, which at LEO is under 0.2 degrees of magnetic
    // field direction change -- an order of magnitude better than the
    // magnetometer plus residual spacecraft dipole. Refusing to publish
    // would cost far more than the error does.
    const bool posUsable = (v == Gnc::OrbitValidity::VALID) ||
                           (v == Gnc::OrbitValidity::STALE);

    if (!posUsable) {
        this->log_WARNING_LO_OrbitUnusable(v);
        m_lastValid = false;
        m_rejectCount++;
        this->emit(m_lastField, false, stamp);
        return;
    }

    const Gnc::Vector3 p = state.get_posTeme();
    const Astro::Vec3 posTemeKm { p.get_x(), p.get_y(), p.get_z() };

    Gnc::MagFieldVec field;
    F32 altKm = 0.0f;
    F32 decYear = 0.0f;
    const bool ok = this->evaluate(posTemeKm,
                                   state.get_jdUt1(),
                                   state.get_gmstRad(),
                                   field,
                                   altKm,
                                   decYear);

    m_lastAltKm = altKm;
    m_lastDecYear = decYear;

    if (!ok) {
        m_lastValid = false;
        m_rejectCount++;
        this->emit(m_lastField, false, stamp);
        return;
    }

    if (!m_lastValid && m_everValid) {
        this->log_ACTIVITY_HI_FieldRestored();
    }

    m_lastField = field;
    m_lastValid = true;
    m_everValid = true;

    this->emit(field, true, stamp);
}

// ----------------------------------------------------------------------
// Publication
// ----------------------------------------------------------------------

void MagneticFieldModel::emit(const Gnc::MagFieldVec& fieldNt,
                              bool valid,
                              const Fw::Time& stamp) {

    const F32 mag = std::sqrt(fieldNt.get_x() * fieldNt.get_x() +
                              fieldNt.get_y() * fieldNt.get_y() +
                              fieldNt.get_z() * fieldNt.get_z());

    // Direction, for TRIAD. Normalizing at the producer means the
    // magnitude stays visible in telemetry and in fieldOut without
    // every consumer repeating the division.
    if (this->isConnected_magRefOut_OutputPort(0)) {
        Gnc::Vec3f unit;
        const bool usable = valid && (mag > 1.0f);  // 1 nT: far below any real field
        if (usable) {
            unit[0] = fieldNt.get_x() / mag;
            unit[1] = fieldNt.get_y() / mag;
            unit[2] = fieldNt.get_z() / mag;
        }

        Gnc::VectorSample sample;
        sample.set_unitVec(unit);
        sample.set_frame(Gnc::FrameId::TEME);
        // The ORBIT STATE's epoch, not "now". This component never
        // reads the clock, so the attitude chain's staleness check
        // measures the age of the underlying observation.
        sample.set_stamp(stamp);
        sample.set_valid(usable);
        this->magRefOut_out(0, sample);
    }

    // Full vector, for the magnetorquer controller.
    if (this->isConnected_fieldOut_OutputPort(0)) {
        Gnc::MagFieldResult result;
        result.set_field(fieldNt);
        result.set_frame(Gnc::FrameId::TEME);
        result.set_stamp(stamp);
        result.set_valid(valid);
        this->fieldOut_out(0, result);
    }
}

// ----------------------------------------------------------------------
// Off-nominal pull interface
// ----------------------------------------------------------------------

Gnc::MagFieldResult MagneticFieldModel::getField_handler(FwIndexType portNum,
                                                         Gnc::Vector3& posTemeKm,
                                                         F64 jdUt1,
                                                         F64 gmstRad) {
    (void)portNum;

    const Astro::Vec3 pos { posTemeKm.get_x(), posTemeKm.get_y(), posTemeKm.get_z() };

    Gnc::MagFieldVec field;
    F32 altKm = 0.0f;
    F32 decYear = 0.0f;
    const bool ok = this->evaluate(pos, jdUt1, gmstRad, field, altKm, decYear);

    // Deliberately does NOT touch m_lastField or the telemetry state.
    // A ground query about a hypothetical position must not perturb
    // what the flight path is reporting.
    Gnc::MagFieldResult result;
    result.set_field(field);
    result.set_frame(Gnc::FrameId::TEME);
    result.set_stamp(this->getTime());
    result.set_valid(ok);
    return result;
}

// ----------------------------------------------------------------------
// Telemetry heartbeat
// ----------------------------------------------------------------------

void MagneticFieldModel::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;

    // Only report a field once one has actually been computed. The old
    // version telemetered a zero-initialized member from boot, which is
    // indistinguishable on the ground from a genuine model failure.
    if (m_everValid) {
        this->tlmWrite_FieldTeme(m_lastField);

        const F32 mag = std::sqrt(m_lastField.get_x() * m_lastField.get_x() +
                                  m_lastField.get_y() * m_lastField.get_y() +
                                  m_lastField.get_z() * m_lastField.get_z());
        this->tlmWrite_FieldMagnitude(mag);
        this->tlmWrite_EvalAltKm(m_lastAltKm);
        this->tlmWrite_EvalDecYear(m_lastDecYear);
    }

    this->tlmWrite_FieldValid(m_lastValid);
    this->tlmWrite_RejectCount(m_rejectCount);
}

}  // namespace Environment
}  // namespace Gnc
