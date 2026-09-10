/**
 * \file MagneticFieldModel.cpp
 * \brief WMM2025 evaluated at the propagated orbit position.
 *
 * \details
 * Evaluation steps:
 * 1. Checks the altitude and time from OrbitState to ensure they're within WMM's valid range.
 * 2. The propogated position's frame is rotated from TEME to ECEF.
 * 3. The position is converted from kilometers in F64 to meters in F32 for XYZGeomag.
 * 4. XYZGeomag is called with the converted position and returns the magnetic field estimation in tesla, ECEF.
 * 5. The field estimation is rotated from ECEF to TEME
 * 6. The field is converted into nanotesla
 * 7. The full field measurement and a normalized unit vector are broadcast for consumers. 
 */

#include "PROVESFlightControllerReference/Gnc/Environment/MagneticFieldModel/MagneticFieldModel.hpp"

#include <cmath>

#include "PROVESFlightControllerReference/Gnc/Types/GncConvert.hpp"

#include "lib/XYZgeomag.hpp"

namespace Gnc {
namespace Environment {

namespace {

/*
 * WMM2025 validated epoch range. The coefficients carry linear secular
 * variation terms fitted over this window, outside it the model is an
 * extrapolation that degrades quickly.
 */
constexpr F32 WMM_MIN_YEAR = 2025.0f;
constexpr F32 WMM_MAX_YEAR = 2030.0f;

/*
 * WMM validated altitude range, km above the WGS-84 ellipsoid.
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

/**
 * \brief Construct the component with a zeroed, invalid field.
 * \param compName  F Prime component instance name
 */
MagneticFieldModel::MagneticFieldModel(const char* const compName)
    : MagneticFieldModelComponentBase(compName),
      m_lastFieldTemeNt(0.0f, 0.0f, 0.0f) {}

//! Destroy the component. Holds no resources.
MagneticFieldModel::~MagneticFieldModel() {}

/*
 * ============================================================================
 * The model evaluation
 * ============================================================================
 */

/**
 * \brief Evaluate WMM2025 and return the field in TEME nanotesla.
 *
 * \details Four steps:
 * 1: Gate on the model's validated altitude and epoch range 
 * 2: Rotate TEME to ECEF(ITRS), 
 * 3: Call the geomag library in ITRS meters
 * 4: Rotate back and scale to nanotesla.
 *
 * \param posTemeKm  Position, TEME, km
 * \param altKm      Geodetic altitude, km, from OrbitState
 * \param jdUt1      UT1 Julian date, from OrbitState
 * \param gmstRad    GMST, from OrbitState
 * \param outTemeNt  [out] Field, TEME, nanotesla
 * \param decYearOut [out] Decimal year evaluated at, written before the
 *                   range check so it is usable in telemetry either way
 * \return true on success; false after WARNING_LO EvaluationOutOfRange
 */
bool MagneticFieldModel::evaluate(const Astro::Vec3& posTemeKm,
                                  F64 altKm,
                                  F64 jdUt1,
                                  F64 gmstRad,
                                  Gnc::Vec3f& outTemeNt,
                                  F32& decYearOut) {
    /*
     * ----------------------------------------------------------------------------
     * Step 1: Range gate
     * ----------------------------------------------------------------------------
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
     * ----------------------------------------------------------------------------
     * Step 2: TEME -> ECEF
     * A rotation about the 3-axis by GMST.
     * ----------------------------------------------------------------------------
     */
    const Astro::Vec3 ecefKm = Astro::temeToEcef(posTemeKm, gmstRad);

    /*
     * ----------------------------------------------------------------------------
     * Step 3: Evaluate
     * ----------------------------------------------------------------------------
     */

     // Convert from kilometers to meters for geomag call
    geomag::Vector p;
    p.x = static_cast<float>(ecefKm.x * KM_TO_M);
    p.y = static_cast<float>(ecefKm.y * KM_TO_M);
    p.z = static_cast<float>(ecefKm.z * KM_TO_M);

    const geomag::Vector bEcefTesla = geomag::GeoMag(decYearOut, p, geomag::WMM2025);

    /*
     * ----------------------------------------------------------------------------
     * Step 4: ECEF -> TEME, Tesla -> nanotesla
     * ----------------------------------------------------------------------------
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

/**
 * \brief Orbit state arrived. Evaluate the field once.
 *
 * \details Both failure paths republish the last good field with
 * valid = false rather than going silent. FieldRestored triggers on
 * the recovery edge only.
 *
 * \param portNum  Port index, unused
 * \param state    Orbit state from OrbitPropagator
 */
void MagneticFieldModel::orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) {
    (void)portNum;

    const Fw::Time stamp = state.get_stamp();

    // Check if the position is valid
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

/**
 * \brief Publish the direction on magRefOut and the full vector on fieldOut.
 *
 * \param fieldTemeNt  Field, TEME, nanotesla
 * \param valid        Whether this cycle produced a fresh evaluation
 * \param stamp        Epoch of the orbit state
 */
void MagneticFieldModel::emit(const Gnc::Vec3f& fieldTemeNt,
                              bool valid,
                              const Fw::Time& stamp) {
    const F32 magNt = std::sqrt(fieldTemeNt.get_x() * fieldTemeNt.get_x() +
                                fieldTemeNt.get_y() * fieldTemeNt.get_y() +
                                fieldTemeNt.get_z() * fieldTemeNt.get_z());

    /*
     * 1 nT is much smaller than any real geomagnetic field, so this only trips
     * on a degenerate result.
     */
    const bool usable = valid && (magNt > 1.0f);

    // Unit vector for TRIAD.
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

        sample.set_stamp(stamp); // Orbit state's epoch
        sample.set_valid(usable);
        this->magRefOut_out(0, sample);
    }

    // Full vector in nanotesla for the magnetorquer controller
    if (this->isConnected_fieldOut_OutputPort(0)) {
        Gnc::VectorSample sample;
        sample.set_vec(fieldTemeNt);
        sample.set_frame(Gnc::FrameId::TEME);
        sample.set_stamp(stamp);
        sample.set_valid(valid);
        this->fieldOut_out(0, sample);
    }
}

/**
 * \brief Telemetry heartbeat. Reports cached state.
 *
 * \param portNum  Port index, unused
 * \param context  Rate group context, unused
 */
void MagneticFieldModel::run_handler(FwIndexType portNum, U32 context) {
    (void)portNum;
    (void)context;

    // Only reports after the first field computation.
    if (m_everValid) {
        this->tlmWrite_FieldTemeNt(m_lastFieldTemeNt);

        const F32 magNt = std::sqrt(m_lastFieldTemeNt.get_x() * m_lastFieldTemeNt.get_x() +
                                    m_lastFieldTemeNt.get_y() * m_lastFieldTemeNt.get_y() +
                                    m_lastFieldTemeNt.get_z() * m_lastFieldTemeNt.get_z());
        this->tlmWrite_FieldMagnitudeNt(magNt);
        this->tlmWrite_EvalDecYear(m_lastDecYear);
    }

    this->tlmWrite_FieldValid(m_lastValid);
    this->tlmWrite_FieldRejectCount(m_rejectCount);
}

}  // namespace Environment
}  // namespace Gnc
