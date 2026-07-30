// ======================================================================
// \title  MagneticFieldModel.cpp
// \author meeple
// \brief  cpp file for MagneticFieldModel component implementation class
// ======================================================================

#include "PROVESFlightControllerReference/Components/MagneticFieldModel/MagneticFieldModel.hpp"

#include <cmath>

#include "lib/XYZgeomag.hpp"

namespace Components {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

MagneticFieldModel ::MagneticFieldModel(const char* const compName) : MagneticFieldModelComponentBase(compName) {}

MagneticFieldModel ::~MagneticFieldModel() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

namespace {}

Components::MagFieldEci MagneticFieldModel ::getField_handler(FwIndexType portNum,
                                                              Components::EciPosition& position,
                                                              F32 decYear) {
    // GMST based ECI -> ECEF rotation 
    F32 gmst = this->computeGmstRad(decYear);   
    F32 cosG = cosf(gmst);
    F32 sinG = sinf(gmst);

    geomag::Vector ecef;
    ecef.x = position.x * cosG + position.y * sinG;
    ecef.y = -position.x * sinG + position.y * cosG;
    ecef.z = position.z;

    // altitude sanity check against WMM's validated range
    F32 radiusKm = sqrtf(ecef.x*ecef.x + ecef.y*ecef.y + ecef.z*ecef.z) / 1000.0f;
    F32 altKm = radiusKm - 6371.2f;  // approx, spherical
    if (altKm < -1.0f || altKm > 850.0f || decYear < 2025.0f || decYear > 2030.0f) {
        this->log_WARNING_LO_OutOfRangeWarning(altKm, decYear);
    }

    // --- evaluate model ---
    geomag::Vector bEcef = geomag::GeoMag(decYear, ecef, geomag::WMM2025);

    // --- rotate field back into ECI ---
    MagFieldEci bEci;
    bEci.x = bEcef.x * cosG - bEcef.y * sinG;
    bEci.y = bEcef.x * sinG + bEcef.y * cosG;
    bEci.z = bEcef.z;

    // convert tesla -> nanotesla for downstream consumers
    bEci.x *= 1.0e9f;
    bEci.y *= 1.0e9f;
    bEci.z *= 1.0e9f;

    m_lastField = bEci;
    return bEci;
}

// Used to periodically downlink field telemetry. Does not compute new field vector.
void MagneticFieldModel ::run_handler(FwIndexType portNum, U32 context) {
    this->tlmWrite_FieldEci(m_lastField);

    F32 mag = sqrtf(m_lastField.x*m_lastField.x
                   + m_lastField.y*m_lastField.y
                   + m_lastField.z*m_lastField.z);

    this->tlmWrite_FieldMagnitude(mag);
}

// Constants and helpers for computeGmstRad
namespace {
    constexpr double PI = 3.14159265358979323846;

    // IAU 1982 GMST expression (Meeus, "Astronomical Algorithms", eq. 12.4)
    // GMST at 0h UT, as a function of centuries since J2000.0 (T),
    // referenced to days since Y2000 epoch (dUT1).
    constexpr double GMST_CONSTANT_TERM_DEG   = 280.46061837;
    constexpr double GMST_LINEAR_TERM_DEG_PER_DAY = 360.98564736629;
    constexpr double GMST_T2_COEFF_DEG        = 0.000387933;
    constexpr double GMST_T3_DIVISOR_DEG      = 38710000.0;

    constexpr double JULIAN_CENTURY_DAYS = 36525.0;
    constexpr double J2000_OFFSET_DAYS   = 0.5;  // Y2000 00:00 UT is 0.5 day before J2000.0 (noon)
    constexpr double DEGREES_PER_CIRCLE  = 360.0;

    bool isLeapYear(I32 year) {
        return (year % 4 == 0) && (year % 100 != 0 || year % 400 == 0);
    }

    // Exact integer day count from 2000-01-01 to Jan 1, 00:00 UT of `year`.
    I32 daysSinceY2000ToYearStart(I32 year) {
        I32 y = year - 2000;
        return y * 365 + (y + 3) / 4 - (y + 99) / 100 + (y + 399) / 400;
    }
}

F32 MagneticFieldModel ::computeGmstRad(F32 decYear) {
    I32 year = static_cast<I32>(decYear);
    F32 fracYear = decYear - static_cast<F32>(year);

    I32 daysInYear = isLeapYear(year) ? 366 : 365;
    F32 dayOfYear = fracYear * static_cast<F32>(daysInYear);  // small, safe in float

    I32 daysToYearStart = daysSinceY2000ToYearStart(year);    // exact, magnitude ~11000

    // Combine in double: this is the step that must not be done in float.
    double daysSinceY2000 = static_cast<double>(daysToYearStart)
                           + static_cast<double>(dayOfYear);

    // Y2000 epoch (2000-01-01 00:00 UT) is 0.5 day before J2000.0 (noon epoch).
    double dUT1 = daysSinceY2000 - J2000_OFFSET_DAYS;
    double T = dUT1 / JULIAN_CENTURY_DAYS;

    double gmstDeg = GMST_CONSTANT_TERM_DEG
                    + GMST_LINEAR_TERM_DEG_PER_DAY * dUT1
                    + GMST_T2_COEFF_DEG * T * T
                    - (T * T * T) / GMST_T3_DIVISOR_DEG;

    gmstDeg = fmod(gmstDeg, DEGREES_PER_CIRCLE);

    return static_cast<F32>(gmstDeg * (PI / 180.0));
}

}  // namespace Components
