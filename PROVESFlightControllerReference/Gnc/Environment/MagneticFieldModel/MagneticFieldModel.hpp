/**
 * \file MagneticFieldModel.hpp
 * \brief WMM2025 evaluated at the propagated orbit position.
 */

#ifndef Gnc_Environment_MagneticFieldModel_HPP
#define Gnc_Environment_MagneticFieldModel_HPP

#include "Gnc/Environment/MagneticFieldModel/MagneticFieldModelComponentAc.hpp"
#include "Gnc/AstroLib/AstroLib.hpp"

namespace Gnc {
namespace Environment {

class MagneticFieldModel final : public MagneticFieldModelComponentBase {
  public:
    explicit MagneticFieldModel(const char* const compName);
    ~MagneticFieldModel();

  private:
    /*
     * ============================================================================
     * Handlers for typed input ports
     * ============================================================================
     */

    /**
     * An orbit state arrived. The trigger for a new field
     * evaluation, and the only one on the flight path.
     */
    void orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) override;


    //! Telemetry heartbeat. Does NOT recompute the field.
    void run_handler(FwIndexType portNum, U32 context) override;

    /*
     * ============================================================================
     * Internals
     * ============================================================================
     */

    /**
     * Evaluate WMM2025 at a TEME position and epoch, returning the
     * field back in TEME.
     *
     * \param posTemeKm  position, TEME, KILOMETRES
     * \param jdUt1      UT1 Julian date
     * \param gmstRad    GMST at jdUt1 (from the ephemeris; never
     *                   recomputed here, so there is exactly one Earth
     *                   rotation angle in the system per cycle)
     * \param outNt      field, TEME, nanotesla
     * \param altKmOut   geodetic altitude actually used, for telemetry
     * \return false if the query is outside the model's validated range
     * \param posTemeKm  position, TEME, KILOMETRES
     * \param altKm      geodetic altitude, from OrbitState -- NOT
     *                   recomputed here; the propagator already did it
     * \param jdUt1      UT1 Julian date, from OrbitState
     * \param gmstRad    GMST, from OrbitState; never recomputed, so
     *                   there is one Earth rotation angle per cycle
     * \param outTemeNt  field, TEME, nanotesla
     */
    bool evaluate(const Astro::Vec3& posTemeKm,
                  F64 altKm,
                  F64 jdUt1,
                  F64 gmstRad,
                  Gnc::Vec3f& outTemeNt,
                  F32& decYearOut);

    //! Push the result (or an explicit invalid) on both output ports.
    void emit(const Gnc::Vec3f& fieldTemeNt, bool valid, const Fw::Time& stamp);

    /*
     * ============================================================================
     * State
     *
     * Touched from two threads -- the propagator thread via orbitIn and
     * the rate group thread via run -- which is why every input port on
     * this component is guarded.
     * ============================================================================
     */

    Gnc::Vec3f m_lastFieldTemeNt;
    F32  m_lastDecYear = 0.0f;
    bool m_lastValid = false;
    bool m_everValid = false;
    U32  m_rejectCount = 0;
};

}  // namespace Environment
}  // namespace Gnc

#endif
