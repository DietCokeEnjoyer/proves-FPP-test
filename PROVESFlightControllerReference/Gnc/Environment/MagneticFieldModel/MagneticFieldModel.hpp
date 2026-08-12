// ======================================================================
// \title  MagneticFieldModel.hpp
// \brief  WMM2025 evaluated at the propagated orbit position.
// ======================================================================

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
    // ----------------------------------------------------------------------
    // Handlers for typed input ports
    // ----------------------------------------------------------------------

    //! An orbit state arrived. The trigger for a new field
    //! evaluation, and the only one on the flight path.
    void orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) override;

    //! Off-nominal pull interface: field at an arbitrary position/epoch.
    Gnc::MagFieldResult getField_handler(FwIndexType portNum,
                                         Gnc::Vector3& posTemeKm,
                                         F64 jdUt1,
                                         F64 gmstRad) override;

    //! Telemetry heartbeat. Does NOT recompute the field.
    void run_handler(FwIndexType portNum, U32 context) override;

    // ----------------------------------------------------------------------
    // Internals
    // ----------------------------------------------------------------------

    //! Evaluate WMM2025 at a TEME position and epoch, returning the
    //! field back in TEME.
    //!
    //! \param posTemeKm  position, TEME, KILOMETRES
    //! \param jdUt1      UT1 Julian date
    //! \param gmstRad    GMST at jdUt1 (from the ephemeris; never
    //!                   recomputed here, so there is exactly one Earth
    //!                   rotation angle in the system per cycle)
    //! \param outNt      field, TEME, nanotesla
    //! \param altKmOut   geodetic altitude actually used, for telemetry
    //! \return false if the query is outside the model's validated range
    bool evaluate(const Astro::Vec3& posTemeKm,
                  F64 jdUt1,
                  F64 gmstRad,
                  Gnc::MagFieldVec& outNt,
                  F32& altKmOut,
                  F32& decYearOut);

    //! Push the result (or an explicit invalid) on both output ports.
    void emit(const Gnc::MagFieldVec& fieldNt, bool valid, const Fw::Time& stamp);

    // ----------------------------------------------------------------------
    // State
    //
    // Touched from two threads -- the propagator thread via orbitIn and
    // the rate group thread via run -- which is why every input port on
    // this component is guarded.
    // ----------------------------------------------------------------------

    Gnc::MagFieldVec m_lastField;
    F32  m_lastAltKm;
    F32  m_lastDecYear;
    bool m_lastValid;
    bool m_everValid;
    U32  m_rejectCount;
};

}  // namespace Environment
}  // namespace Gnc

#endif
