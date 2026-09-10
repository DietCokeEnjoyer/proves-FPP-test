/**
 * \file MagneticFieldModel.hpp
 * \brief WMM2025 evaluated at the propagated orbit position.
 */

#ifndef Gnc_Environment_MagneticFieldModel_HPP
#define Gnc_Environment_MagneticFieldModel_HPP

#include "PROVESFlightControllerReference/Gnc/Environment/MagneticFieldModel/MagneticFieldModelComponentAc.hpp"
#include "PROVESFlightControllerReference/Gnc/AstroLib/AstroLib.hpp"

namespace Gnc {
namespace Environment {

/**
 * \brief WMM2025 evaluated at the propagated orbit position.
 *
 * \details Supplies the magnetic reference vector for TRIAD and the full
 * field vector for the magnetorquer controller. Consumes position,
 * altitude, time and GMST from OrbitState, doesn't recompute them.
 */
class MagneticFieldModel final : public MagneticFieldModelComponentBase {
  public:
    /**
     * \brief Construct the component with a zeroed, invalid field.
     * \param compName  F Prime component instance name
     */
    explicit MagneticFieldModel(const char* const compName);

    //! Destroy the component. Holds no resources.
    ~MagneticFieldModel();

  private:
    /*
     * ============================================================================
     * Handlers for typed input ports
     * ============================================================================
     */

    /**
     * \brief An orbit state arrived. The trigger for a new field
     *        evaluation.
     *
     * \details Gated on the OrbitState's positionUsable flag.
     *
     * \param portNum  Port index, unused (single port)
     * \param state    Orbit state from OrbitPropagator
     */
    void orbitIn_handler(FwIndexType portNum, Gnc::OrbitState& state) override;


    /**
     * \brief Telemetry heartbeat. Does NOT recompute the field.
     *
     * \details Runs on the rate group while evaluation runs on the
     * propagator thread, which is why every input port on this component
     * is guarded. Reports nothing until at least one field has been
     * computed.
     *
     * \param portNum  Port index, unused (single port)
     * \param context  Rate group context, unused
     */
    void run_handler(FwIndexType portNum, U32 context) override;

    /*
     * ============================================================================
     * Internals
     * ============================================================================
     */

    /**
     * \brief Evaluate WMM2025 at a TEME position and epoch, returning
     *        the field back in TEME.
     *
     * \details The unit and frame boundary of the subsystem. TEME
     * kilometers in F64 go in, ITRS meters in F32 go to the geomag
     * library, and TEME nanotesla come back out.
     *
     * Rejects out-of-range altitude and epoch values.
     *
     * \param posTemeKm  Position, TEME, Kilometers
     * \param altKm      Geodetic altitude above the WGS-84 ellipsoid,
     *                   km, from OrbitState
     * \param jdUt1      UT1 Julian date, from OrbitState
     * \param gmstRad    GMST, from OrbitState;
     * \param outTemeNt  [out] Field, TEME, nanotesla. Untouched on failure.
     * \param decYearOut [out] Decimal year the model was evaluated at.
     *                   Written before the range check, so it is valid
     *                   for telemetry even when the call fails.
     * 
     * \return false if altitude or epoch is outside the model's validated range;
     *         true otherwise
     */
    bool evaluate(const Astro::Vec3& posTemeKm,
                  F64 altKm,
                  F64 jdUt1,
                  F64 gmstRad,
                  Gnc::Vec3f& outTemeNt,
                  F32& decYearOut);

    /**
     * \brief Push the result (or an explicit invalid) on both output ports.
     *
     * \details magRefOut carries the unit vector for TRIAD; fieldOut
     * carries the full vector in nanotesla for the magnetorquer
     * controller, which needs the magnitude. Both are VectorSample; that
     * type does not require a unit vector, so no near-identical second
     * struct exists just to carry a magnitude.
     *
     * \param fieldTemeNt  Field, TEME, nanotesla. On the invalid path
     *                     this is the last good value.
     * \param valid        Whether this cycle produced a fresh evaluation
     * \param stamp        Epoch of the orbit state.
     */
    void emit(const Gnc::Vec3f& fieldTemeNt, bool valid, const Fw::Time& stamp);

    /*
     * ============================================================================
     * State
     *
     * Touched by two threads: the propagator thread via orbitIn and
     * the rate group thread via run. The component's input ports are guarded because of this.
     * ============================================================================
     */

    Gnc::Vec3f m_lastFieldTemeNt;  //!< Most recent successful field, TEME nT
    F32  m_lastDecYear = 0.0f;     //!< Decimal year of the most recent evaluation
    bool m_lastValid = false;      //!< Whether the most recent evaluation succeeded
    bool m_everValid = false;      //!< Whether any evaluation has ever succeeded
    U32  m_rejectCount = 0;        //!< Cumulative rejected evaluations, any reason
};

}  // namespace Environment
}  // namespace Gnc

#endif
