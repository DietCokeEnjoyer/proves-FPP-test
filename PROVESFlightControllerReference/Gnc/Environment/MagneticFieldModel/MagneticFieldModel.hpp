/**
 * \file MagneticFieldModel.hpp
 * \brief WMM2025 evaluated at the propagated orbit position.
 */

#ifndef Gnc_Environment_MagneticFieldModel_HPP
#define Gnc_Environment_MagneticFieldModel_HPP

#include "PROVESFlightControllerReference/Gnc/Environment/MagneticFieldModel/MagneticFieldModelComponentAc.hpp"
#include "PROVESFlightControllerReference/Gnc/Environment/MagneticFieldModel/WmmModel.hpp"

namespace Gnc {
namespace Environment {

/**
 * \brief Framework wrapper around the WMM field model.
 *
 * \details Supplies the magnetic reference vector for TRIAD and the full
 * field vector for the magnetorquer controller. Consumes position,
 * altitude, time and GMST from OrbitState, doesn't recompute them.
 *
 * This file is the FRAMEWORK layer. It owns the orbit gate, caching of
 * the last good field, events, telemetry and fault reporting. It
 * contains no field math -- that all lives in WmmModel.cpp, which
 * knows nothing about F Prime and can be unit tested on a workstation.
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
     * \details Gated on the OrbitState's positionUsable flag, then
     * handed to evaluateField(). Both failure paths republish the last
     * good field with valid = false rather than going silent.
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
     * \brief Push the result (or an explicit invalid) on both output ports.
     *
     * \details magRefOut carries the unit vector for TRIAD; fieldOut
     * carries the full vector in nanotesla for the magnetorquer
     * controller, which needs the magnitude. Both are VectorSample; that
     * type does not require a unit vector, so no near-identical second
     * struct exists just to carry a magnitude.
     *
     * On the invalid path magRefOut still publishes, carrying a zero
     * vector flagged invalid, so a stalled producer and a rejected
     * evaluation do not look the same downstream.
     *
     * \param sol    Field to publish. On the invalid path this is the
     *               last good solution.
     * \param valid  Whether this cycle produced a fresh evaluation
     * \param stamp  Epoch of the orbit state.
     */
    void emit(const FieldSolution& sol, bool valid, const Fw::Time& stamp);

    /*
     * ============================================================================
     * State
     *
     * Touched by two threads: the propagator thread via orbitIn and
     * the rate group thread via run. The component's input ports are guarded because of this.
     * ============================================================================
     */

    FieldSolution m_lastField;  //!< Most recent successful evaluation
    F32 m_lastDecYear = 0.0f;   //!< Decimal year of the most recent evaluation, successful or not
    bool m_lastValid = false;   //!< Whether the most recent evaluation succeeded
    bool m_everValid = false;   //!< Whether any evaluation has ever succeeded
    U32 m_rejectCount = 0;      //!< Cumulative rejected evaluations, any reason
};

}  // namespace Environment
}  // namespace Gnc

#endif
