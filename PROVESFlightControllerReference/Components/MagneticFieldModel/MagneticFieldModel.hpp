// ======================================================================
// \title  MagneticFieldModel.hpp
// \author meeple
// \brief  hpp file for MagneticFieldModel component implementation class
// ======================================================================

#ifndef Components_MagneticFieldModel_HPP
#define Components_MagneticFieldModel_HPP

#include "PROVESFlightControllerReference/Components/MagneticFieldModel/MagneticFieldModelComponentAc.hpp"

namespace Components {

class MagneticFieldModel final : public MagneticFieldModelComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct MagneticFieldModel object
    MagneticFieldModel(const char* const compName  //!< The component name
    );

    //! Destroy MagneticFieldModel object
    ~MagneticFieldModel();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for getField
    //!
    //! Synchronous request/response port for magnetic field queries
    Components::MagFieldEci getField_handler(FwIndexType portNum,  //!< The port number
                                             Components::EciPosition& position,
                                             F32 decYear  //!< decimal year, ex: 2027.44
                                             ) override;

    //! Handler implementation for run
    //!
    //! Port receiving calls from the rate group
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;
};

}  // namespace Components

#endif
