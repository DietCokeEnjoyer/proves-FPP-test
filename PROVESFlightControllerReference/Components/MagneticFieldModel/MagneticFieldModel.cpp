// ======================================================================
// \title  MagneticFieldModel.cpp
// \author meeple
// \brief  cpp file for MagneticFieldModel component implementation class
// ======================================================================

#include "PROVESFlightControllerReference/Components/MagneticFieldModel/MagneticFieldModel.hpp"

namespace Components {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

MagneticFieldModel ::MagneticFieldModel(const char* const compName) : MagneticFieldModelComponentBase(compName) {}

MagneticFieldModel ::~MagneticFieldModel() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

Components::MagFieldEci MagneticFieldModel ::getField_handler(FwIndexType portNum,
                                                              Components::EciPosition& position,
                                                              F32 decYear) {
    // TODO return
}

void MagneticFieldModel ::run_handler(FwIndexType portNum, U32 context) {
    // TODO
}

}  // namespace Components
