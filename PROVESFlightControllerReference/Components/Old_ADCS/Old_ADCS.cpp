// ======================================================================
// \title  Old_ADCS.cpp
// \brief  cpp file for Old_ADCS component implementation class
// ======================================================================

#include "PROVESFlightControllerReference/Components/Old_ADCS/Old_ADCS.hpp"

#include <Fw/Types/Assert.hpp>

namespace Components {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

Old_ADCS::Old_ADCS(const char* const compName) : Old_ADCSComponentBase(compName) {}

Old_ADCS::~Old_ADCS() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void Old_ADCS::run_handler(FwIndexType portNum, U32 context) {
    Fw::Success condition;

    // Visible light
    for (FwIndexType i = 0; i < this->getNum_visibleLightGet_OutputPorts(); i++) {
        this->visibleLightGet_out(i, condition);
    }
}

}  // namespace Components
