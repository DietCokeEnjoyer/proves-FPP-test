// ======================================================================
// \title  FaultManager.cpp
// \author meeple
// \brief  cpp file for FaultManager component implementation class
// ======================================================================

#include "PROVESFlightControllerReference/Components/FaultManager/FaultManager.hpp"

namespace Components {

// ----------------------------------------------------------------------
// Component construction and destruction
// ----------------------------------------------------------------------

FaultManager ::FaultManager(const char* const compName) : FaultManagerComponentBase(compName) {}

FaultManager ::~FaultManager() {}

// ----------------------------------------------------------------------
// Handler implementations for typed input ports
// ----------------------------------------------------------------------

void FaultManager ::run_handler(FwIndexType portNum, U32 context) {
    // TODO
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void FaultManager ::CLEAR_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

}  // namespace Components
