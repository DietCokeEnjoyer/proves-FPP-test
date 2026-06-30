// ======================================================================
// \title  FaultManager.hpp
// \author meeple
// \brief  hpp file for FaultManager component implementation class
// ======================================================================

#ifndef Components_FaultManager_HPP
#define Components_FaultManager_HPP

#include "PROVESFlightControllerReference/Components/FaultManager/FaultManagerComponentAc.hpp"

namespace Components {

class FaultManager final : public FaultManagerComponentBase {
  public:
    // ----------------------------------------------------------------------
    // Component construction and destruction
    // ----------------------------------------------------------------------

    //! Construct FaultManager object
    FaultManager(const char* const compName  //!< The component name
    );

    //! Destroy FaultManager object
    ~FaultManager();

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for run
    //!
    //! Port receiving calls from the rate group (1Hz)
    void run_handler(FwIndexType portNum,  //!< The port number
                     U32 context           //!< The call order
                     ) override;

  private:
    // ----------------------------------------------------------------------
    // Handler implementations for commands
    // ----------------------------------------------------------------------

    //! Handler implementation for command CLEAR_FAULT
    //!
    //! Manually reset the FaultType to NONE
    void CLEAR_FAULT_cmdHandler(FwOpcodeType opCode,  //!< The opcode
                                U32 cmdSeq            //!< The command sequence number
                                ) override;
};

}  // namespace Components

#endif
