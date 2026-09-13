// ======================================================================
// \title  FaultManager.cpp
// \author Autocoder / Elias Dahl
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
    
    // Get current parameters
    Fw::ParamValid paramValid;
    F32 entryVoltage = this->paramGet_SafeModeEntryVoltage(paramValid);
    F32 recoveryVoltage = this->paramGet_SafeModeRecoveryVoltage(paramValid);
    U32 debounceSeconds = this->paramGet_SafeModeDebounceSeconds(paramValid);
    
    //TODO: Test to see if parameter validity needs to be checked, or if default values handle it.

    // Get system mode and voltage
    Components::SystemMode currentMode = this->getSystemMode_out(0);

    bool voltageValid = false;
    F32 voltage = this->getCurrentVoltage(voltageValid);
    
    bool isFault = !voltageValid || (voltage < entryVoltage);

    // In NORMAL, check for faults
    if (currentMode == Components::SystemMode::NORMAL) {
        if (isFault) {
            this->m_lowVoltageCounter++;
            
            if (this->m_lowVoltageCounter >= debounceSeconds) {
                // Trigger Fault!
                this->m_currentFault = FaultType::LOW_BATTERY;
                this->setModeSafe_out(0, Components::SafeModeReason::LOW_BATTERY);
                this->log_WARNING_HI_FaultDetected(this->m_currentFault, voltageValid ? voltage : 0.0f);
                
                this->m_lowVoltageCounter = 0; // Reset after tripping
            }
        } else {
            this->m_lowVoltageCounter = 0; // Voltage is fine, reset counter
        }
    }
    // In SAFE, try to recover
    else if (currentMode == Components::SystemMode::SAFE_MODE && 
             this->m_currentFault == FaultType::LOW_BATTERY) {
        
        if (voltageValid && voltage > recoveryVoltage) {
            this->m_recoveryCounter++;
            
            if (this->m_recoveryCounter >= debounceSeconds) {
                // Recover!
                this->m_currentFault = FaultType::NONE;
                this->setModeNormal_out(0);
                this->log_ACTIVITY_HI_FaultRecovered(this->m_currentFault, voltage);
                
                this->m_recoveryCounter = 0;
            }
        } else {
            this->m_recoveryCounter = 0; // Voltage dropped again, reset counter
        }
    }

    // Downlink Telemetry
    this->tlmWrite_CurrentFault(this->m_currentFault);
    this->tlmWrite_LowVoltageCounter(this->m_lowVoltageCounter);
    this->tlmWrite_RecoveryVoltageCounter(this->m_recoveryCounter);
}

// ----------------------------------------------------------------------
// Handler implementations for commands
// ----------------------------------------------------------------------

void FaultManager ::CLEAR_FAULT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
    // Clear state variables
    this->m_currentFault = FaultType::NONE;
    this->m_lowVoltageCounter = 0;
    this->m_recoveryCounter = 0;
    
    // Update telemetry with cleared values
    this->tlmWrite_CurrentFault(this->m_currentFault);
    this->tlmWrite_LowVoltageCounter(this->m_lowVoltageCounter);
    this->tlmWrite_RecoveryVoltageCounter(this->m_recoveryCounter);

    // Success
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

// ----------------------------------------------------------------------
// Helper Methods
// ----------------------------------------------------------------------

F32 FaultManager ::getCurrentVoltage(bool& valid) {
    // Check if the port is connected and get the current system voltage
    if (this->isConnected_voltageGet_OutputPort(0)) {
        F64 voltage = this->voltageGet_out(0);
        valid = true;
        return static_cast<F32>(voltage);
    }
    
    // Flag invalid readings
    valid = false;
    return 0.0f;
}

}  // namespace Components
