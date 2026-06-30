module Components{
    
    @ Fault Types handled by the manager
    enum FaultType{
        NONE = 0
        LOW_BATTERY = 1
        #TODO: Add Fault types
    }

    @
    active component FaultManager{
        # ----------------------------------------------------------------------
        # Input Ports
        # ----------------------------------------------------------------------
        
        @ Port receiving calls from the rate group (1Hz)
        sync input port run: Svc.Sched

        # ----------------------------------------------------------------------
        # Output Ports
        # ----------------------------------------------------------------------
        
        @ Checks the SystemMode
        output port getSystemMode: Components.GetSystemMode

        @ Port to get system voltage from INA219 manager
        output port voltageGet: Drv.VoltageGet

        @ Sets the mode manager to SAFE
        output port setModeSafe: Components.ForceSafeModeWithReason

        @ Sets the mode manager to Normal
        output port setModeNormal: Fw.Signal # Just a trigger, no data passed.

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------

        @ Voltage threshold for safe mode entry (V)
        param SafeModeEntryVoltage: F32 default 6.7

        @ Voltage threshold for safe mode recovery (V)
        param SafeModeRecoveryVoltage: F32 default 8.0

        @ Debounce time for voltage transitions (seconds)
        param SafeModeDebounceSeconds: U32 default 10


        # ----------------------------------------------------------------------
        # Commands
        # ----------------------------------------------------------------------

        @ Manually reset the FaultType to NONE
        async command CLEAR_FAULT()


        # ----------------------------------------------------------------------
        # Telemetry
        # ----------------------------------------------------------------------

        @ The current fault being handled
        telemetry CurrentFault: FaultType

        @ Consecutive seconds the voltage has been below the entry threshold
        telemetry LowVoltageCounter: U32

        @ Consecutive seconds the voltage has been above the recovery threshold
        telemetry RecoveryVoltageCounter: U32

        # ----------------------------------------------------------------------
        # Events
        # ----------------------------------------------------------------------

        event FaultDetected(
            fault: FaultType @< The type of fault triggered.
            faultValue: F32 @< The sensor reading related to the fault. Ex: V, °C
        )\
        severity warning high \
        format "Fault Detected: {} with reading {}. Entering Safe Mode."
        
        event FaultRecovered(
            fault: FaultType @< The type of fault recovered from.
            faultValue: F32 @< The related sensor reading at the time of recovery.
        )\
        severity activity high \
        format "Fault Recovered: {} with reading {}. Entering Normal Mode."


        ###############################################################################
        # Standard AC Ports: Required for Channels, Events, Commands, and Parameters  #
        ###############################################################################
        @ Port for requesting the current time
        time get port timeCaller

        @ Port for sending command registrations
        command reg port cmdRegOut

        @ Port for receiving commands
        command recv port cmdIn

        @ Port for sending command responses
        command resp port cmdResponseOut

        @ Port for sending textual representation of events
        text event port logTextOut

        @ Port for sending events to downlink
        event port logOut

        @ Port for sending telemetry channels to downlink
        telemetry port tlmOut

        @ Port for getting parameter values
        param get port prmGetOut

        @ Port for setting parameter values
        param set port prmSetOut
    }
}