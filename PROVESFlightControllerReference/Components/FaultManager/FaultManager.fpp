module components{
    
    @ Recovery Manager States
    enum RecState {

    }

    @ rec manager
    active component RecoveryManager{
        # ----------------------------------------------------------------------
        # Input Ports
        # ----------------------------------------------------------------------
        
        @ Port receiving calls from the rate group (1Hz)
        sync input port run: Svc.Sched

        @ Checks the SystemMode
        sync input port getSystemMode: Components.GetSystemMode


        # ----------------------------------------------------------------------
        # Output Ports
        # ----------------------------------------------------------------------

        @ Port to get system voltage from INA219 manager
        output port voltageGet: Drv.VoltageGet

        @ Sets the mode manager to SAFE
        output port setModeSafe: Components.ForceSafeModeWithReason

        @ Sets the mode manager to Normal
        output port setModeNormal: 

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------

        @ Voltage threshold for safe mode entry (V)
        param SafeModeEntryVoltage: F32 default 6.7

        @ Voltage threshold for safe mode recovery (V)
        param SafeModeRecoveryVoltage: F32 default 8.0

        @ Debounce time for voltage transitions (seconds)
        param SafeModeDebounceSeconds: U32 default 10


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