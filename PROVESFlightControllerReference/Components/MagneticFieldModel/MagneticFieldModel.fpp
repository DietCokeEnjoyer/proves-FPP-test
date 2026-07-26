module Components {

  @ Cartesian vector, meters, ECI frame (J2000)
  struct EciPosition {
    x: F32
    y: F32
    z: F32
  }

  @ Magnetic field vector in nanotesla, ECI frame
  struct MagFieldEci {
    x: F32
    y: F32
    z: F32
  }

  @ NED elements
  struct MagFieldElements {
    north: F32       @< nT
    east: F32        @< nT
    down: F32        @< nT
    declination: F32 @< deg
    inclination: F32 @< deg
    totalField: F32  @< nT
  }

  @ Port: request the field at a given ECI position and time
  port MagFieldRequest(
    ref position: EciPosition
    decYear: F32              @< decimal year, ex: 2027.44
  ) -> MagFieldEci

  passive component MagneticFieldModel {

    ######################################################################
    # Ports
    ######################################################################
    @ Port receiving calls from the rate group
    sync input port run: Svc.Sched

    @ Synchronous request/response port for magnetic field queries
    sync input port getField: MagFieldRequest
    
    ######################################################################
    # Telemetry
    ######################################################################
    @ Last computed field vector
    telemetry FieldEci: MagFieldEci

    @ Field magnitude
    telemetry FieldMagnitude: F32

    ######################################################################
    # Events
    ######################################################################

    @ Emittied if a field request is outside the model's altitude/epoch range
    event OutOfRangeWarning(
      altitudeKm: F32
      decYear: F32
    ) severity warning low format \
      "Magnetic field query outside validated range: alt={f}km, year={f}"

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

    @ Port to return the value of a parameter
    param get port prmGetOut

    @Port to set the value of a parameter
    param set port prmSetOut

  }

}