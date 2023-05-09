within MetroscopeModelingLibrary.Sensors.Power;
model PowerSensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PowerSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PowerIcon;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Utilities.Units.Power W;
                  // Power in W
  Real W_MW(min=0, nominal=100, start=100); // Power in MW

  MetroscopeModelingLibrary.Power.Connectors.Inlet C_in(dummy=0) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
  MetroscopeModelingLibrary.Power.Connectors.Outlet C_out annotation (Placement(transformation(extent={{88,-10},{108,10}}), iconTransformation(extent={{88,-10},{108,10}})));
equation
  // Conservation of power
  C_in.W + C_out.W = 0; // C_out.W < 0 if power flows out of component, as for mass flows

  // Measure
  W = C_in.W;
  W_MW = W/1e6;
end PowerSensor;
