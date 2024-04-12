within MetroscopeModelingLibrary.Sensors.Power;
model PowerSensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PowerSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PowerIcon;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Utilities.Units.Power W;
                  // Power in W
  Real W_MW(min=0, nominal=100, start=100); // Power in MW

  parameter String display_unit = "MW" "Specify the display unit"
    annotation(choices(choice="MW", choice="W"));
  outer parameter Boolean display_output = false "Used to switch ON or OFF output display";

  MetroscopeModelingLibrary.Power.Connectors.Inlet C_in annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
  MetroscopeModelingLibrary.Power.Connectors.Outlet C_out annotation (Placement(transformation(extent={{88,-10},{108,10}}), iconTransformation(extent={{88,-10},{108,10}})));
equation
  // Conservation of power
  C_in.W + C_out.W = 0; // C_out.W < 0 if power flows out of component, as for mass flows

  // Measure
  W = C_in.W;
  W_MW = W/1e6;

  annotation (Icon(graphics={Text(
      extent={{-100,-160},{102,-200}},
      textColor={0,0,0},
      textString=if display_output then
                 if display_unit == "W" then DynamicSelect("",String(W)+" W")
                 else DynamicSelect("",String(W_MW)+" MW")
                 else "")}));
end PowerSensor;
