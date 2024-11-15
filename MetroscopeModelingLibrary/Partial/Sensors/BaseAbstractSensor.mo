within MetroscopeModelingLibrary.Partial.Sensors;
model BaseAbstractSensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.AbstractSensorIcon;

  // Icon parameters
  parameter String sensor_function = "Unidentified" "Specify if the sensor is a BC or used for calibration"
    annotation(choices(choice="Unidentified" "No specific function", choice="BC" "Boundary condition", choice="Calibration" "Used for calibration"));
  parameter String causality = "" "Specify which parameter is calibrated by this sensor";

  WaterSteam.Connectors.Inlet C_in annotation (Placement(transformation(extent={{-30,-150},{-10,-130}}), iconTransformation(extent={{-30,-150},{-10,-130}})));
  WaterSteam.Connectors.Outlet C_out annotation (Placement(transformation(extent={{10,-150},{30,-130}}), iconTransformation(extent={{10,-150},{30,-130}})));
  annotation (Icon(
    graphics={
      Rectangle(
        extent={{-100,100},{100,-100}},
        lineColor={0,0,0},
        pattern=LinePattern.None,
        fillColor=if sensor_function == "BC" then {238, 46, 47} elseif sensor_function == "Calibration" then {107, 175, 17} else {255, 255, 255},
        fillPattern=if sensor_function == "BC" or sensor_function == "Calibration" then FillPattern.Solid else FillPattern.None),
      Text(
        extent={{-100,160},{100,120}},
        textColor={85,170,255},
        textString="%name"),
      Text(
        extent={{-100,200},{100,160}},
        textColor={107,175,17},
        textString=if causality <> "" then "%causality" else ""),
      Line(
        points={{100,0},{140,0},{140,180},{100,180}},
        color={107,175,17},
        arrow=if causality == "" then {Arrow.None,Arrow.None} else {Arrow.None,Arrow.Filled},
        thickness=0.5,
        pattern=if causality == "" then LinePattern.None else LinePattern.Solid,
        smooth=Smooth.Bezier)}));

end BaseAbstractSensor;
