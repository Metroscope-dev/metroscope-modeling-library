within MetroscopeModelingLibrary.Utilities.Icons.Sensors;
partial record FuelSensorIcon "should be extended in partial base classes"
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  parameter String sensor_function = "Unidentified" "Specify if the sensor is a BC or used for calibration"
    annotation(choices(choice="Unidentified" "No specific function", choice="BC" "Boundary condition", choice="Calibration" "Used for calibration"));
  parameter String causality = "" "Specify which parameter is calibrated by this sensor";
  outer parameter Boolean show_causality = true "Used to switch show or not the causality";

  annotation (Icon(
      graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor=if sensor_function == "BC" then {238, 46, 47} elseif sensor_function == "Calibration" then {107, 175, 17} else {255, 255, 255},
          fillPattern=if sensor_function == "BC" or sensor_function == "Calibration" then FillPattern.Solid else FillPattern.None),
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-80,80},{80,-80}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Text(
          extent={{-100,160},{100,120}},
          textColor={213,213,0},
          textString="%name"),
        Text(
          extent={{-100,-120},{100,-160}},
          textColor={107,175,17},
          textString=if show_causality then "%causality" else ""),
        Line(
          points={{100,-60},{140,-60},{140,-140},{100,-140}},
          color={107,175,17},
          arrow=if causality == "" or show_causality == false then {Arrow.None,Arrow.None} else {Arrow.None,Arrow.Filled},
          thickness=0.5,
          pattern=if causality == "" or show_causality == false then LinePattern.None else LinePattern.Solid,
          smooth=Smooth.Bezier)}));
end FuelSensorIcon;
