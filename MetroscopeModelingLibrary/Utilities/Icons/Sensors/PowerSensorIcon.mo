within MetroscopeModelingLibrary.Utilities.Icons.Sensors;
partial record PowerSensorIcon "should be extended in partial base classes"
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  parameter Boolean BC = false "True if the sensor is a boundary condition";
  parameter Boolean no_calibration = false "True is the sensor is not used for calibration";

  annotation (Icon(
      graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor=if BC then {238, 46, 47} elseif no_calibration then {0,140,72} else {255, 255, 255},
          fillPattern=if BC or no_calibration then FillPattern.Solid else FillPattern.None),
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-80,80},{80,-80}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Text(
          extent={{-100,160},{100,120}},
          textColor={244,125,35},
          textString="%name")}));
end PowerSensorIcon;
