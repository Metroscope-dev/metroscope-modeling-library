within MetroscopeModelingLibrary.Utilities.Icons.Sensors;
partial record MoistAirSensorIcon "should be extended in partial base classes"
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  annotation (Icon(
      graphics={
        Ellipse(
          extent={{-100,100},{100,-98}},
          lineColor={0,0,0},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-80,81},{80,-79}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Text(
          extent={{-100,160},{100,120}},
          textColor={85,170,255},
          textString="%name")}));
end MoistAirSensorIcon;
