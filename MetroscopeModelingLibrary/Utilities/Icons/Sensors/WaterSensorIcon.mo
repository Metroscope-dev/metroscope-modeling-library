within MetroscopeModelingLibrary.Utilities.Icons.Sensors;
partial record WaterSensorIcon "should be extended in partial base classes"
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  annotation (Icon(
      graphics={
        Ellipse(
          extent={{-100,100},{100,-98}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-80,81},{80,-79}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}));
end WaterSensorIcon;
