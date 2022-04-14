within MetroscopeModelingLibrary.Icons.Sensors;
partial record PowerSensorIcon "should be extended in partial base classes"
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(
      graphics={
        Ellipse(
          extent={{-100,100},{100,-98}},
          lineColor={0,0,0},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Ellipse(
          extent={{-80,81},{80,-79}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}));
end PowerSensorIcon;
