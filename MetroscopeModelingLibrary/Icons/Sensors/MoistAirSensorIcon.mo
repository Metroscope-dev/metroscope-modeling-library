within MetroscopeModelingLibrary.Icons.Sensors;
partial record MoistAirSensorIcon "should be extended in partial base classes"
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(
      graphics={
        Ellipse(
          extent={{-100,100},{100,-98}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Ellipse(
          extent={{-80,81},{80,-79}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end MoistAirSensorIcon;
