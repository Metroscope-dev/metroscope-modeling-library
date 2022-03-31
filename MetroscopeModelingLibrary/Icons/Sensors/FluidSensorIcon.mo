within MetroscopeModelingLibrary.Icons.Sensors;
partial model FluidSensorIcon "should be extended in partial base classes"
  extends MetroscopeModelingLibrary.Icons.Colors(medium_name = "Partial");
  annotation (Icon(coordinateSystem(preserveAspectRatio=true),
      graphics={
        Ellipse(
          extent={{-100,100},{100,-98}},
          lineColor={0,0,0},
          fillColor=DynamicSelect({28,108,200}, line_color),
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Ellipse(
          extent={{-80,81},{80,-79}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio=true)));
end FluidSensorIcon;
