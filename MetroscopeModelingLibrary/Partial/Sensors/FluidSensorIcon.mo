within MetroscopeModelingLibrary.Partial.Sensors;
partial model FluidSensorIcon
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-100,98},{100,-100}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Ellipse(
          extent={{-80,79},{80,-81}},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}),
     Diagram(coordinateSystem(preserveAspectRatio=true)));
end FluidSensorIcon;
