within MetroscopeModelingLibrary.Partial.Sensors;
partial model FluidSensorIcon
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Ellipse(
          extent={{-52,52},{52,-52}},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}),
                              Diagram(coordinateSystem(preserveAspectRatio=false)));
end FluidSensorIcon;
