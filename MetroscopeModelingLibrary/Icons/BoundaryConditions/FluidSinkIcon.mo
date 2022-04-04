within MetroscopeModelingLibrary.Icons.BoundaryConditions;
partial record FluidSinkIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Line(points={{-88,0},{-60,0},{-74,10}}),
        Line(points={{-74,-10},{-60,0}}),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-15,35},{55,-35}}, color={0,0,0}),
        Line(points={{-15,-35},{55,35}}, color={0,0,0})}));
end FluidSinkIcon;
