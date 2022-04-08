within MetroscopeModelingLibrary.Icons.BoundaryConditions;
partial record FluidSinkIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-16,36},{57,-37}}, color={0,0,0},
          thickness=1),
        Line(points={{-16,-36},{57,37}}, color={0,0,0},
          thickness=1)}));
end FluidSinkIcon;
