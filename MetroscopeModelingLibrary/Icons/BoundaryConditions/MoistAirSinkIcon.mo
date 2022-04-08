within MetroscopeModelingLibrary.Icons.BoundaryConditions;
partial record MoistAirSinkIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={85,170,255},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={85,170,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-18,38},{55,-35}}, color={85,170,255},
          thickness=1),
        Line(points={{-18,-38},{55,35}}, color={85,170,255},
          thickness=1)}));
end MoistAirSinkIcon;
