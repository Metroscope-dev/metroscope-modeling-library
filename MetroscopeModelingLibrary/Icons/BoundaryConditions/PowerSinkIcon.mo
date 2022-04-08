within MetroscopeModelingLibrary.Icons.BoundaryConditions;
partial record PowerSinkIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={255,128,0},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={255,128,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-18,38},{55,-35}}, color={255,128,0},
          thickness=1),
        Line(points={{-18,-38},{55,35}}, color={255,128,0},
          thickness=1)}));
end PowerSinkIcon;
