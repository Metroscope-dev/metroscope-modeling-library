within MetroscopeModelingLibrary.Icons.BoundaryConditions;
partial record WaterSinkIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-16,36},{57,-37}}, color={28,108,200},
          thickness=1),
        Line(points={{-16,-36},{55,35}}, color={28,108,200},
          thickness=1)}));
end WaterSinkIcon;
