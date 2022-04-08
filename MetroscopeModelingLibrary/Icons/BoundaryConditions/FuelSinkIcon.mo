within MetroscopeModelingLibrary.Icons.BoundaryConditions;
partial record FuelSinkIcon
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={0,0,0},
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(points={{-15,35},{55,-35}}, color={213,213,0},
          thickness=0.5),
        Line(points={{-15,-35},{55,35}}, color={213,213,0},
          thickness=0.5)}));
end FuelSinkIcon;
