within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model Source
  extends PartialBoundaryCondition(redeclare MetroscopeModelingLibrary.Connectors.FluidConnectors.FluidOutlet C);
equation
  C.h_outflow = h;
  C.Xi_outflow = Xi;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-60,60},{60,-60}},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineThickness=1,
          pattern=LinePattern.None),
        Line(points={{60,0},{98,0},{84,10}}),
        Line(points={{84,-10},{98,0}})}));
end Source;
