within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model Source
  extends PartialBoundaryCondition(redeclare replaceable MetroscopeModelingLibrary.Partial.Connectors.FluidOutlet C);
equation
  C.h_outflow = h;
  C.Xi_outflow = Xi;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-118,58},{2,-62}},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineThickness=1,
          pattern=LinePattern.None),
        Line(points={{-2,0},{36,0},{22,10}}),
        Line(points={{22,-10},{36,0}})}));
end Source;
