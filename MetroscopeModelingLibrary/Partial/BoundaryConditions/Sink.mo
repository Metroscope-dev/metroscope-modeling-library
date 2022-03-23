within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model Sink
  extends PartialBoundaryCondition(redeclare replaceable MetroscopeModelingLibrary.Partial.Connectors.FluidInlet C);
equation
  inStream(C.h_outflow) = h;
  inStream(C.Xi_outflow) = Xi;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-40,0},{-12,0},{-26,10}}),
        Line(points={{-26,-10},{-12,0}}),
        Ellipse(
          extent={{8,60},{128,-60}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{18,50},{118,-50}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0})}),             Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end Sink;
