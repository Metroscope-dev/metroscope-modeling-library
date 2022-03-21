within MetroscopeModelingLibrary.Partial.BoundaryConditions;
partial model Sink
  extends PartialBoundaryCondition(redeclare MetroscopeModelingLibrary.Connectors.FluidConnectors.FluidInlet C);
equation
  inStream(C.h_outflow) = h;
  inStream(C.Xi_outflow) = Xi;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-98,0},{-60,0},{-74,10}}),
        Line(points={{-74,-10},{-60,0}}),
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-50,50},{50,-50}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0})}),             Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end Sink;
