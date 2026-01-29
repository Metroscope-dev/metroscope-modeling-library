within MetroscopeModelingLibrary.FlueGases.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in, redeclare package Medium =
        FlueGasesMedium)                                                                                                                                        annotation (IconMap(primitivesVisible=
          false));
  annotation (Icon(coordinateSystem(initialScale=0.2),
                   graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={95,95,95},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-18,38},{55,-35}}, color={95,95,95},
          thickness=1),
        Line(points={{-18,-38},{55,35}}, color={95,95,95},
          thickness=1)}), Diagram(coordinateSystem(initialScale=0.2)));
end Sink;
