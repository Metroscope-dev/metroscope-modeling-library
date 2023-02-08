within MetroscopeModelingLibrary.Fuel.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in, redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
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
end Sink;
