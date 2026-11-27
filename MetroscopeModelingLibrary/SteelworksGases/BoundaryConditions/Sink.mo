within MetroscopeModelingLibrary.SteelworksGases.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package SteelworksGasMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  extends Partial.BoundaryConditions.FluidSink(redeclare
      MetroscopeModelingLibrary.SteelworksGases.Connectors.Inlet C_in,                                         redeclare package Medium = SteelworksGasMedium) annotation (IconMap(primitivesVisible=false));
  annotation (Icon(coordinateSystem(initialScale=0.2),
                   graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={0,0,0},
          fillColor={213,170,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={213,170,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(points={{-15,35},{55,-35}}, color={213,170,255},
          thickness=0.5),
        Line(points={{-15,-35},{55,35}}, color={213,170,255},
          thickness=0.5)}), Diagram(coordinateSystem(initialScale=0.2)));
end Sink;
