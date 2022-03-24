within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model WaterSink
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BoundaryConditions.Sink(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
                                          redeclare package Medium = WaterSteamMedium)
                                          annotation(IconMap(primitivesVisible=false));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-88,0},{-60,0},{-74,10}}),
        Line(points={{-74,-10},{-60,0}}),
        Ellipse(
          extent={{-50,60},{70,-60}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,50},{60,-50}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0})}),             Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end WaterSink;
