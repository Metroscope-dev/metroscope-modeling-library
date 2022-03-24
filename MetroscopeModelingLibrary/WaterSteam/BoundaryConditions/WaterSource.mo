within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model WaterSource
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BoundaryConditions.Source(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
                                            redeclare package Medium = WaterSteamMedium)
                                            annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={Ellipse(
          extent={{-88,60},{34,-60}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(points={{50,0},{78,0},{64,10}}),
        Line(points={{64,-10},{78,0}})}));
end WaterSource;
