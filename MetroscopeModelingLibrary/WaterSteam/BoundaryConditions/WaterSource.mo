within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model WaterSource
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BoundaryConditions.Source(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidOutlet C,
                                            redeclare package Medium = WaterSteamMedium);
  annotation (Icon(graphics={Ellipse(
          extent={{-120,58},{2,-62}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}));
end WaterSource;
