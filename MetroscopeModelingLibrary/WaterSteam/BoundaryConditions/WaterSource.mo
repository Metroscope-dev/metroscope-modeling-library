within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model WaterSource
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BoundaryConditions.FluidSource(
                                            redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
                                            redeclare package Medium = WaterSteamMedium);
  annotation (Icon(graphics={
        Line(points={{50,0},{78,0},{64,10}}),
        Line(points={{64,-10},{78,0}})}),
     Diagram(coordinateSystem(preserveAspectRatio=true)));
end WaterSource;
