within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model WaterSource
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BoundaryConditions.FluidSource(
                                            redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
                                            redeclare package Medium = WaterSteamMedium);
  annotation (
     Diagram(coordinateSystem(preserveAspectRatio=true)));
end WaterSource;
