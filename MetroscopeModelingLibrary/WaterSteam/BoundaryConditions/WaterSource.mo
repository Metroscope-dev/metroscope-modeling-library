within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model WaterSource
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.WaterSourceIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BoundaryConditions.FluidSource(
                                            redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
                                            redeclare package Medium = WaterSteamMedium) annotation(IconMap(primitivesVisible=false));
  annotation (
     Diagram(coordinateSystem(preserveAspectRatio=true)));
end WaterSource;
