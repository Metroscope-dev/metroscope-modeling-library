within MetroscopeModelingLibrary.WaterSteam.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Icons.BoundaryConditions.WaterSourceIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out, redeclare
      package                                                                                                                              Medium =
        WaterSteamMedium)                                                                                                                                             annotation (IconMap(
        primitivesVisible=false));
end Source;
