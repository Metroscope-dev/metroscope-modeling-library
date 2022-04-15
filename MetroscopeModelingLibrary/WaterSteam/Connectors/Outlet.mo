within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Icons.Connectors.WaterOutletIcon;

  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium =
        WaterSteamMedium)                                                             annotation(IconMap(primitivesVisible=false));
end Outlet;
