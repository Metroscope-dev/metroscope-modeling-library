within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Utilities.Icons.Connectors.WaterOutletIcon;

  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium =
        WaterSteamMedium)                                                             annotation(IconMap(primitivesVisible=false));
end Outlet;
