within MetroscopeModelingLibrary.WaterSteam.Connectors;
connector WaterOutlet
  extends MetroscopeModelingLibrary.Icons.Connectors.WaterOutletIcon;

  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium = WaterSteamMedium, medium_name="WaterSteam") annotation(IconMap(primitivesVisible=false));
end WaterOutlet;
