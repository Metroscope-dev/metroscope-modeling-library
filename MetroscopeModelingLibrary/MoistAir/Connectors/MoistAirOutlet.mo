within MetroscopeModelingLibrary.MoistAir.Connectors;
connector MoistAirOutlet
  extends MetroscopeModelingLibrary.Icons.Connectors.MoistAirOutletIcon;

  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium = MoistAirMedium) annotation(IconMap(primitivesVisible=false));
end MoistAirOutlet;