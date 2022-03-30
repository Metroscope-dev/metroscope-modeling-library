within MetroscopeModelingLibrary.MoistAir.Connectors;
connector MoistAirOutlet
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium = MoistAirMedium);
end MoistAirOutlet;
