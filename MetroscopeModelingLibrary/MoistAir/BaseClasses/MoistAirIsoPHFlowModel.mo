within MetroscopeModelingLibrary.MoistAir.BaseClasses;
model MoistAirIsoPHFlowModel
  package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
  extends Partial.BaseClasses.IsoPHFlowModel(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
    redeclare package Medium = MoistAirMedium, transports_partial=false, transports_moist_air=true);
end MoistAirIsoPHFlowModel;
