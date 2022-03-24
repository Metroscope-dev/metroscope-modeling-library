within MetroscopeModelingLibrary.WaterSteam.BaseClasses;
model WaterIsoPHFlowModel
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BaseClasses.IsoPHFlowModel(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
    redeclare package Medium = WaterSteamMedium);
end WaterIsoPHFlowModel;
