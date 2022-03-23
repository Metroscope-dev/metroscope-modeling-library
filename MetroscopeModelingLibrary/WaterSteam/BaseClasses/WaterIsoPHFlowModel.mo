within MetroscopeModelingLibrary.WaterSteam.BaseClasses;
model WaterIsoPHFlowModel
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BaseClasses.IsoPHFlowModel(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidInlet C_in,
                                             redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidOutlet C_out,
                                             redeclare package Medium = WaterSteamMedium);
end WaterIsoPHFlowModel;
