within MetroscopeModelingLibrary.WaterSteam.BaseClasses;
model WaterIsoPFlowModel
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BaseClasses.IsoPFlowModel(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidInlet C_in,
                                            redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidOutlet C_out,
                                            redeclare package Medium = WaterSteamMedium);

  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputPower W_input(start=0);
equation
  W = W_input;
end WaterIsoPFlowModel;
