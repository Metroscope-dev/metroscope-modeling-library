within MetroscopeModelingLibrary.WaterSteam.BaseClasses;
model WaterFlowModel
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BaseClasses.FlowModel(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidInlet C_in,
                                        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidOutlet C_out,
                                        redeclare package Medium = WaterSteamMedium);

  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputPower W_input(start=0);
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  W = W_input;
  DP = DP_input;
end WaterFlowModel;
