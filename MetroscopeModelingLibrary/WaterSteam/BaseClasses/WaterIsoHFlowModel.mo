within MetroscopeModelingLibrary.WaterSteam.BaseClasses;
model WaterIsoHFlowModel
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BaseClasses.IsoHFlowModel(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
    redeclare package Medium = WaterSteamMedium);

  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  DP = DP_input;
end WaterIsoHFlowModel;
