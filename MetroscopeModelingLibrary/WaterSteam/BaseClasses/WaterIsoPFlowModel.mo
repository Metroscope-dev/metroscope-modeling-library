within MetroscopeModelingLibrary.WaterSteam.BaseClasses;
model WaterIsoPFlowModel
  extends MetroscopeModelingLibrary.Icons.BaseClasses.WaterSteamBaseClassIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BaseClasses.IsoPFlowModel(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterInlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterOutlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation(primitivesVisible=false);

  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputPower W_input(start=0);
equation
  W = W_input;
end WaterIsoPFlowModel;
