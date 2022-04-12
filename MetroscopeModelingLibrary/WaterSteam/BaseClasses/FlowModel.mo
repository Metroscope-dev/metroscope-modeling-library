within MetroscopeModelingLibrary.WaterSteam.BaseClasses;
model FlowModel
  extends MetroscopeModelingLibrary.Icons.BaseClasses.WaterSteamBaseClassIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  extends Partial.BaseClasses.FlowModel(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputPower W_input(start=0);
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  W = W_input;
  DP = DP_input;
end FlowModel;
