within MetroscopeModelingLibrary.WaterSteam.BaseClasses;
model IsoPFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.WaterSteamBaseClassIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  extends Partial.BaseClasses.IsoPFlowModel(
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
    redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputPower W_input(start=0);
equation
  W = W_input;
end IsoPFlowModel;
