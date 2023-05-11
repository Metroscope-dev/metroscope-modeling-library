within MetroscopeModelingLibrary.MoistAir.BaseClasses;
model IsoPFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.MoistAirBaseClassIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  extends Partial.BaseClasses.IsoPFlowModel(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputPower W_input(start=0);
equation
  W = W_input;
end IsoPFlowModel;
