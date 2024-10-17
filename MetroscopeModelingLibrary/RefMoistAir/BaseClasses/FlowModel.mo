within MetroscopeModelingLibrary.RefMoistAir.BaseClasses;
model FlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.RefMoistAirBaseClassIcon;
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
  extends Partial.BaseClasses.FlowModel(
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
    redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputPower W_input(start=0);
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  W = W_input;
  DP = DP_input;
end FlowModel;
