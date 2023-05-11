within MetroscopeModelingLibrary.MoistAir.BaseClasses;
model IsoHFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.MoistAirBaseClassIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  extends Partial.BaseClasses.IsoHFlowModel(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  DP = DP_input;
end IsoHFlowModel;
