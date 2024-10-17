within MetroscopeModelingLibrary.RefMoistAir.BaseClasses;
model IsoHFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.RefMoistAirBaseClassIcon;
  package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;
  extends Partial.BaseClasses.IsoHFlowModel(
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
    redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  DP = DP_input;
end IsoHFlowModel;
