within MetroscopeModelingLibrary.Fuel.BaseClasses;
model IsoHFlowModel
  extends MetroscopeModelingLibrary.Icons.BaseClasses.FuelBaseClassIcon;
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends Partial.BaseClasses.IsoHFlowModel(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  DP = DP_input;
end IsoHFlowModel;
