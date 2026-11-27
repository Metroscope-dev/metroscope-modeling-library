within MetroscopeModelingLibrary.Steelwork.BaseClasses;
model IsoHFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.FuelBaseClassIcon;
  package SteelworksGasMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasMedium;
  extends Partial.BaseClasses.IsoHFlowModel(
    redeclare MetroscopeModelingLibrary.Steelwork.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Steelwork.Connectors.Outlet C_out,
    redeclare package Medium = SteelworksGasMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  DP = DP_input;
end IsoHFlowModel;
