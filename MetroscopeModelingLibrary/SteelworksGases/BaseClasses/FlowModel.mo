within MetroscopeModelingLibrary.SteelworksGases.BaseClasses;
model FlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.SteelworksBaseClassIcon;
  package SteelworksGasMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  extends Partial.BaseClasses.FlowModel(
    redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Outlet C_out,
    redeclare package Medium = SteelworksGasMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputPower W_input(start=0);
  Inputs.InputDifferentialPressure DP_input(start=0);
equation
  W = W_input;
  DP = DP_input;
end FlowModel;
