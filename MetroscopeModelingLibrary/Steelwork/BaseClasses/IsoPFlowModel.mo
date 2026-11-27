within MetroscopeModelingLibrary.Steelwork.BaseClasses;
model IsoPFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.FuelBaseClassIcon;
  package SteelworksGasMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasMedium;
  extends Partial.BaseClasses.IsoPFlowModel(
    redeclare MetroscopeModelingLibrary.Steelwork.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Steelwork.Connectors.Outlet C_out,
    redeclare package Medium = SteelworksGasMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputPower W_input(start=0);
equation
  W = W_input;
end IsoPFlowModel;
