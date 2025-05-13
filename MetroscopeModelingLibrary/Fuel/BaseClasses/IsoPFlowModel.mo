within MetroscopeModelingLibrary.Fuel.BaseClasses;
model IsoPFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.BaseClasses.FuelBaseClassIcon;
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.BaseClasses.IsoPFlowModel(
    medium=Types.Medium.Fuel, line=Types.Line.Main, pressure_level=Types.PressureLevel.IP, plant=Types.Plant.CCGT,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Utilities.Types;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputPower W_input(start=0);
equation
  W = W_input;

  assert(medium == Types.Medium.Fuel, "only fuel medium config accepted for Fuel flow model");
end IsoPFlowModel;
