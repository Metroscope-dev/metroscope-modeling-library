within MetroscopeModelingLibrary.Sensors.Fuel;
model PressureSensor
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

  extends Partial.Sensors.PressureSensor(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
end PressureSensor;
