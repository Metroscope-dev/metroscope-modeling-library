within MetroscopeModelingLibrary.Sensors.Fuel;
model FlowSensor
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

  extends Partial.Sensors.FlowSensor(
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
end FlowSensor;
