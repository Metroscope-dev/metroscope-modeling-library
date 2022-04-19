within MetroscopeModelingLibrary.Sensors.FlueGases;
model FlueGasesDeltaPressureSensor
  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.OutlineSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.DeltaPressureIcon;

  extends Partial.Sensors.DeltaPressureSensor(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=false));
end FlueGasesDeltaPressureSensor;
