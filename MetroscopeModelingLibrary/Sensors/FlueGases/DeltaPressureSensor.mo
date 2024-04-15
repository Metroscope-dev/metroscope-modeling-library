within MetroscopeModelingLibrary.Sensors.FlueGases;
model DeltaPressureSensor
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

  extends Partial.Sensors.DeltaPressureSensor(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

end DeltaPressureSensor;
