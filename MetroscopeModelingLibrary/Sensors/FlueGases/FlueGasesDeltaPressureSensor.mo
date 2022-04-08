within MetroscopeModelingLibrary.Sensors.FlueGases;
model FlueGasesDeltaPressureSensor
  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.OtherSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.DeltaPressureIcon;

  extends Partial.Sensors.DeltaPressureSensor(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesInlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesOutlet C_out,
    redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));
end FlueGasesDeltaPressureSensor;
