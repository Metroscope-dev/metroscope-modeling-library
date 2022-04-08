within MetroscopeModelingLibrary.Sensors.FlueGases;
model FlueGasesTemperatureSensor
  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.FlueGasesSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.TemperatureIcon;

  extends Partial.Sensors.TemperatureSensor(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesInlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesOutlet C_out,
    redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));
end FlueGasesTemperatureSensor;
