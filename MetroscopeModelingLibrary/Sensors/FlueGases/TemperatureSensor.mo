within MetroscopeModelingLibrary.Sensors.FlueGases;
model TemperatureSensor
  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.FlueGasesSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.TemperatureIcon;

  extends Partial.Sensors.TemperatureSensor(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=false));
end TemperatureSensor;
