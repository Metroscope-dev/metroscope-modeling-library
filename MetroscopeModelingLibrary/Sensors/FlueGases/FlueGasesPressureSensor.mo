within MetroscopeModelingLibrary.Sensors.FlueGases;
model FlueGasesPressureSensor
  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends MetroscopeModelingLibrary.Icons.Sensors.FlueGasesSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.PressureIcon;

  extends Partial.Sensors.PressureSensor(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=false));
end FlueGasesPressureSensor;
