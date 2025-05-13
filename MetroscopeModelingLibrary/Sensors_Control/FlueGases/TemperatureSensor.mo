within MetroscopeModelingLibrary.Sensors_Control.FlueGases;
model TemperatureSensor
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  import MetroscopeModelingLibrary.Utilities.Types;

  extends Partial.Sensors_Control.TemperatureSensor(
    medium=Types.Medium.FlueGases, line=Types.Line.Main, pressure_level=Types.PressureLevel.IP, plant=Types.Plant.CCGT,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlueGasesSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

end TemperatureSensor;
