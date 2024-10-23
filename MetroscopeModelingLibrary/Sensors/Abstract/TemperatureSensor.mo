within MetroscopeModelingLibrary.Sensors.Abstract;
model TemperatureSensor
  extends MetroscopeModelingLibrary.Partial.Sensors.BaseAbstractSensor annotation(IconMap(primitivesVisible=true));
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.AbstractSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;
end TemperatureSensor;
