within MetroscopeModelingLibrary.Fuel.Sensors;
model FuelTemperatureSensor
    replaceable package FuelMedium =
      MetroscopeModelingLibrary.Fuel.Medium.FuelMedium;
  extends MetroscopeModelingLibrary.Common.Sensors.TemperatureSensor(redeclare
      package Medium =
        FuelMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuelTemperatureSensor;
