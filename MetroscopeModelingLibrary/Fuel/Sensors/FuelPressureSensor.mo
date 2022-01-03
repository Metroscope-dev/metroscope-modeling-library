within MetroscopeModelingLibrary.Fuel.Sensors;
model FuelPressureSensor
    replaceable package FuelMedium =
      MetroscopeModelingLibrary.Fuel.Medium.FuelMedium;
  extends MetroscopeModelingLibrary.Common.Sensors.PressureSensor(redeclare
      package Medium =
        FuelMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuelPressureSensor;
