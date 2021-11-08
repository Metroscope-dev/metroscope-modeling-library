within MetroscopeModelingLibrary.MoistAir.Sensors;
model MoistAirTemperatureSensor
    replaceable package MoistAirMedium =
      MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
  extends MetroscopeModelingLibrary.Common.Sensors.TemperatureSensor(redeclare
      package Medium =
        MoistAirMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end MoistAirTemperatureSensor;
