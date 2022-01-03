within MetroscopeModelingLibrary.MoistAir.Sensors;
model MoistAirPressureSensor
    replaceable package MoistAirMedium =
      MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
  extends MetroscopeModelingLibrary.Common.Sensors.PressureSensor(redeclare
      package
      Medium =
        MoistAirMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end MoistAirPressureSensor;
