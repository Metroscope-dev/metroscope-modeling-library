within MetroscopeModelingLibrary.MoistAir.Sensors;
model MoistAirHumiditySensor
    replaceable package MoistAirMedium =
      MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
  extends MetroscopeModelingLibrary.Common.Sensors.HumiditySensor(redeclare
      package Medium =
        MoistAirMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end MoistAirHumiditySensor;
