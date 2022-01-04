within MetroscopeModelingLibrary.MoistAir.Sensors;
model MoistAirFlowSensor
  package MoistAirMedium =
      MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
  extends MetroscopeModelingLibrary.Common.Sensors.FlowSensor(redeclare package
              Medium =
        MoistAirMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end MoistAirFlowSensor;
