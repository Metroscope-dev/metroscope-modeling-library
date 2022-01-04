within MetroscopeModelingLibrary.Fuel.Sensors;
model FuelFlowSensor
  package FuelMedium =
      MetroscopeModelingLibrary.Fuel.Medium.FuelMedium;
  extends MetroscopeModelingLibrary.Common.Sensors.FlowSensor(redeclare package
      Medium =
        FuelMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FuelFlowSensor;
