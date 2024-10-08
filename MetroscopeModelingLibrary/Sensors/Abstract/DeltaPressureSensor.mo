within MetroscopeModelingLibrary.Sensors.Abstract;
model DeltaPressureSensor
  extends MetroscopeModelingLibrary.Partial.Sensors.BaseAbstractSensor;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.AbstractDifferenceSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;
  MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet inlet annotation (Placement(transformation(extent={{-168,-10},{-148,10}}), iconTransformation(extent={{-168,-10},{-148,10}})));
  MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet outlet annotation (Placement(transformation(extent={{148,-10},{168,10}}), iconTransformation(extent={{148,-10},{168,10}})));
end DeltaPressureSensor;
