within MetroscopeModelingLibrary.Partial.Sensors;
model BaseAbstractSensor
  extends MetroscopeModelingLibrary.Icons.Sensors.AbstractSensorIcon;
  WaterSteam.Connectors.Inlet inlet annotation (Placement(transformation(extent={{-30,-160},{-10,-140}}), iconTransformation(extent={{-30,-160},{-10,-140}})));
  WaterSteam.Connectors.Outlet outlet annotation (Placement(transformation(extent={{10,-160},{30,-140}}), iconTransformation(extent={{10,-160},{30,-140}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=true), graphics={Rectangle(
          extent={{-10,-146},{14,-154}},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}),                          Diagram(coordinateSystem(preserveAspectRatio=true)));
end BaseAbstractSensor;
