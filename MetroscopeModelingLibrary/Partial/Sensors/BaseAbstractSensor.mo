within MetroscopeModelingLibrary.Partial.Sensors;
model BaseAbstractSensor
  extends MetroscopeModelingLibrary.Icons.Sensors.AbstractSensorIcon;
  WaterSteam.Connectors.Inlet inlet annotation (Placement(transformation(extent={{-30,-150},{-10,-130}}), iconTransformation(extent={{-30,-150},{-10,-130}})));
  WaterSteam.Connectors.Outlet outlet annotation (Placement(transformation(extent={{10,-150},{30,-130}}), iconTransformation(extent={{10,-150},{30,-130}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=true), graphics={Rectangle(
          extent={{-12,-136},{12,-144}},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}),                          Diagram(coordinateSystem(preserveAspectRatio=true)));
end BaseAbstractSensor;