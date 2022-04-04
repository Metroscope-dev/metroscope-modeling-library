within MetroscopeModelingLibrary.Sensors.Other;
model VRotSensor
  extends MetroscopeModelingLibrary.Icons.Sensors.OtherSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.VRotIcon;
  import MetroscopeModelingLibrary.Units.Inputs;

  Modelica.Blocks.Interfaces.RealOutput VRot(min=0, nominal=2000, start=2000) "rotations per minute" annotation (Placement(transformation(
        extent={{-27,-27},{27,27}},
        rotation=270,
        origin={1,-17}), iconTransformation(extent={{-27,-27},{27,27}},
        rotation=270,
        origin={0,-102})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end VRotSensor;
