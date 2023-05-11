within MetroscopeModelingLibrary.Sensors.Outline;
model VRotSensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.VRotIcon;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Modelica.Blocks.Interfaces.RealOutput VRot(min=0, nominal=2000, start=2000) "rotations per minute" annotation (Placement(transformation(
        extent={{-27,-27},{27,27}},
        rotation=270,
        origin={1,-17}), iconTransformation(extent={{-27,-27},{27,27}},
        rotation=270,
        origin={0,-102})));
end VRotSensor;
