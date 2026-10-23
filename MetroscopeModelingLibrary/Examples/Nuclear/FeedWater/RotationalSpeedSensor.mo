within MetroscopeModelingLibrary.Examples.Nuclear.FeedWater;
model RotationalSpeedSensor "MML4 signal adapter for the library rotational-speed sensor"
  extends MetroscopeModelingLibrary.SensorsControl.Outline.VRotSensor(
    VRot(start=VRot_start));
  parameter Real VRot_start=1000 "Initialization-point speed in rpm";
  parameter String sensor_function="Unidentified";
  parameter String causality="";
  Utilities.Interfaces.GenericReal VRot_sensor(start=VRot_start)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,102}),
      iconTransformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,102})));
equation
  VRot_sensor = VRot;
end RotationalSpeedSensor;
