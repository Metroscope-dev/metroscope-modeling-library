within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.Sensors;
model TestSensors_NoEq
  extends Modelica.Icons.Example;
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-74,10},{-54,30}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (
     Placement(visible=true, transformation(
        origin={46,20},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureLoss PL
    annotation (Placement(transformation(extent={{-22,10},{-2,30}})));
  Common.Sensors_NoEq.TemperatureSensor T
    annotation (Placement(transformation(extent={{-58,22},{-38,42}})));
  Common.Sensors_NoEq.PressureSensor P
    annotation (Placement(transformation(extent={{6,22},{26,42}})));
  Common.Sensors_NoEq.FlowSensor Q
    annotation (Placement(transformation(extent={{-42,22},{-22,42}})));
  Common.Sensors_NoEq.DeltaPSensor DeltaP
    annotation (Placement(transformation(extent={{-28,-6},{6,14}})));
equation

  // Sensors_NoEq are only here to mark on the model graph where the measurements are
  // They do not appear in the equations
  source.P_out = 20e5;
  source.Q_out = -4000;
  source.T_vol = 20 + 273.15;
  sink.h_vol = 1e6;
  PL.Kfr = 1e-3;


  connect(source.C_out, PL.C_in)
    annotation (Line(points={{-54,20},{-22,20}}, color={238,46,47}));
  connect(PL.C_out, sink.C_in)
    annotation (Line(points={{-1.8,20},{36,20}},  color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                           Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,0},{60,40}})));
end TestSensors_NoEq;
