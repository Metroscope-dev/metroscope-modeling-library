within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.Sensors;
model TestMassFlowSensor
  extends Modelica.Icons.Example;
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-74,10},{-54,30}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (
     Placement(visible=true, transformation(
        origin={46,20},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureLoss PL
    annotation (Placement(transformation(extent={{-34,10},{-14,30}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.MassFlowWater MassFlowSensor
    annotation (Placement(transformation(extent={{6,20},{16,36}})));
equation

  // Use sensor as a sensor
  source.P_out = 20e5;
  source.Q_out = -4000;
  source.T_vol = 20 + 273.15;
  sink.h_vol = 1e6;
  PL.Kfr = 100;

  // Use sensor as a boundary condition
  /*
  source.P_out = 20e5;
  source.T_vol = 20 + 273.15;
  sink.h_vol = 1e6;
  PL.Kfr = 100;
  
  MassFlowSensor.Q = 4000;
  */


  connect(source.C_out, PL.C_in)
    annotation (Line(points={{-54,20},{-34,20}}, color={238,46,47}));
  connect(PL.C_out, MassFlowSensor.C_in)
    annotation (Line(points={{-13.8,20},{7,20}}, color={238,46,47}));
  connect(MassFlowSensor.C_out, sink.C_in)
    annotation (Line(points={{13,20},{36,20}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                           Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,0},{60,40}})));
end TestMassFlowSensor;
