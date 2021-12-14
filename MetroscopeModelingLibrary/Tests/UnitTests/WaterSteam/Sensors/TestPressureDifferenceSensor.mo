within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.Sensors;
model TestPressureDifferenceSensor
  extends Modelica.Icons.Example;
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source sourcesWater1
    annotation (Placement(transformation(extent={{-74,10},{-54,30}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink1 annotation (
    Placement(visible = true, transformation(origin={36,20},    extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source sourcesWater2
    annotation (Placement(transformation(extent={{-74,-10},{-54,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink2 annotation (
    Placement(visible = true, transformation(origin={36,0},     extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureLoss singularPressureLoss1
    annotation (Placement(transformation(extent={{-44,10},{-24,30}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureLoss
    singularPressureLoss2
    annotation (Placement(transformation(extent={{-44,-10},{-24,10}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.PressureDifferenceWater
    pressureDifference
    annotation (Placement(transformation(extent={{-4,4},{10,16}})));
equation

   // Use sensor as a sensor (output)
   // Source1
   sourcesWater1.P_out = 20e5;
   sourcesWater1.Q_out = -4000;
   sourcesWater1.T_vol = 20+273.15;
   //Sink1
    sink1.h_vol = 1e6;
   // Source2
   sourcesWater2.P_out = 10e5;
   sourcesWater2.Q_out = -2000;
   sourcesWater2.T_vol = 20+273.15;
   //Sink2
   sink2.h_vol = 1e6;
   //Pressure Losses
   singularPressureLoss1.Kfr = 10;
   singularPressureLoss2.Kfr = 5;


   // Use sensor as a boundary condition (input)
   /*
   // Source1
   sourcesWater1.P_out = 20e5;
   sourcesWater1.Q_out = -4000;
   sourcesWater1.T_vol = 20+273.15;
   //Sink1
    sink1.h_vol = 1e6;
   // Source2
   sourcesWater2.Q_out = -2000;
   sourcesWater2.T_vol = 20+273.15;
   //Sink2
   sink2.h_vol = 1e6;
   //Pressure Losses
   singularPressureLoss1.Kfr = 10;
   singularPressureLoss2.Kfr = 5;
   
   pressureDifference.deltaP = -859873;
   */

  connect(sourcesWater1.C_out, singularPressureLoss1.C_in)
    annotation (Line(points={{-54,20},{-44,20}}, color={238,46,47}));
  connect(singularPressureLoss1.C_out, sink1.C_in)
    annotation (Line(points={{-23.8,20},{26,20}}, color={238,46,47}));
  connect(sourcesWater2.C_out, singularPressureLoss2.C_in)
    annotation (Line(points={{-54,0},{-44,0}}, color={238,46,47}));
  connect(singularPressureLoss2.C_out, sink2.C_in)
    annotation (Line(points={{-23.8,0},{26,0}}, color={238,46,47}));
  connect(pressureDifference.C_in, sink1.C_in)
    annotation (Line(points={{2,16},{2,20},{26,20}}, color={0,0,255}));
  connect(pressureDifference.C_out, sink2.C_in)
    annotation (Line(points={{2,4},{2,0},{26,0}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},{60,40}})));
end TestPressureDifferenceSensor;
