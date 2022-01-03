within MetroscopeModelingLibrary.Tests.SimpleExamples.WaterSteam;
model TestPumpPressureLoss_1
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;
  input Real valveOpening(start=1);
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve controlValve
    annotation (Placement(visible = true, transformation(origin={-45,17},         extent={{
            -9.00001,-11},{9.00001,11}},                                                                                     rotation = 0)));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source sourcesWater
    annotation (Placement(visible=true, transformation(extent={{-106,0},{-86,20}},
          rotation=0)));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sinkWater
    annotation (Placement(transformation(extent={{-86,-46},{-106,-26}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StaticCentrifugalPump
    staticCentrifugalPump
    annotation (Placement(transformation(extent={{16,20},{36,0}})));
  Modelica.Blocks.Sources.Constant const1(k=1400)
    annotation (Placement(transformation(extent={{-26,38},{-6,58}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureLoss
    singularPressureLoss annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={72,-14})));
equation
  connect(controlValve.C_out, staticCentrifugalPump.C_in) annotation (
    Line(points={{-35.82,10},{16,10}},  color = {238, 46, 47}));
  connect(sourcesWater.C_out, controlValve.C_in) annotation (
    Line(points={{-86,10},{-54,10}},      color = {238, 46, 47}));
  sourcesWater.P_out = 2e5;
  sourcesWater.T_vol = 15 + 273.15;
  sinkWater.P_in = 2.2e5;
  sinkWater.T_vol = 85 + 273.15;
  controlValve.Cvmax = 8005.42;
  controlValve.Opening = valveOpening;
  singularPressureLoss.Kfr = 1e3;
  staticCentrifugalPump.VRotn=1400;
  staticCentrifugalPump.rm=0.85;
  staticCentrifugalPump.a1=-88.67;
  staticCentrifugalPump.a2=0;
  staticCentrifugalPump.a3=43.15;
  staticCentrifugalPump.b1=-3.7751;
  staticCentrifugalPump.b2=3.61;
  staticCentrifugalPump.b3=-0.0075464;
  staticCentrifugalPump.rhmin=0.20;
  connect(const1.y, staticCentrifugalPump.VRot)
    annotation (Line(points={{-5,48},{26,48},{26,22}},       color={0,0,127}));
  connect(staticCentrifugalPump.C_out, singularPressureLoss.C_in)
    annotation (Line(points={{36.2,10},{72,10},{72,-4}}, color={238,46,47}));
  connect(singularPressureLoss.C_out, sinkWater.C_in) annotation (Line(points={{72,
          -24.2},{72,-36},{-86,-36}},    color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end TestPumpPressureLoss_1;
