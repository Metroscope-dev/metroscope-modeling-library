within ;
model Steam_turbine_extraction_exercise
  MetroscopeModelingLibrary.Power.Machines.Generator generator
    annotation (Placement(transformation(extent={{38,30},{64,46}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source(
    P_0=5000000,
    Q_0=1000,
    h_0=3200000,
    T_0=673.15)
    annotation (Placement(transformation(extent={{-90,-6},{-80,4}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine steamTurbine2
    annotation (Placement(transformation(extent={{14,-10},{28,8}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation
    (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={2,-30})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink1
    annotation (Placement(transformation(extent={{56,-7},{68,5}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine steamTurbine1
    annotation (Placement(transformation(extent={{-42,-10},{-28,8}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter
    steamExtractionSplitter(Q_ext_0=100)
    annotation (Placement(transformation(extent={{-6,-9},{10,7}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink2
    annotation (Placement(transformation(extent={{80,29},{98,47}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor
    pressureSensor(sensor_function="Calibration", causality="steamTurbine1_eta_is")
    annotation (Placement(transformation(extent={{-16,-4},{-10,2}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor
    pressureSensor1(sensor_function="Calibration", causality="steamTurbine2_Cst")
    annotation (Placement(transformation(extent={{38,-4},{44,2}})));
  MetroscopeModelingLibrary.SensorsControl.Power.PowerSensor powerSensor(
      sensor_function="Calibration", causality="steamTurbine2_eta_is")
    annotation (Placement(transformation(extent={{68,34},{76,42}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor
    temperatureSensor(sensor_function="BC")
    annotation (Placement(transformation(extent={{-76,-4},{-70,2}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor
    pressureSensor2(sensor_function="BC")
    annotation (Placement(transformation(extent={{-66,-4},{-60,2}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor
    temperatureSensor1(sensor_function="Calibration", causality="steamTurbine1_Cst")
    annotation (Placement(transformation(extent={{-24,-4},{-18,2}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor flowSensor(
      sensor_function="BC")
    annotation (Placement(transformation(extent={{-2,-22},{6,-14}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor flowSensor1(
      sensor_function="BC")
    annotation (Placement(transformation(extent={{-56,-4},{-50,2}})));
  Modelica.Blocks.Sources.Constant const1(k=50)
    annotation (Placement(transformation(extent={{-70,8},{-66,12}})));
  Modelica.Blocks.Sources.Constant const2(k=1000)
    annotation (Placement(transformation(extent={{-60,8},{-56,12}})));
  Modelica.Blocks.Sources.Constant const3(k=100)
    annotation (Placement(transformation(extent={{-14,-16},{-10,-12}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal steamTurbine1_Cst
    annotation (Placement(transformation(extent={{-46,-16},{-42,-12}}),
        iconTransformation(extent={{-64,-14},{-44,6}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal
    steamTurbine1_eta_is annotation (Placement(transformation(extent={{-40,-18},
            {-36,-14}}), iconTransformation(extent={{-64,-14},{-44,6}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal steamTurbine2_Cst
    annotation (Placement(transformation(extent={{12,-14},{16,-10}}),
        iconTransformation(extent={{-64,-14},{-44,6}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal
    steamTurbine2_eta_is annotation (Placement(transformation(extent={{18,-16},{
            22,-12}}), iconTransformation(extent={{-64,-14},{-44,6}})));
  Modelica.Blocks.Sources.Constant const4(k=200)
    annotation (Placement(transformation(extent={{-20,6},{-16,10}})));
  Modelica.Blocks.Sources.Constant const5(k=30)
    annotation (Placement(transformation(extent={{-10,6},{-6,10}})));
  Modelica.Blocks.Sources.Constant const6(k=20)
    annotation (Placement(transformation(extent={{44,4},{48,8}})));
  Modelica.Blocks.Sources.Constant const7(k=2000)
    annotation (Placement(transformation(extent={{72,44},{76,48}})));
  Modelica.Blocks.Sources.Constant const8(k=1)
    annotation (Placement(transformation(extent={{4,6},{8,10}})));
equation
  source.h_out = 3196600;
  connect(generator.C_in, steamTurbine1.C_W_out) annotation (Line(points={{42.94,
          38},{-22,38},{-22,6.56},{-28,6.56}}, color={244,125,35}));
  connect(generator.C_in, steamTurbine2.C_W_out) annotation (Line(points={{42.94,
          38},{34,38},{34,6.56},{28,6.56}}, color={244,125,35}));
  connect(steamExtractionSplitter.C_main_out, steamTurbine2.C_in)
    annotation (Line(points={{10.48,-1},{14,-1}}, color={28,108,200}));
  connect(pressureSensor.C_out, steamExtractionSplitter.C_in)
    annotation (Line(points={{-10,-1},{-6.48,-1}}, color={28,108,200}));
  connect(steamTurbine2.C_out, pressureSensor1.C_in)
    annotation (Line(points={{28,-1},{38,-1}}, color={28,108,200}));
  connect(pressureSensor1.C_out, sink1.C_in)
    annotation (Line(points={{44,-1},{59,-1}}, color={28,108,200}));
  connect(generator.C_out, powerSensor.C_in)
    annotation (Line(points={{60.1,38},{68,38}}, color={244,125,35}));
  connect(sink2.C_in, powerSensor.C_out)
    annotation (Line(points={{84.5,38},{75.92,38}}, color={244,125,35}));
  connect(source.C_out, temperatureSensor.C_in)
    annotation (Line(points={{-82.5,-1},{-76,-1}}, color={28,108,200}));
  connect(temperatureSensor.C_out, pressureSensor2.C_in)
    annotation (Line(points={{-70,-1},{-66,-1}}, color={28,108,200}));
  connect(steamTurbine1.C_out, temperatureSensor1.C_in)
    annotation (Line(points={{-28,-1},{-24,-1}}, color={28,108,200}));
  connect(temperatureSensor1.C_out, pressureSensor.C_in)
    annotation (Line(points={{-18,-1},{-16,-1}}, color={28,108,200}));
  connect(flowSensor.C_in, steamExtractionSplitter.C_ext_out) annotation (Line(
        points={{-2,-18},{-2,-12},{2,-12},{2,-6.44}}, color={28,108,200}));
  connect(flowSensor.C_out, sink.C_in)
    annotation (Line(points={{6,-18},{2,-18},{2,-27}}, color={28,108,200}));
  connect(pressureSensor2.C_out, flowSensor1.C_in)
    annotation (Line(points={{-60,-1},{-56,-1}}, color={28,108,200}));
  connect(flowSensor1.C_out, steamTurbine1.C_in)
    annotation (Line(points={{-50,-1},{-42,-1}}, color={28,108,200}));
  connect(const1.y, pressureSensor2.P_sensor) annotation (Line(points={{-65.8,10},
          {-64,10},{-64,2},{-63,2}}, color={0,0,127}));
  connect(const2.y, flowSensor1.Q_sensor) annotation (Line(points={{-55.8,10},{-54,
          10},{-54,2},{-53,2}}, color={0,0,127}));
  connect(const3.y, flowSensor.Q_sensor)
    annotation (Line(points={{-9.8,-14},{2,-14}}, color={0,0,127}));
  connect(steamTurbine1_Cst, steamTurbine1.Cst) annotation (Line(points={{-44,-14},
          {-44,-12},{-39.2,-12},{-39.2,-7.3}}, color={0,0,127}));
  connect(steamTurbine1_eta_is, steamTurbine1.eta_is) annotation (Line(points={{
          -38,-16},{-38,-12},{-36.4,-12},{-36.4,-8.2}}, color={0,0,127}));
  connect(steamTurbine2_Cst, steamTurbine2.Cst) annotation (Line(points={{14,-12},
          {16.8,-12},{16.8,-7.3}}, color={0,0,127}));
  connect(steamTurbine2_eta_is, steamTurbine2.eta_is) annotation (Line(points={{
          20,-14},{20,-12},{19.6,-12},{19.6,-8.2}}, color={0,0,127}));
  connect(const4.y, temperatureSensor1.T_sensor) annotation (Line(points={{-15.8,
          8},{-15.8,4},{-21,4},{-21,2}}, color={0,0,127}));
  connect(const5.y, pressureSensor.P_sensor) annotation (Line(points={{-5.8,8},{
          -10,8},{-10,2},{-13,2}}, color={0,0,127}));
  connect(const6.y, pressureSensor1.P_sensor)
    annotation (Line(points={{48.2,6},{44,6},{44,2},{41,2}}, color={0,0,127}));
  connect(const7.y, powerSensor.W_sensor) annotation (Line(points={{76.2,46},{78,
          46},{78,42},{72,42}}, color={0,0,127}));
  connect(const8.y, steamExtractionSplitter.alpha)
    annotation (Line(points={{8.2,8},{8.4,4},{8.4,0.92}}, color={0,0,127}));
  annotation (uses(MetroscopeModelingLibrary(version="4.0.0-beta"), Modelica(
          version="4.1.0")));
end Steam_turbine_extraction_exercise;
