within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model OnceThroughEvaporator_direct "Isolated direct test of a once-through HRSG evaporator"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.OnceThroughEvaporator evaporator(S=15000)
    annotation (Placement(transformation(extent={{-50,-50},{14,140}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={250,180}, rotation=180)));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor cold_pressure_sensor(
    sensor_function="BC",
    causality="",
    P_start=130,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={210,180}, rotation=180)));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor cold_temp_sensor(
    sensor_function="BC",
    causality="",
    T_start=305,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={170,180}, rotation=180)));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor cold_flow_sensor(
    sensor_function="BC",
    causality="",
    Q_start=85,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={130,180}, rotation=180)));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor cold_out_pressure_sensor(
    sensor_function="Calibration",
    causality="evaporator_Kfr_cold",
    P_start=129.5,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-115,180}, rotation=180)));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-160,180}, rotation=180)));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-250,-110})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor hot_pressure_sensor(
    sensor_function="BC",
    causality="",
    P_start=1.1,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-210,-110})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor hot_temp_sensor(
    sensor_function="BC",
    causality="",
    T_start=540,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-170,-110})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.FlowSensor hot_flow_sensor(
    sensor_function="BC",
    causality="",
    Q_start=600,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-130,-110})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor hot_out_temp_sensor(
    T_start=398,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={115,-110})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={160,-110})));

  parameter Units.MassFraction hotComposition[FlueGasesMedium.nXi] =
    {0.752,0.135,0.052,0.061,0};

  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={210,150}),
      iconTransformation(extent={{-140,150},{-100,190}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={170,150}),
      iconTransformation(extent={{-140,95},{-100,135}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={130,150}),
      iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable cold_out_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-115,150}),
      iconTransformation(extent={{100,40},{140,80}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-210,-80}),
      iconTransformation(extent={{-140,-15},{-100,25}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-170,-80}),
      iconTransformation(extent={{-140,-70},{-100,-30}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_flow(start=hot_flow_sensor.Q_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-130,-80}),
      iconTransformation(extent={{-140,-125},{-100,-85}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable hot_out_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={115,-80}),
      iconTransformation(extent={{100,-15},{140,25}})));
  Modelica.Blocks.Math.Add hot_flow_command
    annotation (Placement(transformation(extent={{-8,-8},{8,8}}, origin={-130,-55})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal evaporator_Kfr_cold
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={65,44})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal evaporator_Kth(
    unit="W/(m2.K)", start=70)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-42,-72}, rotation=90)));
  SensorsControl.WaterSteam.TemperatureSensor cold_temp_out_sensor(
    sensor_function="Calibration",
    causality="\"\"evaporator_Kth\"",
    T_start=340,
    signal_unit="degC",
    display_unit="degC") annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={-88,180},
        rotation=180)));
  Utilities.Interfaces.Observable cold_temp_out annotation (Placement(
        transformation(extent={{-4,-4},{4,4}}, origin={-88,150}),
        iconTransformation(extent={{-140,95},{-100,135}})));
equation
  hot_source.Xi_out = hotComposition;
  evaporator_Kfr_cold = 4917.32861328125;
  evaporator_Kth = 43.345829010009766;
  hot_flow_command.u2 = 500*time;

  connect(cold_source.C_out, cold_pressure_sensor.C_in)
    annotation (Line(points={{245,180},{220,180}}, color={28,108,200}));
  connect(cold_pressure_sensor.C_out, cold_temp_sensor.C_in)
    annotation (Line(points={{200,180},{180,180}}, color={28,108,200}));
  connect(cold_temp_sensor.C_out, cold_flow_sensor.C_in)
    annotation (Line(points={{160,180},{140,180}}, color={28,108,200}));
  connect(cold_flow_sensor.C_out, evaporator.C_cold_in)
    annotation (Line(points={{120,180},{76,180},{76,44},{28,44},{28,-62},{-18,
          -62},{-18,-48.1}},                                      color={28,108,200}));
  connect(cold_out_pressure_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{-125,180},{-155,180}}, color={28,108,200}));
  connect(hot_source.C_out, hot_pressure_sensor.C_in)
    annotation (Line(points={{-245,-110},{-220,-110}}, color={95,95,95}));
  connect(hot_pressure_sensor.C_out, hot_temp_sensor.C_in)
    annotation (Line(points={{-200,-110},{-180,-110}}, color={95,95,95}));
  connect(hot_temp_sensor.C_out, hot_flow_sensor.C_in)
    annotation (Line(points={{-160,-110},{-140,-110}}, color={95,95,95}));
  connect(hot_flow_sensor.C_out, evaporator.C_hot_in)
    annotation (Line(points={{-120,-110},{-66,-110},{-66,45},{-48.4,45}},
                                                                      color={95,95,95}));
  connect(evaporator.C_hot_out, hot_out_temp_sensor.C_in)
    annotation (Line(points={{14,45},{56,45},{56,-110},{105,-110}},
                                                                  color={95,95,95}));
  connect(hot_out_temp_sensor.C_out, hot_sink.C_in)
    annotation (Line(points={{125,-110},{155,-110}}, color={95,95,95}));

  connect(cold_pressure_sensor.P_sensor, cold_pressure)
    annotation (Line(points={{210,170},{210,150}}, color={0,0,127}));
  connect(cold_temp_sensor.T_sensor, cold_temp)
    annotation (Line(points={{170,170},{170,150}}, color={0,0,127}));
  connect(cold_flow_sensor.Q_sensor, cold_flow)
    annotation (Line(points={{130,170},{130,150}}, color={0,0,127}));
  connect(cold_out_pressure_sensor.P_sensor, cold_out_pressure)
    annotation (Line(points={{-115,170},{-115,150}}, color={0,0,127}));
  connect(hot_pressure_sensor.P_sensor, hot_pressure)
    annotation (Line(points={{-210,-100},{-210,-80}}, color={0,0,127}));
  connect(hot_temp_sensor.T_sensor, hot_temp)
    annotation (Line(points={{-170,-100},{-170,-80}}, color={0,0,127}));
  connect(hot_flow, hot_flow_command.u1)
    annotation (Line(points={{-130,-80},{-150,-80},{-150,-50.2},{-139.6,-50.2}}, color={0,0,127}));
  connect(hot_flow_command.y, hot_flow_sensor.Q_sensor)
    annotation (Line(points={{-121.2,-55},{-118,-55},{-118,-94},{-130,-94},{-130,-100}}, color={0,0,127}));
  connect(hot_out_temp_sensor.T_sensor, hot_out_temp)
    annotation (Line(points={{115,-100},{115,-80}}, color={0,0,127}));
  connect(evaporator_Kfr_cold, evaporator.Kfr_cold)
    annotation (Line(points={{65,44},{65,154},{-64,154},{-64,121},{-53.2,121}},
                                               color={0,0,127}));
  connect(evaporator_Kth, evaporator.Kth)
    annotation (Line(points={{-42,-72},{-42,-56},{-64,-56},{-64,-31},{-53.2,-31}},
                                                   color={0,0,127}));
  connect(evaporator.C_cold_out, cold_temp_out_sensor.C_in) annotation (Line(
        points={{-18,140},{-18,180},{-78,180}}, color={28,108,200}));
  connect(cold_temp_out_sensor.C_out, cold_out_pressure_sensor.C_in)
    annotation (Line(points={{-98,180},{-105,180}}, color={28,108,200}));
  connect(cold_temp_out, cold_temp_out_sensor.T_sensor)
    annotation (Line(points={{-88,150},{-88,170}}, color={28,108,200}));
  annotation (
    experiment(StopTime=1, Tolerance=1e-6),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-280,-155},{280,215}})),
    Documentation(info="<html><p>Illustrative counterflow HRSG point. The exchange area, wet flue-gas composition and 0.5 bar cold-side pressure drop are illustrative assumptions. The hot-side flow command adds a 500 kg/s per second ramp to the external hot_flow input. The fixed coefficients come from the Dymola 2026x simulation of OnceThroughEvaporator_reverse at this point.</p><p>MML4 source interface inspected at commit 10344cb, Modelica Standard Library 4.1.0.</p></html>"));
end OnceThroughEvaporator_direct;
