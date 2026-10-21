within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Evaporator_direct
  "MML4 isolated Evaporator direct test"
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Evaporator evaporator(
    faulty=false,
    S=50000)
    annotation (Placement(transformation(extent={{-50,-50},{50,140}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source(
    P_0=13000000,
    Q_0=85,
    T_0=599.15,
    h_0=2700000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,-120})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor cold_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=130,
    signal_unit="barA",
    display_unit="barA",
    P_0=13000000,
    Q_0=85,
    h_0=2700000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,-90}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor cold_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=326,
    signal_unit="degC",
    display_unit="degC",
    P_0=13000000,
    Q_0=85,
    h_0=2700000,
    T_0=599.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,-90}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor steam_flow_sensor(
    sensor_function="Calibration",
    causality="evaporator_Kth",
    show_causality=true,
    Q_start=85,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=13000000,
    Q_0=85,
    h_0=2700000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={70,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable steam_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={70,-90}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source(
    P_0=110000,
    Q_0=640,
    T_0=748.15,
    h_0=822965)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,160})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor hot_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=1.1,
    signal_unit="barA",
    display_unit="barA",
    P_0=110000,
    Q_0=640,
    h_0=822965)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,190}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor hot_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=475,
    signal_unit="degC",
    display_unit="degC",
    P_0=110000,
    Q_0=640,
    h_0=822965,
    T_0=748.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,190}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.FlowSensor hot_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=640,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=110000,
    Q_0=640,
    h_0=822965)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,190}), iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal evaporator_Kth(
    start=50)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-60,60})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor cold_sink_temperature_sensor(
    sensor_function="Unidentified",
    causality="",
    show_causality=true,
    T_start=330.85,
    signal_unit="degC",
    display_unit="degC",
    P_0=13000000,
    Q_0=85,
    h_0=1389570,
    T_0=604)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable cold_sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={130,-90}), iconTransformation(extent={{100,175},{140,215}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={220,-120})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor hot_sink_temperature_sensor(
    sensor_function="Unidentified",
    causality="",
    show_causality=true,
    T_start=345,
    signal_unit="degC",
    display_unit="degC",
    P_0=110000,
    Q_0=640,
    h_0=500000,
    T_0=618.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable hot_sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,190}), iconTransformation(extent={{100,130},{140,170}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,160})));
equation
  hot_source.Xi_out = {0.768,0.232,0,0,0};
  evaporator_Kth = 50;
  connect(cold_source_pressure_sensor.P_sensor, cold_source_pressure)
    annotation (Line(points={{-240,-110},{-240,-90}}, color={0,0,127}));
  connect(cold_source.C_out, cold_source_pressure_sensor.C_in)
    annotation (Line(points={{-275,-120},{-250,-120}}, color={28,108,200}));
  connect(cold_source_temperature_sensor.T_sensor, cold_source_temperature)
    annotation (Line(points={{-200,-110},{-200,-90}}, color={0,0,127}));
  connect(cold_source_pressure_sensor.C_out, cold_source_temperature_sensor.C_in)
    annotation (Line(points={{-230,-120},{-210,-120}}, color={28,108,200}));
  connect(cold_source_temperature_sensor.C_out, evaporator.C_cold_in)
    annotation (Line(points={{-190,-120},{35,-120},{35,80}}, color={28,108,200}));
  connect(steam_flow_sensor.Q_sensor, steam_flow)
    annotation (Line(points={{70,-110},{70,-90}}, color={0,0,127}));
  connect(evaporator.C_cold_out, steam_flow_sensor.C_in)
    annotation (Line(points={{-35,120},{60,120},{60,-120}}, color={28,108,200}));
  connect(hot_source_pressure_sensor.P_sensor, hot_source_pressure)
    annotation (Line(points={{-240,170},{-240,190}}, color={0,0,127}));
  connect(hot_source.C_out, hot_source_pressure_sensor.C_in)
    annotation (Line(points={{-275,160},{-250,160}}, color={28,108,200}));
  connect(hot_source_temperature_sensor.T_sensor, hot_source_temperature)
    annotation (Line(points={{-200,170},{-200,190}}, color={0,0,127}));
  connect(hot_source_pressure_sensor.C_out, hot_source_temperature_sensor.C_in)
    annotation (Line(points={{-230,160},{-210,160}}, color={28,108,200}));
  connect(hot_source_flow_sensor.Q_sensor, hot_source_flow)
    annotation (Line(points={{-160,170},{-160,190}}, color={0,0,127}));
  connect(hot_source_temperature_sensor.C_out, hot_source_flow_sensor.C_in)
    annotation (Line(points={{-190,160},{-170,160}}, color={28,108,200}));
  connect(hot_source_flow_sensor.C_out, evaporator.C_hot_in)
    annotation (Line(points={{-150,160},{-40,160},{-40,0}}, color={28,108,200}));
  connect(evaporator_Kth, evaporator.Kth)
    annotation (Line(points={{-60,60},{-42,60},{-42,-40}}, color={0,0,127}));
  connect(cold_sink_temperature_sensor.T_sensor, cold_sink_temperature)
    annotation (Line(points={{130,-110},{130,-90}}, color={0,0,127}));
  connect(steam_flow_sensor.C_out, cold_sink_temperature_sensor.C_in)
    annotation (Line(points={{80,-120},{120,-120}}, color={28,108,200}));
  connect(cold_sink_temperature_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{140,-120},{215,-120}}, color={28,108,200}));
  connect(hot_sink_temperature_sensor.T_sensor, hot_sink_temperature)
    annotation (Line(points={{80,170},{80,190}}, color={0,0,127}));
  connect(evaporator.C_hot_out, hot_sink_temperature_sensor.C_in)
    annotation (Line(points={{40,0},{70,0},{70,160}}, color={28,108,200}));
  connect(hot_sink_temperature_sensor.C_out, hot_sink.C_in)
    annotation (Line(points={{90,160},{165,160}}, color={28,108,200}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end Evaporator_direct;
