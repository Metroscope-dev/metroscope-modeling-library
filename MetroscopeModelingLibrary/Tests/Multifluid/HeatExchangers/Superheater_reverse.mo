within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Superheater_reverse
  "MML4 isolated Superheater reverse test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.Superheater superheater(
    faulty=false,
    QCp_max_side="hot")
    annotation (Placement(transformation(extent={{-50,-50},{50,50}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source(
    P_0=13000000,
    Q_0=85,
    T_0=683.15,
    h_0=3400000)
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
    h_0=3400000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,-90}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor cold_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=410,
    signal_unit="degC",
    display_unit="degC",
    P_0=13000000,
    Q_0=85,
    h_0=3400000,
    T_0=683.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,-90}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor cold_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=85,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=13000000,
    Q_0=85,
    h_0=3400000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,-90}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source(
    P_0=110000,
    Q_0=640,
    T_0=873.15,
    h_0=960465)
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
    h_0=960465)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,190}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor hot_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=600,
    signal_unit="degC",
    display_unit="degC",
    P_0=110000,
    Q_0=640,
    h_0=960465,
    T_0=873.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,190}), iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.FlowSensor hot_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=640,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=110000,
    Q_0=640,
    h_0=960465)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,190}), iconTransformation(extent={{-140,-5},{-100,35}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal superheater_Kfr_cold(
    start=2740.4844290657)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-60,-50})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal superheater_Kth(
    start=50)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-60,60})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor cold_sink_pressure_sensor(
    sensor_function="Calibration",
    causality="superheater_Kfr_cold",
    show_causality=true,
    P_start=129.8,
    signal_unit="barA",
    display_unit="barA",
    P_0=12980000,
    Q_0=85,
    h_0=2163000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput cold_sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,-90}), iconTransformation(extent={{-140,-50},{-100,-10}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor cold_sink_temperature_sensor(
    sensor_function="Calibration",
    causality="superheater_Kth",
    show_causality=true,
    T_start=515,
    signal_unit="degC",
    display_unit="degC",
    P_0=13000000,
    Q_0=85,
    h_0=2163000,
    T_0=788.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput cold_sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={130,-90}), iconTransformation(extent={{-140,-95},{-100,-55}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={220,-120})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={120,160})));
equation
  hot_source.Xi_out = {0.768,0.232,0,0,0};
  connect(cold_source_pressure_sensor.P_sensor, cold_source_pressure)
    annotation (Line(points={{-240,-110},{-240,-90}}, color={0,0,127}));
  connect(cold_source.C_out, cold_source_pressure_sensor.C_in)
    annotation (Line(points={{-275,-120},{-250,-120}}, color={28,108,200}));
  connect(cold_source_temperature_sensor.T_sensor, cold_source_temperature)
    annotation (Line(points={{-200,-110},{-200,-90}}, color={0,0,127}));
  connect(cold_source_pressure_sensor.C_out, cold_source_temperature_sensor.C_in)
    annotation (Line(points={{-230,-120},{-210,-120}}, color={28,108,200}));
  connect(cold_source_flow_sensor.Q_sensor, cold_source_flow)
    annotation (Line(points={{-160,-110},{-160,-90}}, color={0,0,127}));
  connect(cold_source_temperature_sensor.C_out, cold_source_flow_sensor.C_in)
    annotation (Line(points={{-190,-120},{-170,-120}}, color={28,108,200}));
  connect(cold_source_flow_sensor.C_out, superheater.C_cold_in)
    annotation (Line(points={{-150,-120},{0,-120},{0,-50}}, color={28,108,200}));
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
  connect(hot_source_flow_sensor.C_out, superheater.C_hot_in)
    annotation (Line(points={{-150,160},{-20,160},{-20,0}}, color={28,108,200}));
  connect(superheater_Kfr_cold, superheater.Kfr_cold)
    annotation (Line(points={{-60,-50},{-22,-50},{-22,40}}, color={0,0,127}));
  connect(superheater_Kth, superheater.Kth)
    annotation (Line(points={{-60,60},{-22,60},{-22,-40}}, color={0,0,127}));
  connect(cold_sink_pressure_sensor.P_sensor, cold_sink_pressure)
    annotation (Line(points={{80,-110},{80,-90}}, color={0,0,127}));
  connect(superheater.C_cold_out, cold_sink_pressure_sensor.C_in)
    annotation (Line(points={{0,50},{70,50},{70,-120}}, color={28,108,200}));
  connect(cold_sink_temperature_sensor.T_sensor, cold_sink_temperature)
    annotation (Line(points={{130,-110},{130,-90}}, color={0,0,127}));
  connect(cold_sink_pressure_sensor.C_out, cold_sink_temperature_sensor.C_in)
    annotation (Line(points={{90,-120},{120,-120}}, color={28,108,200}));
  connect(cold_sink_temperature_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{140,-120},{215,-120}}, color={28,108,200}));
  connect(superheater.C_hot_out, hot_sink.C_in)
    annotation (Line(points={{20,0},{115,0},{115,160}}, color={28,108,200}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end Superheater_reverse;
