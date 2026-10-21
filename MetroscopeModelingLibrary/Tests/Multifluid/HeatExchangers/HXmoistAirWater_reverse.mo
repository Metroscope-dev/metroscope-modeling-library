within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model HXmoistAirWater_reverse
  "MML4 isolated HXmoistAirWater reverse test"
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.HXmoistAirWater hXmoistAirWater(
    QCp_max_side="cold")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source cold_source(
    P_0=100000,
    Q_0=700,
    T_0=281.15,
    h_0=28040)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,-120})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.PressureSensor cold_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=1,
    signal_unit="barA",
    display_unit="barA",
    P_0=100000,
    Q_0=700,
    h_0=28040)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,-90}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.TemperatureSensor cold_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=8,
    signal_unit="degC",
    display_unit="degC",
    P_0=100000,
    Q_0=700,
    h_0=28040,
    T_0=281.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,-90}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.FlowSensor cold_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=700,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=100000,
    Q_0=700,
    h_0=28040)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,-90}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.RelativeHumiditySensor cold_source_humidity_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    H_start=50,
    signal_unit="%",
    display_unit="%",
    P_0=100000,
    Q_0=700,
    h_0=28040)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-120,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_humidity
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-120,-90})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source(
    P_0=300000,
    Q_0=0.15,
    T_0=290.95,
    h_0=74760)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,160})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor hot_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=3,
    signal_unit="barA",
    display_unit="barA",
    P_0=300000,
    Q_0=0.15,
    h_0=74760)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,190}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor hot_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=17.8,
    signal_unit="degC",
    display_unit="degC",
    P_0=300000,
    Q_0=0.15,
    h_0=74760,
    T_0=290.95)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,190}), iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor hot_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=0.15,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=300000,
    Q_0=0.15,
    h_0=74760)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,190}), iconTransformation(extent={{-140,-5},{-100,35}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal hXmoistAirWater_Kfr_cold(
    start=0.0012755102040816)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-60,-50})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal hXmoistAirWater_Kfr_hot(
    start=755555555.55556)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={60,60})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal hXmoistAirWater_Kth(
    start=20)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-60,60})));
  MetroscopeModelingLibrary.SensorsControl.MoistAir.PressureSensor cold_sink_pressure_sensor(
    sensor_function="Calibration",
    causality="hXmoistAirWater_Kfr_cold",
    show_causality=true,
    P_start=0.995,
    signal_unit="barA",
    display_unit="barA",
    P_0=99500,
    Q_0=700,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput cold_sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,-90}), iconTransformation(extent={{-140,-50},{-100,-10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,-120})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor hot_sink_pressure_sensor(
    sensor_function="Calibration",
    causality="hXmoistAirWater_Kfr_hot",
    show_causality=true,
    P_start=2.8,
    signal_unit="barA",
    display_unit="barA",
    P_0=280000,
    Q_0=0.15,
    h_0=32760)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput hot_sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,190}), iconTransformation(extent={{-140,-95},{-100,-55}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor hot_sink_temperature_sensor(
    sensor_function="Calibration",
    causality="hXmoistAirWater_Kth",
    show_causality=true,
    T_start=7.8,
    signal_unit="degC",
    display_unit="degC",
    P_0=300000,
    Q_0=0.15,
    h_0=32760,
    T_0=280.95)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput hot_sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={130,190}), iconTransformation(extent={{-140,-140},{-100,-100}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={220,160})));
equation
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
  connect(cold_source_humidity_sensor.H_sensor, cold_source_humidity)
    annotation (Line(points={{-120,-110},{-120,-90}}, color={0,0,127}));
  connect(cold_source_flow_sensor.C_out, cold_source_humidity_sensor.C_in)
    annotation (Line(points={{-150,-120},{-130,-120}}, color={28,108,200}));
  connect(cold_source_humidity_sensor.C_out, hXmoistAirWater.C_cold_in)
    annotation (Line(points={{-110,-120},{-10,-120},{-10,0}}, color={28,108,200}));
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
  connect(hot_source_flow_sensor.C_out, hXmoistAirWater.C_hot_in)
    annotation (Line(points={{-150,160},{0,160},{0,8}}, color={28,108,200}));
  connect(hXmoistAirWater_Kfr_cold, hXmoistAirWater.Kfr_cold)
    annotation (Line(points={{-60,-50},{-11,-50},{-11,6}}, color={0,0,127}));
  connect(hXmoistAirWater_Kfr_hot, hXmoistAirWater.Kfr_hot)
    annotation (Line(points={{60,60},{11,60},{11,-6}}, color={0,0,127}));
  connect(hXmoistAirWater_Kth, hXmoistAirWater.Kth)
    annotation (Line(points={{-60,60},{-11,60},{-11,-6}}, color={0,0,127}));
  connect(cold_sink_pressure_sensor.P_sensor, cold_sink_pressure)
    annotation (Line(points={{80,-110},{80,-90}}, color={0,0,127}));
  connect(hXmoistAirWater.C_cold_out, cold_sink_pressure_sensor.C_in)
    annotation (Line(points={{10,0},{70,0},{70,-120}}, color={28,108,200}));
  connect(cold_sink_pressure_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{90,-120},{165,-120}}, color={28,108,200}));
  connect(hot_sink_pressure_sensor.P_sensor, hot_sink_pressure)
    annotation (Line(points={{80,170},{80,190}}, color={0,0,127}));
  connect(hXmoistAirWater.C_hot_out, hot_sink_pressure_sensor.C_in)
    annotation (Line(points={{0,-8},{70,-8},{70,160}}, color={28,108,200}));
  connect(hot_sink_temperature_sensor.T_sensor, hot_sink_temperature)
    annotation (Line(points={{130,170},{130,190}}, color={0,0,127}));
  connect(hot_sink_pressure_sensor.C_out, hot_sink_temperature_sensor.C_in)
    annotation (Line(points={{90,160},{120,160}}, color={28,108,200}));
  connect(hot_sink_temperature_sensor.C_out, hot_sink.C_in)
    annotation (Line(points={{140,160},{215,160}}, color={28,108,200}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end HXmoistAirWater_reverse;
