within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model LMTDFuelHeater_reverse
  "MML4 isolated LMTDFuelHeater reverse test"
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.LMTDFuelHeater lMTDFuelHeater(
    faulty=false)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source cold_source(
    P_0=3000000,
    Q_0=15,
    T_0=302.65,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,-120})));
  MetroscopeModelingLibrary.SensorsControl.Fuel.PressureSensor cold_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=30,
    signal_unit="barA",
    display_unit="barA",
    P_0=3000000,
    Q_0=15,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,-90}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.Fuel.TemperatureSensor cold_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=29.5,
    signal_unit="degC",
    display_unit="degC",
    P_0=3000000,
    Q_0=15,
    h_0=500000,
    T_0=302.65)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,-90}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.Fuel.FlowSensor cold_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=15,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=3000000,
    Q_0=15,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,-90}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source(
    P_0=4700000,
    Q_0=11,
    T_0=503.15,
    h_0=966000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,160})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor hot_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=47,
    signal_unit="barA",
    display_unit="barA",
    P_0=4700000,
    Q_0=11,
    h_0=966000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,190}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor hot_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=230,
    signal_unit="degC",
    display_unit="degC",
    P_0=4700000,
    Q_0=11,
    h_0=966000,
    T_0=503.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,190}), iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor hot_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=11,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=4700000,
    Q_0=11,
    h_0=966000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,190}), iconTransformation(extent={{-140,-5},{-100,35}})));
  output Real lMTDFuelHeater_Kfr_cold(start=1777.7777777778);
  output Real lMTDFuelHeater_Kfr_hot(start=140495.8677686);
  output Real lMTDFuelHeater_S(start=3000);
  output Real lMTDFuelHeater_Kth(start=20);
  MetroscopeModelingLibrary.SensorsControl.Fuel.PressureSensor cold_sink_pressure_sensor(
    sensor_function="Calibration",
    causality="lMTDFuelHeater_Kfr_cold",
    show_causality=true,
    P_start=29.8,
    signal_unit="barA",
    display_unit="barA",
    P_0=2980000,
    Q_0=15,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput cold_sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,-90}), iconTransformation(extent={{-140,-50},{-100,-10}})));
  MetroscopeModelingLibrary.SensorsControl.Fuel.TemperatureSensor cold_sink_temperature_sensor(
    sensor_function="Calibration",
    causality="lMTDFuelHeater_Kth",
    show_causality=true,
    T_start=97,
    signal_unit="degC",
    display_unit="degC",
    P_0=3000000,
    Q_0=15,
    h_0=500000,
    T_0=370.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput cold_sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={130,-90}), iconTransformation(extent={{-140,-95},{-100,-55}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={220,-120})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor hot_sink_pressure_sensor(
    sensor_function="Calibration",
    causality="lMTDFuelHeater_Kfr_hot",
    show_causality=true,
    P_start=46.8,
    signal_unit="barA",
    display_unit="barA",
    P_0=4680000,
    Q_0=11,
    h_0=840000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput hot_sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,190}), iconTransformation(extent={{-140,-140},{-100,-100}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,160})));
equation
  cold_source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};
  lMTDFuelHeater.Kfr_cold = lMTDFuelHeater_Kfr_cold;
  lMTDFuelHeater.Kfr_hot = lMTDFuelHeater_Kfr_hot;
  lMTDFuelHeater.S = lMTDFuelHeater_S;
  lMTDFuelHeater_S = 3000;
  lMTDFuelHeater.Kth = lMTDFuelHeater_Kth;
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
  connect(cold_source_flow_sensor.C_out, lMTDFuelHeater.C_cold_in)
    annotation (Line(points={{-150,-120},{-10,-120},{-10,0}}, color={28,108,200}));
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
  connect(hot_source_flow_sensor.C_out, lMTDFuelHeater.C_hot_in)
    annotation (Line(points={{-150,160},{4,160},{4,8}}, color={28,108,200}));
  connect(cold_sink_pressure_sensor.P_sensor, cold_sink_pressure)
    annotation (Line(points={{80,-110},{80,-90}}, color={0,0,127}));
  connect(lMTDFuelHeater.C_cold_out, cold_sink_pressure_sensor.C_in)
    annotation (Line(points={{10,0},{70,0},{70,-120}}, color={28,108,200}));
  connect(cold_sink_temperature_sensor.T_sensor, cold_sink_temperature)
    annotation (Line(points={{130,-110},{130,-90}}, color={0,0,127}));
  connect(cold_sink_pressure_sensor.C_out, cold_sink_temperature_sensor.C_in)
    annotation (Line(points={{90,-120},{120,-120}}, color={28,108,200}));
  connect(cold_sink_temperature_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{140,-120},{215,-120}}, color={28,108,200}));
  connect(hot_sink_pressure_sensor.P_sensor, hot_sink_pressure)
    annotation (Line(points={{80,170},{80,190}}, color={0,0,127}));
  connect(lMTDFuelHeater.C_hot_out, hot_sink_pressure_sensor.C_in)
    annotation (Line(points={{-4,-8},{70,-8},{70,160}}, color={28,108,200}));
  connect(hot_sink_pressure_sensor.C_out, hot_sink.C_in)
    annotation (Line(points={{90,160},{165,160}}, color={28,108,200}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p>Scalar API (no graphical component port): Kfr_cold; Scalar API (no graphical component port): Kfr_hot; Scalar API (no graphical component port): S; Scalar API (no graphical component port): Kth</p></html>"));
end LMTDFuelHeater_reverse;
