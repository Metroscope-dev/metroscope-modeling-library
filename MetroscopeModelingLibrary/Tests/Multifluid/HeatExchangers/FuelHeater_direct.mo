within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model FuelHeater_direct
  "MML4 isolated FuelHeater direct test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.FuelHeater fuelHeater(
    faulty=false,
    QCp_max_side="hot")
    annotation (Placement(transformation(extent={{-20,-20},{20,20}}, rotation=0, origin={0,0})));
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
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal fuelHeater_Kfr_cold(
    start=1777.7777777778)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-60,-50})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal fuelHeater_Kfr_hot(
    start=140495.8677686)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={60,60})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal fuelHeater_Kth(
    start=20)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-60,60})));
  MetroscopeModelingLibrary.SensorsControl.Fuel.PressureSensor cold_sink_pressure_sensor(
    sensor_function="Calibration",
    causality="fuelHeater_Kfr_cold",
    show_causality=true,
    P_start=29.8,
    signal_unit="barA",
    display_unit="barA",
    P_0=2980000,
    Q_0=15,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable cold_sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,-90}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.SensorsControl.Fuel.TemperatureSensor cold_sink_temperature_sensor(
    sensor_function="Calibration",
    causality="fuelHeater_Kth",
    show_causality=true,
    T_start=97,
    signal_unit="degC",
    display_unit="degC",
    P_0=3000000,
    Q_0=15,
    h_0=500000,
    T_0=370.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable cold_sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={130,-90}), iconTransformation(extent={{100,175},{140,215}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={220,-120})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor hot_sink_pressure_sensor(
    sensor_function="Calibration",
    causality="fuelHeater_Kfr_hot",
    show_causality=true,
    P_start=46.8,
    signal_unit="barA",
    display_unit="barA",
    P_0=4680000,
    Q_0=11,
    h_0=840000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable hot_sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,190}), iconTransformation(extent={{100,130},{140,170}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={170,160})));
equation
  cold_source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};
  fuelHeater_Kfr_cold = 1777.7777777778;
  fuelHeater_Kfr_hot = 140495.8677686;
  fuelHeater_Kth = 20;
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
  connect(cold_source_flow_sensor.C_out, fuelHeater.C_cold_in)
    annotation (Line(points={{-150,-120},{-20,-120},{-20,0}}, color={28,108,200}));
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
  connect(hot_source_flow_sensor.C_out, fuelHeater.C_hot_in)
    annotation (Line(points={{-150,160},{8,160},{8,16}}, color={28,108,200}));
  connect(fuelHeater_Kfr_cold, fuelHeater.Kfr_cold)
    annotation (Line(points={{-60,-50},{-20,-50},{-20,-8}}, color={0,0,127}));
  connect(fuelHeater_Kfr_hot, fuelHeater.Kfr_hot)
    annotation (Line(points={{60,60},{16,60},{16,12}}, color={0,0,127}));
  connect(fuelHeater_Kth, fuelHeater.Kth)
    annotation (Line(points={{-60,60},{-16,60},{-16,12}}, color={0,0,127}));
  connect(cold_sink_pressure_sensor.P_sensor, cold_sink_pressure)
    annotation (Line(points={{80,-110},{80,-90}}, color={0,0,127}));
  connect(fuelHeater.C_cold_out, cold_sink_pressure_sensor.C_in)
    annotation (Line(points={{20,0},{70,0},{70,-120}}, color={28,108,200}));
  connect(cold_sink_temperature_sensor.T_sensor, cold_sink_temperature)
    annotation (Line(points={{130,-110},{130,-90}}, color={0,0,127}));
  connect(cold_sink_pressure_sensor.C_out, cold_sink_temperature_sensor.C_in)
    annotation (Line(points={{90,-120},{120,-120}}, color={28,108,200}));
  connect(cold_sink_temperature_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{140,-120},{215,-120}}, color={28,108,200}));
  connect(hot_sink_pressure_sensor.P_sensor, hot_sink_pressure)
    annotation (Line(points={{80,170},{80,190}}, color={0,0,127}));
  connect(fuelHeater.C_hot_out, hot_sink_pressure_sensor.C_in)
    annotation (Line(points={{-8,-16},{70,-16},{70,160}}, color={28,108,200}));
  connect(hot_sink_pressure_sensor.C_out, hot_sink.C_in)
    annotation (Line(points={{90,160},{165,160}}, color={28,108,200}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end FuelHeater_direct;
