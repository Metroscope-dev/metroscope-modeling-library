within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model LiqLiqHX_reverse
  "MML4 isolated LiqLiqHX reverse test"

  // MML4 interfaces declare quantities; isolated component at illustrative starts.
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.LiqLiqHX liqLiqHX(
    S=100,
    faulty=false,
    Q_cold_0=100,
    Q_hot_0=50,
    P_cold_in_0=2000000,
    P_cold_out_0=1980000,
    P_hot_in_0=5000000,
    P_hot_out_0=4980000,
    T_cold_in_0=323.15,
    T_cold_out_0=328.20437217175,
    T_hot_in_0=373.15,
    T_hot_out_0=363.1141371217,
    h_cold_in_0=211049.93714044,
    h_cold_out_0=232152.8732834,
    h_hot_in_0=422781.87511926,
    h_hot_out_0=380576.00283333)
    annotation (Placement(transformation(extent={{-48,-24},{48,24}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source(
    P_0=2000000,
    Q_0=100,
    h_0=211049.93714044,
    T_0=323.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-260,0})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor cold_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=20,
    signal_unit="barA",
    display_unit="barA",
    P_0=2000000,
    Q_0=100,
    h_0=211049.93714044)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-220,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-220,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor cold_temp_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=50,
    signal_unit="degC",
    display_unit="degC",
    P_0=2000000,
    Q_0=100,
    h_0=211049.93714044,
    T_0=323.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-180,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-180,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor cold_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=100,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=2000000,
    Q_0=100,
    h_0=211049.93714044)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-140,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-140,30}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor cold_out_pressure_sensor(
    sensor_function="Calibration",
    causality="liqLiqHX_Kfr_cold",
    show_causality=true,
    P_start=19.8,
    signal_unit="barA",
    display_unit="barA",
    P_0=1980000,
    Q_0=100,
    h_0=232152.8732834)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={90,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput cold_out_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={90,30}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor cold_out_temp_sensor(
    sensor_function="Calibration",
    causality="liqLiqHX_Kth",
    show_causality=true,
    T_start=55.054372171748,
    signal_unit="degC",
    display_unit="degC",
    P_0=1980000,
    Q_0=100,
    h_0=232152.8732834,
    T_0=328.20437217175)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={140,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput cold_out_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={140,30}), iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={200,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source(
    P_0=5000000,
    Q_0=50,
    h_0=422781.87511926,
    T_0=373.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,200})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor hot_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=50,
    signal_unit="barA",
    display_unit="barA",
    P_0=5000000,
    Q_0=50,
    h_0=422781.87511926)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={30,160}), iconTransformation(extent={{-140,-5},{-100,35}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor hot_temp_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=100,
    signal_unit="degC",
    display_unit="degC",
    P_0=5000000,
    Q_0=50,
    h_0=422781.87511926,
    T_0=373.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={30,120}), iconTransformation(extent={{-140,-50},{-100,-10}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor hot_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=50,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=5000000,
    Q_0=50,
    h_0=422781.87511926)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,80})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={30,80}), iconTransformation(extent={{-140,-95},{-100,-55}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor hot_out_pressure_sensor(
    sensor_function="Calibration",
    causality="liqLiqHX_Kfr_hot",
    show_causality=true,
    P_start=49.8,
    signal_unit="barA",
    display_unit="barA",
    P_0=4980000,
    Q_0=50,
    h_0=380576.00283333)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,-60})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput hot_out_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={30,-60}), iconTransformation(extent={{-140,-140},{-100,-100}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,-105})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal liqLiqHX_Kth(
    start=500)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-30,55}), iconTransformation(extent={{-70,-290},{-50,-270}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal liqLiqHX_Kfr_hot(
    start=7685.0806346343)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={65,55}), iconTransformation(extent={{-30,-290},{-10,-270}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal liqLiqHX_Kfr_cold(
    start=1977.7433045201)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-85,12}), iconTransformation(extent={{10,-290},{30,-270}})));

equation


  connect(cold_pressure_sensor.P_sensor, cold_pressure)
    annotation (Line(points={{-220,10},{-220,30}}, color={0,0,127}));
  connect(cold_temp_sensor.T_sensor, cold_temp)
    annotation (Line(points={{-180,10},{-180,30}}, color={0,0,127}));
  connect(cold_flow_sensor.Q_sensor, cold_flow)
    annotation (Line(points={{-140,10},{-140,30}}, color={0,0,127}));
  connect(cold_out_pressure_sensor.P_sensor, cold_out_pressure)
    annotation (Line(points={{90,10},{90,30}}, color={0,0,127}));
  connect(cold_out_temp_sensor.T_sensor, cold_out_temp)
    annotation (Line(points={{140,10},{140,30}}, color={0,0,127}));
  connect(cold_source.C_out, cold_pressure_sensor.C_in)
    annotation (Line(points={{-255,0},{-230,0}}, color={28,108,200}));
  connect(cold_pressure_sensor.C_out, cold_temp_sensor.C_in)
    annotation (Line(points={{-210,0},{-190,0}}, color={28,108,200}));
  connect(cold_temp_sensor.C_out, cold_flow_sensor.C_in)
    annotation (Line(points={{-170,0},{-150,0}}, color={28,108,200}));
  connect(cold_flow_sensor.C_out, liqLiqHX.C_cold_in)
    annotation (Line(points={{-130,0},{-48.6,0}}, color={28,108,200}));
  connect(liqLiqHX.C_cold_out, cold_out_pressure_sensor.C_in)
    annotation (Line(points={{48,0},{80,0}}, color={28,108,200}));
  connect(cold_out_pressure_sensor.C_out, cold_out_temp_sensor.C_in)
    annotation (Line(points={{100,0},{130,0}}, color={28,108,200}));
  connect(cold_out_temp_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{150,0},{195,0}}, color={28,108,200}));
  connect(hot_pressure_sensor.P_sensor, hot_pressure)
    annotation (Line(points={{10,160},{30,160}}, color={0,0,127}));
  connect(hot_temp_sensor.T_sensor, hot_temp)
    annotation (Line(points={{10,120},{30,120}}, color={0,0,127}));
  connect(hot_flow_sensor.Q_sensor, hot_flow)
    annotation (Line(points={{10,80},{30,80}}, color={0,0,127}));
  connect(hot_out_pressure_sensor.P_sensor, hot_out_pressure)
    annotation (Line(points={{10,-60},{30,-60}}, color={0,0,127}));
  connect(hot_source.C_out, hot_pressure_sensor.C_in)
    annotation (Line(points={{-0,195},{0,170}}, color={28,108,200}));
  connect(hot_pressure_sensor.C_out, hot_temp_sensor.C_in)
    annotation (Line(points={{-0,150},{0,130}}, color={28,108,200}));
  connect(hot_temp_sensor.C_out, hot_flow_sensor.C_in)
    annotation (Line(points={{-0,110},{0,90}}, color={28,108,200}));
  connect(hot_flow_sensor.C_out, liqLiqHX.C_hot_in)
    annotation (Line(points={{-0,70},{0,24}}, color={28,108,200}));
  connect(liqLiqHX.C_hot_out, hot_out_pressure_sensor.C_in)
    annotation (Line(points={{0,-24},{0,-50}}, color={28,108,200}));
  connect(hot_out_pressure_sensor.C_out, hot_sink.C_in)
    annotation (Line(points={{-0,-70},{0,-100}}, color={28,108,200}));
  connect(liqLiqHX_Kth, liqLiqHX.Kth)
    annotation (Line(points={{-30,55},{-30,25.2}}, color={0,0,127}));
  connect(liqLiqHX_Kfr_hot, liqLiqHX.Kfr_hot)
    annotation (Line(points={{65,55},{30,55},{30,25.2}}, color={0,0,127}));
  connect(liqLiqHX_Kfr_cold, liqLiqHX.Kfr_cold)
    annotation (Line(points={{-85,12},{-49.2,12}}, color={0,0,127}));
  annotation (
    experiment(StopTime=1, Tolerance=1e-6),
    Icon(coordinateSystem(extent={{-140,-300},{140,300}}), graphics={
      Rectangle(extent={{-100,-260},{100,260}}, lineColor={28,108,200}),
      Text(extent={{-95,20},{95,-20}}, textString="%name")}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-300,-180},{300,240}})),
    Documentation(info="<html><p>MML4 isolated component suite, 2026-10-20. Target local fix-bug-reheater HEAD 126f351e652e73df0eab808bbb7b29f9cb5e556f plus existing workspace changes, MSL 4.0.0. Approved graphical patterns: AirCompressor, MetroscopiaCCGT and MetroscopiaNPP. Native or approved icon dimensions retained.</p><p>Operating points and characteristic values are illustrative engineering sizing, not calibrated plant data. External inputs use existing sensor starts; ordinary enthalpy/fault inputs retain typed declarations and equations. Faults are constant. One Check Model and at most one simulation attempt; see the accompanying report for actual outcomes. No failed-model correction or rerun.</p></html>"));
end LiqLiqHX_reverse;
