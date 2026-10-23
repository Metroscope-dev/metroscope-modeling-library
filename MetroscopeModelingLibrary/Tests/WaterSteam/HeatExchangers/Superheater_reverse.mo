within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Superheater_reverse
  "MML4 isolated Superheater reverse test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // MML4 interfaces declare quantities; isolated component at illustrative starts.
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Superheater superheater(
    S=100,
    Q_vent=1,
    faulty=false,
    Q_cold_0=1300,
    Q_hot_0=107.59474067011,
    P_cold_in_0=1100000,
    P_cold_out_0=1100000,
    P_hot_in_0=6000000,
    P_hot_out_0=6000000,
    T_cold_in_0=457.21967567795,
    T_cold_out_0=497.15,
    T_hot_in_0=551.73641075605,
    T_hot_out_0=548.73641075605,
    h_cold_in_0=2750000,
    h_cold_out_0=2880078.3011153,
    h_hot_in_0=2800000,
    h_hot_out_0=1213731.0823707)
    annotation (Placement(transformation(extent={{-48,-24},{48,24}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source(
    P_0=6000000,
    Q_0=107.59474067011,
    h_0=2800000,
    T_0=551.73641075605)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-150,0})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor hot_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=60,
    signal_unit="barA",
    display_unit="barA",
    P_0=6000000,
    Q_0=107.59474067011,
    h_0=2800000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-100,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition hot_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-100,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink drains_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source(
    P_0=1100000,
    Q_0=1300,
    h_0=2750000,
    T_0=457.21967567795)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,-150})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor cold_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=11,
    signal_unit="barA",
    display_unit="barA",
    P_0=1100000,
    Q_0=1300,
    h_0=2750000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,-110})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-30,-110}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor cold_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=1300,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=1100000,
    Q_0=1300,
    h_0=2750000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,-70})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cold_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-30,-70}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor cold_out_temp_sensor(
    sensor_function="Calibration",
    causality="superheater_Kth",
    show_causality=true,
    T_start=224,
    signal_unit="degC",
    display_unit="degC",
    P_0=1100000,
    Q_0=1300,
    h_0=2880078.3011153,
    T_0=497.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,90})));
  MetroscopeModelingLibrary.Utilities.Interfaces.CalibrationInput cold_out_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-30,90}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,145})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink vent_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,-24})));
  input MetroscopeModelingLibrary.Utilities.Units.SpecificEnthalpy cold_enthalpy(start=2750000);
  input MetroscopeModelingLibrary.Utilities.Units.SpecificEnthalpy hot_enthalpy(start=2800000);
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal superheater_Kth(
    start=17633.485399613)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-36,55}), iconTransformation(extent={{-70,-290},{-50,-270}})));

equation
  cold_source.h_out = cold_enthalpy;
  hot_source.h_out = hot_enthalpy;

  connect(hot_pressure_sensor.P_sensor, hot_pressure)
    annotation (Line(points={{-100,10},{-100,30}}, color={0,0,127}));
  connect(hot_source.C_out, hot_pressure_sensor.C_in)
    annotation (Line(points={{-145,0},{-110,0}}, color={28,108,200}));
  connect(hot_pressure_sensor.C_out, superheater.C_hot_in)
    annotation (Line(points={{-90,0},{-48,0}}, color={28,108,200}));
  connect(superheater.C_hot_out, drains_sink.C_in)
    annotation (Line(points={{48,0},{125,0}}, color={28,108,200}));
  connect(cold_pressure_sensor.P_sensor, cold_pressure)
    annotation (Line(points={{-10,-110},{-30,-110}}, color={0,0,127}));
  connect(cold_flow_sensor.Q_sensor, cold_flow)
    annotation (Line(points={{-10,-70},{-30,-70}}, color={0,0,127}));
  connect(cold_out_temp_sensor.T_sensor, cold_out_temp)
    annotation (Line(points={{-10,90},{-30,90}}, color={0,0,127}));
  connect(cold_source.C_out, cold_pressure_sensor.C_in)
    annotation (Line(points={{0,-145},{-0,-120}}, color={28,108,200}));
  connect(cold_pressure_sensor.C_out, cold_flow_sensor.C_in)
    annotation (Line(points={{0,-100},{-0,-80}}, color={28,108,200}));
  connect(cold_flow_sensor.C_out, superheater.C_cold_in)
    annotation (Line(points={{0,-60},{0,-24}}, color={28,108,200}));
  connect(superheater.C_cold_out, cold_out_temp_sensor.C_in)
    annotation (Line(points={{0,24},{-0,80}}, color={28,108,200}));
  connect(cold_out_temp_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{0,100},{-0,140}}, color={28,108,200}));
  connect(superheater.C_vent, vent_sink.C_in)
    annotation (Line(points={{48,-24},{125,-24}}, color={28,108,200}));
  connect(superheater_Kth, superheater.Kth)
    annotation (Line(points={{-36,55},{-36,25.2}}, color={0,0,127}));
  annotation (
    experiment(StopTime=1, Tolerance=1e-6),

    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-300,-180},{300,240}})),
    Documentation(info="<html><p>MML4 isolated component suite, 2026-10-20. Target local fix-bug-reheater HEAD 126f351e652e73df0eab808bbb7b29f9cb5e556f plus existing workspace changes, MSL 4.0.0. Approved graphical patterns: AirCompressor, MetroscopiaCCGT and MetroscopiaNPP. Native or approved icon dimensions retained.</p><p>Operating points and characteristic values are illustrative engineering sizing, not calibrated plant data. External inputs use existing sensor starts; ordinary enthalpy/fault inputs retain typed declarations and equations. Faults are constant. One Check Model and at most one simulation attempt; see the accompanying report for actual outcomes. No failed-model correction or rerun.</p></html>"));
end Superheater_reverse;
