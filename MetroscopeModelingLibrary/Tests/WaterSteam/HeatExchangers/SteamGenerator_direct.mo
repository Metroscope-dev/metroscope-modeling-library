within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model SteamGenerator_direct
  "MML4 isolated SteamGenerator direct test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // MML4 interfaces declare quantities; isolated component at illustrative starts.
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.SteamGenerator steamGenerator(
    faulty=false,
    steam_source(P_0=70e5, Q_0=502.81254303236, h_0=2757517.8090694, T_0=558.98002280575))
    annotation (Placement(transformation(extent={{-42,-84},{42,84}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source feedwater_source(
    P_0=8000000,
    Q_0=503.81254303236,
    h_0=968181.49130068,
    T_0=498.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={230,0})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor feedwater_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=80,
    signal_unit="barA",
    display_unit="barA",
    P_0=8000000,
    Q_0=503.81254303236,
    h_0=968181.49130068)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}}, rotation=0, origin={180,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition feedwater_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={180,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor feedwater_temp_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=225,
    signal_unit="degC",
    display_unit="degC",
    P_0=8000000,
    Q_0=503.81254303236,
    h_0=968181.49130068,
    T_0=498.15)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}}, rotation=0, origin={140,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition feedwater_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={140,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor feedwater_flow_sensor(
    sensor_function="Unidentified",
    causality="",
    show_causality=true,
    Q_start=503.81254303236,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=8000000,
    Q_0=503.81254303236,
    h_0=968181.49130068)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}}, rotation=0, origin={100,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable feedwater_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={100,30}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor steam_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=70,
    signal_unit="barA",
    display_unit="barA",
    P_0=7000000,
    Q_0=502.81254303236,
    h_0=2757517.8090694)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition steam_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-30,120}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor steam_flow_sensor(
    sensor_function="Calibration",
    causality="steamGenerator_vapor_fraction",
    show_causality=true,
    Q_start=502.81254303236,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=7000000,
    Q_0=502.81254303236,
    h_0=2757517.8090694)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,160})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable steam_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-30,160}), iconTransformation(extent={{100,175},{140,215}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink steam_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={0,210})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source power_source
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-170,0})));
  MetroscopeModelingLibrary.SensorsControl.Power.PowerSensor power_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    W_start=900,
    signal_unit="MW",
    display_unit="MW")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-120,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition thermal_power
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-120,30}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal steamGenerator_vapor_fraction(
    start=0.99)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-65,56}), iconTransformation(extent={{-70,-290},{-50,-270}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor purge_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=1,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=7000000,
    Q_0=1,
    h_0=1267437.2138653)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,-120})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition purge_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={30,-120}), iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink purge_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,-165})));

equation
  steamGenerator_vapor_fraction = 0.99;

  connect(feedwater_pressure_sensor.P_sensor, feedwater_pressure)
    annotation (Line(points={{180,10},{180,30}}, color={0,0,127}));
  connect(feedwater_temp_sensor.T_sensor, feedwater_temp)
    annotation (Line(points={{140,10},{140,30}}, color={0,0,127}));
  connect(feedwater_flow_sensor.Q_sensor, feedwater_flow)
    annotation (Line(points={{100,10},{100,30}}, color={0,0,127}));
  connect(feedwater_source.C_out, feedwater_pressure_sensor.C_in)
    annotation (Line(points={{225,0},{190,0}}, color={28,108,200}));
  connect(feedwater_pressure_sensor.C_out, feedwater_temp_sensor.C_in)
    annotation (Line(points={{170,0},{150,0}}, color={28,108,200}));
  connect(feedwater_temp_sensor.C_out, feedwater_flow_sensor.C_in)
    annotation (Line(points={{130,0},{110,0}}, color={28,108,200}));
  connect(feedwater_flow_sensor.C_out, steamGenerator.feedwater_inlet)
    annotation (Line(points={{90,0},{21,0}}, color={28,108,200}));
  connect(steam_pressure_sensor.P_sensor, steam_pressure)
    annotation (Line(points={{-10,120},{-30,120}}, color={0,0,127}));
  connect(steam_flow_sensor.Q_sensor, steam_flow)
    annotation (Line(points={{-10,160},{-30,160}}, color={0,0,127}));
  connect(steamGenerator.steam_outlet, steam_pressure_sensor.C_in)
    annotation (Line(points={{0,84},{-0,110}}, color={28,108,200}));
  connect(steam_pressure_sensor.C_out, steam_flow_sensor.C_in)
    annotation (Line(points={{0,130},{-0,150}}, color={28,108,200}));
  connect(steam_flow_sensor.C_out, steam_sink.C_in)
    annotation (Line(points={{0,170},{-0,205}}, color={28,108,200}));
  connect(power_sensor.W_sensor, thermal_power)
    annotation (Line(points={{-120,10},{-120,30}}, color={0,0,127}));
  connect(power_source.C_out, power_sensor.C_in)
    annotation (Line(points={{-165.2,0},{-130,0}}, color={244,125,35}));
  connect(power_sensor.C_out, steamGenerator.C_thermal_power)
    annotation (Line(points={{-110.2,0},{-21,0}}, color={244,125,35}));
  connect(steamGenerator_vapor_fraction, steamGenerator.vapor_fraction)
    annotation (Line(points={{-65,56},{-32.2,56}}, color={0,0,127}));
  connect(purge_flow_sensor.Q_sensor, purge_flow)
    annotation (Line(points={{10,-120},{30,-120}}, color={0,0,127}));
  connect(steamGenerator.purge_outlet, purge_flow_sensor.C_in)
    annotation (Line(points={{0,-82.6},{0,-110}}, color={28,108,200}));
  connect(purge_flow_sensor.C_out, purge_sink.C_in)
    annotation (Line(points={{-0,-130},{0,-160}}, color={28,108,200}));
  annotation (
    experiment(StopTime=1, Tolerance=1e-6),

    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-300,-180},{300,240}})),
    Documentation(info="<html><p>MML4 isolated component suite, 2026-10-20. Target local fix-bug-reheater HEAD 126f351e652e73df0eab808bbb7b29f9cb5e556f plus existing workspace changes, MSL 4.0.0. Approved graphical patterns: AirCompressor, MetroscopiaCCGT and MetroscopiaNPP. Native or approved icon dimensions retained.</p><p>Operating points and characteristic values are illustrative engineering sizing, not calibrated plant data. External inputs use existing sensor starts; ordinary enthalpy/fault inputs retain typed declarations and equations. Faults are constant. One Check Model and at most one simulation attempt; see the accompanying report for actual outcomes. No failed-model correction or rerun.</p></html>"));
end SteamGenerator_direct;
