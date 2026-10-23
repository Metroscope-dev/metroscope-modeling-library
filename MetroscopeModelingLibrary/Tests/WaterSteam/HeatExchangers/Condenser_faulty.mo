within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Condenser_faulty
  "MML4 isolated Condenser faulty test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // MML4 interfaces declare quantities; isolated component at illustrative starts.
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser(
    S=10000,
    water_height=2,
    faulty=true,
    Q_hot_0=150,
    Q_cold_0=7986,
    Psat_0=5151.695,
    P_cold_in_0=500000,
    P_cold_out_0=500000,
    T_cold_in_0=288.15,
    T_cold_out_0=297.8485,
    h_hot_in_0=2300000,
    h_cold_in_0=63375,
    h_cold_out_0=103944)
    annotation (Placement(transformation(extent={{-50,-50},{50,50}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source steam_source(
    P_0=5151.695,
    Q_0=150,
    h_0=2300000,
    T_0=306.55)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,180})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor steam_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=150,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=5151.695,
    Q_0=150,
    h_0=2300000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,145})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition steam_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={30,145}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor steam_pressure_sensor(
    sensor_function="Calibration",
    causality="condenser_Kth",
    show_causality=true,
    P_start=0.05151695013046,
    signal_unit="barA",
    display_unit="barA",
    P_0=5151.695013046,
    Q_0=150,
    h_0=2300000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,105})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable steam_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={30,105}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink condensate_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={0,-95})));
  input MetroscopeModelingLibrary.Utilities.Units.SpecificEnthalpy steam_enthalpy(start=2300000);
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cooling_source(
    P_0=500000,
    Q_0=7986,
    h_0=63375,
    T_0=288.15)
    annotation (Placement(transformation(extent={{-20,-20},{20,20}}, rotation=180, origin={250,10})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor cw_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=5,
    signal_unit="barA",
    display_unit="barA",
    P_0=500000,
    Q_0=7986,
    h_0=63375)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}}, rotation=0, origin={200,10})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cw_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={200,40}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor cw_in_temp_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=15,
    signal_unit="degC",
    display_unit="degC",
    P_0=500000,
    Q_0=7986,
    h_0=63375,
    T_0=288.15)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}}, rotation=0, origin={160,10})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition cw_in_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={160,40}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor cw_out_temp_sensor(
    sensor_function="Calibration",
    causality="condenser_Qv_cold_in",
    show_causality=true,
    T_start=24.69846725464,
    signal_unit="degC",
    display_unit="degC",
    P_0=500000,
    Q_0=7986,
    h_0=103944,
    T_0=297.84846725464)
    annotation (Placement(transformation(extent={{-10,10},{10,-10}}, rotation=0, origin={100,-20})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable cw_out_temp
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={100,-50}), iconTransformation(extent={{100,175},{140,215}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cooling_sink
    annotation (Placement(transformation(extent={{-20,-20},{20,20}}, rotation=0, origin={200,-20})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal condenser_Kth(
    start=2500)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-85,10}), iconTransformation(extent={{-70,-290},{-50,-270}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal condenser_Qv_cold_in(
    start=8)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-85,0}), iconTransformation(extent={{-30,-290},{-10,-270}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealExpression condenser_Kfr_cold(
    y=0)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={-85,-25})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealExpression condenser_C_incond(
    y=0)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=90, origin={-85,40})));
  input MetroscopeModelingLibrary.Utilities.Units.Percentage fault_fouling(start=20);

equation
  steam_source.h_out = steam_enthalpy;
  condenser_Kth = 2500;
  condenser_Qv_cold_in = 8;
  condenser.fouling = fault_fouling;
  condenser.air_intake = 0;
  condenser.Qv_cold_in_decrease = 0;

  connect(steam_flow_sensor.Q_sensor, steam_flow)
    annotation (Line(points={{10,145},{30,145}}, color={0,0,127}));
  connect(steam_pressure_sensor.P_sensor, steam_pressure)
    annotation (Line(points={{10,105},{30,105}}, color={0,0,127}));
  connect(steam_source.C_out, steam_flow_sensor.C_in)
    annotation (Line(points={{-0,175},{0,155}}, color={28,108,200}));
  connect(steam_flow_sensor.C_out, steam_pressure_sensor.C_in)
    annotation (Line(points={{-0,135},{0,115}}, color={28,108,200}));
  connect(steam_pressure_sensor.C_out, condenser.C_hot_in)
    annotation (Line(points={{-0,95},{0,50}}, color={28,108,200}));
  connect(condenser.C_hot_out, condensate_sink.C_in)
    annotation (Line(points={{0,-50},{0,-90}}, color={28,108,200}));
  connect(cw_pressure_sensor.P_sensor, cw_pressure)
    annotation (Line(points={{200,20},{200,40}}, color={0,0,127}));
  connect(cw_in_temp_sensor.T_sensor, cw_in_temp)
    annotation (Line(points={{160,20},{160,40}}, color={0,0,127}));
  connect(cw_out_temp_sensor.T_sensor, cw_out_temp)
    annotation (Line(points={{100,-30},{100,-50}}, color={0,0,127}));
  connect(cooling_source.C_out, cw_pressure_sensor.C_in)
    annotation (Line(points={{240,10},{210,10}}, color={28,108,200}));
  connect(cw_pressure_sensor.C_out, cw_in_temp_sensor.C_in)
    annotation (Line(points={{190,10},{170,10}}, color={28,108,200}));
  connect(cw_in_temp_sensor.C_out, condenser.C_cold_in)
    annotation (Line(points={{150,10},{50,10}}, color={28,108,200}));
  connect(condenser.C_cold_out, cw_out_temp_sensor.C_in)
    annotation (Line(points={{50,-20},{90,-20}}, color={28,108,200}));
  connect(cw_out_temp_sensor.C_out, cooling_sink.C_in)
    annotation (Line(points={{110,-20},{190,-20}}, color={28,108,200}));
  connect(condenser_Kth, condenser.Kth)
    annotation (Line(points={{-85,10},{-52,10}}, color={0,0,127}));
  connect(condenser_Qv_cold_in, condenser.Qv_cold_in)
    annotation (Line(points={{-85,0},{-52,0}}, color={0,0,127}));
  connect(condenser_Kfr_cold.y, condenser.Kfr_cold)
    annotation (Line(points={{-80,-25},{-52,-25},{-52,-10}}, color={0,0,127}));
  connect(condenser_C_incond.y, condenser.C_incond)
    annotation (Line(points={{-80,40},{-52,40},{-52,20}}, color={0,0,127}));
  annotation (
    experiment(StopTime=1, Tolerance=1e-6),

    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-300,-180},{300,240}})),
    Documentation(info="<html><p>MML4 isolated component suite, 2026-10-20. Target local fix-bug-reheater HEAD 126f351e652e73df0eab808bbb7b29f9cb5e556f plus existing workspace changes, MSL 4.0.0. Approved graphical patterns: AirCompressor, MetroscopiaCCGT and MetroscopiaNPP. Native or approved icon dimensions retained.</p><p>Operating points and characteristic values are illustrative engineering sizing, not calibrated plant data. External inputs use existing sensor starts; ordinary enthalpy/fault inputs retain typed declarations and equations. Faults are constant. One Check Model and at most one simulation attempt; see the accompanying report for actual outcomes. No failed-model correction or rerun.</p></html>"));
end Condenser_faulty;
