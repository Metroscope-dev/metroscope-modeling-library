within MetroscopeModelingLibrary.Examples.Nuclear.MainSteam;
model MoistureSeparatorReheater_reverse
  // Initialization parameters
  parameter Utilities.Units.PositiveMassFlowRate Q_cold_0=660 "kg/s";
  parameter Utilities.Units.PositiveMassFlowRate Q_hot_1_0=33 "kg/s";
  parameter Utilities.Units.PositiveMassFlowRate Q_hot_2_0=23 "kg/s";
  parameter Utilities.Units.Temperature T_cold_in_1_0=202 + 273.15 "K";
  parameter Utilities.Units.Temperature T_cold_in_2_0=234 + 273.15 "K";
  parameter Utilities.Units.Temperature T_hot_in_1_0=249 + 273.15 "K";
  parameter Utilities.Units.Temperature T_hot_in_2_0=282 + 273.15 "K";

  // Boundary conditions

  input Utilities.Units.SpecificEnthalpy h_cold_steam(start=2.560e6) "J/kg";
  input Utilities.Units.SpecificEnthalpy h_hot_steam_1(start=2.696e6) "J/kg";
  input Utilities.Units.SpecificEnthalpy h_hot_steam_2(start=2.769e6) "J/kg";

  // Parameters
  parameter Utilities.Units.Area S=100;

  parameter Utilities.Units.PositiveMassFlowRate Q_vent=1;
  parameter Utilities.Units.Fraction steam_dryer_x_steam_out=0.99;

  // Components
  WaterSteam.BoundaryConditions.Source hot_steam_source_1(Q_out(start=-Q_hot_1_0)) annotation (Placement(transformation(extent={{-150,-30},{-130,-10}})));
  WaterSteam.BoundaryConditions.Sink drains_1_sink annotation (Placement(transformation(extent={{70,-30},{90,-10}})));
  WaterSteam.BoundaryConditions.Source cold_steam_source annotation (Placement(transformation(extent={{-150,-70},{-130,-50}})));
  WaterSteam.BoundaryConditions.Sink superheated_steam_sink annotation (Placement(transformation(extent={{70,70},{90,90}})));
  WaterSteam.HeatExchangers.Superheater superheater_1(Q_cold_0=Q_cold_0,
    Q_hot_0=Q_hot_1_0,
    T_cold_in_0=T_cold_in_1_0,
    T_hot_in_0=T_hot_in_1_0,
    S=S,
    Q_vent=Q_vent) annotation (Placement(transformation(extent={{-16,-28},{16,-12}})));
  WaterSteam.BoundaryConditions.Sink vent_1_sink annotation (Placement(transformation(extent={{70,-50},{90,-30}})));
  SensorsControl.WaterSteam.TemperatureSensor inter_superheater_T_sensor(sensor_function="Calibration",
    causality="Kth_1",
    T_start=235,
    signal_unit="degC",
    display_unit="degC",
    T_0=273.15 + (235),
    P_0=1625000,
    Q_0=Q_cold_0,
    h_0=2880000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,8})));
  SensorsControl.WaterSteam.PressureSensor drains_1_P_sensor(sensor_function="Calibration",
    causality="Kfr_hot_1",
    P_start=39.1,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(39.1),
    Q_0=Q_hot_1_0,
    h_0=1.1e6) annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
  WaterSteam.HeatExchangers.Superheater superheater_2(Q_cold_0=Q_cold_0,
    Q_hot_0=Q_hot_2_0,
    T_cold_in_0=T_cold_in_2_0,
    T_hot_in_0=T_hot_in_2_0,
    S=S,
    Q_vent=Q_vent) annotation (Placement(transformation(extent={{-16,32},{16,48}})));
  WaterSteam.BoundaryConditions.Sink drains_2_sink annotation (Placement(transformation(extent={{70,30},{90,50}})));
  WaterSteam.BoundaryConditions.Sink vent_2_sink annotation (Placement(transformation(extent={{70,10},{90,30}})));
  SensorsControl.WaterSteam.PressureSensor drains_2_P_sensor(sensor_function="Calibration",
    causality="Kfr_hot_2",
    P_start=66.1,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(66.1),
    Q_0=Q_hot_2_0,
    h_0=1.1e6) annotation (Placement(transformation(extent={{40,30},{60,50}})));
  WaterSteam.BoundaryConditions.Source hot_steam_source_2(Q_out(start=-Q_hot_2_0)) annotation (Placement(transformation(extent={{-150,30},{-130,50}})));
  SensorsControl.WaterSteam.TemperatureSensor superheated_steam_T_sensor(sensor_function="Calibration",
    causality="Kth_2",
    T_start=260.3,
    signal_unit="degC",
    display_unit="degC",
    T_0=273.15 + (260.3),
    P_0=1625000,
    Q_0=Q_cold_0,
    h_0=2940000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,64})));
  WaterSteam.Volumes.SteamDryer steam_dryer(x_steam_out=steam_dryer_x_steam_out) annotation (Placement(transformation(extent={{-45.5,-87.5041},{-10.5,-44.595}})));
  WaterSteam.BoundaryConditions.Sink dryer_liq_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={80,-86})));
  // MML4 instrument interfaces and component characteristics.
  Utilities.Interfaces.CalibrationInput drains_pressure_1 annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={50,10})));
  Utilities.Interfaces.CalibrationInput drains_pressure_2 annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={50,70})));
  Utilities.Interfaces.CalibrationInput inter_superheater_T annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-30,8})));
  Utilities.Interfaces.CalibrationInput superheated_steam_T annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-30,64})));
  Utilities.Interfaces.BoundaryCondition P_hot_steam_1 annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-90,10})));
  Utilities.Interfaces.BoundaryCondition P_hot_steam_2 annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-90,70})));
  Utilities.Interfaces.BoundaryCondition P_cold_steam annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-105,-30})));
  Utilities.Interfaces.BoundaryCondition Q_cold annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-75,-30})));
  Utilities.Interfaces.RealOutput Kth_1 annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-35,-15})));
  Utilities.Interfaces.RealOutput Kfr_hot_1 annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={25,-2})));
  Utilities.Interfaces.RealOutput Kth_2 annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-35,45})));
  Utilities.Interfaces.RealOutput Kfr_hot_2 annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={25,58})));
  SensorsControl.WaterSteam.PressureSensor P_hot_steam_1_sensor(sensor_function="BC",
    causality="",
    P_start=39.4,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(39.4),
    Q_0=Q_hot_1_0,
    h_0=2696000) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-90,-20})));
  SensorsControl.WaterSteam.PressureSensor P_hot_steam_2_sensor(sensor_function="BC",
    causality="",
    P_start=66.6,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(66.6),
    Q_0=Q_hot_2_0,
    h_0=2769000) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-90,40})));
  SensorsControl.WaterSteam.PressureSensor P_cold_steam_sensor(sensor_function="BC",
    causality="",
    P_start=16.25,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(16.25),
    Q_0=Q_cold_0,
    h_0=2560000) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-105,-60})));
  SensorsControl.WaterSteam.FlowSensor Q_cold_sensor(sensor_function="BC",
    causality="",
    Q_start=Q_cold_0,
    signal_unit="kg/s",
    display_unit="kg/s",
    Q_0=Q_cold_0,
    P_0=1625000,
    h_0=2560000) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-75,-60})));
  WaterSteam.Pipes.FrictionPipe drains_1_pipe(P_in_0=3.94e+06,
    P_out_0=3.91e+06,
    Q_0=Q_hot_1_0,
    T_0=T_hot_in_1_0,
    h_0=1.08e+06) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={28,-20})));
  WaterSteam.Pipes.FrictionPipe drains_2_pipe(P_in_0=6.66e+06,
    P_out_0=6.61e+06,
    Q_0=Q_hot_2_0,
    T_0=T_hot_in_2_0,
    h_0=1.25e+06) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={28,40})));

equation
  // Boundary conditions

  hot_steam_source_1.h_out = h_hot_steam_1;

  hot_steam_source_2.h_out = h_hot_steam_2;

  cold_steam_source.h_out = h_cold_steam;




  connect(vent_1_sink.C_in, superheater_1.C_vent) annotation (Line(points={{75,-40},{20,-40},{20,-27.8},{16,-27.8},{16,-28}}, color={28,108,200}));
  connect(superheater_1.C_cold_out, inter_superheater_T_sensor.C_in) annotation (Line(points={{0,-12},{0,-2}},    color={28,108,200}));
  connect(drains_1_P_sensor.C_out, drains_1_sink.C_in) annotation (Line(points={{60,-20},{75,-20}}, color={28,108,200}));
  connect(inter_superheater_T_sensor.C_out, superheater_2.C_cold_in) annotation (Line(points={{0,18},{0,32}}, color={28,108,200}));
  connect(drains_2_P_sensor.C_out, drains_2_sink.C_in) annotation (Line(points={{60,40},{75,40}}, color={28,108,200}));
  connect(vent_2_sink.C_in, superheater_2.C_vent) annotation (Line(points={{75,20},{20,20},{20,32.2},{16,32.2},{16,32}}, color={28,108,200}));
  connect(superheater_2.C_cold_out, superheated_steam_T_sensor.C_in) annotation (Line(points={{0,48},{0,54}},              color={28,108,200}));
  connect(superheated_steam_T_sensor.C_out, superheated_steam_sink.C_in) annotation (Line(points={{0,74},{0,80},{75,80}}, color={28,108,200}));
  connect(steam_dryer.C_hot_steam, superheater_1.C_cold_in) annotation (Line(points={{-10.5,-60.1983091},{0,-60.1983091},{0,-60.1983},{0,-28}}, color={28,108,200}));
  connect(steam_dryer.C_hot_liquid, dryer_liq_sink.C_in) annotation (Line(points={{-10.5,-75.8016182},{-10.5,-74},{-10,-74},{-10,-86},{75,-86}},  color={28,108,200}));

  // MML4 signal connections and fixed characteristics.
  connect(hot_steam_source_1.C_out, P_hot_steam_1_sensor.C_in) annotation (Line(points={{-135,-20},{-100,-20}}, color={28,108,200}));
  connect(P_hot_steam_1_sensor.C_out, superheater_1.C_hot_in) annotation (Line(points={{-80,-20},{-16,-20}}, color={28,108,200}));
  connect(hot_steam_source_2.C_out, P_hot_steam_2_sensor.C_in) annotation (Line(points={{-135,40},{-100,40}}, color={28,108,200}));
  connect(P_hot_steam_2_sensor.C_out, superheater_2.C_hot_in) annotation (Line(points={{-80,40},{-16,40}}, color={28,108,200}));
  connect(cold_steam_source.C_out, P_cold_steam_sensor.C_in) annotation (Line(points={{-135,-60},{-115,-60}}, color={28,108,200}));
  connect(P_cold_steam_sensor.C_out, Q_cold_sensor.C_in) annotation (Line(points={{-95,-60},{-85,-60}}, color={28,108,200}));
  connect(Q_cold_sensor.C_out, steam_dryer.C_in) annotation (Line(points={{-65,-60},{-45.5,-60},{-45.5,-60.1983091}}, color={28,108,200}));
  connect(superheater_1.C_hot_out, drains_1_pipe.C_in) annotation (Line(points={{16,-20},{18,-20}}, color={28,108,200}));
  connect(drains_1_pipe.C_out, drains_1_P_sensor.C_in) annotation (Line(points={{38,-20},{40,-20}}, color={28,108,200}));
  connect(superheater_2.C_hot_out, drains_2_pipe.C_in) annotation (Line(points={{16,40},{18,40}}, color={28,108,200}));
  connect(drains_2_pipe.C_out, drains_2_P_sensor.C_in) annotation (Line(points={{38,40},{40,40}}, color={28,108,200}));
  connect(drains_1_P_sensor.P_sensor, drains_pressure_1) annotation (Line(points={{50,-10},{50,10}}, color={0,0,127}));
  connect(drains_2_P_sensor.P_sensor, drains_pressure_2) annotation (Line(points={{50,50},{50,70}}, color={0,0,127}));
  connect(inter_superheater_T_sensor.T_sensor, inter_superheater_T) annotation (Line(points={{-10,8},{-30,8}}, color={0,0,127}));
  connect(superheated_steam_T_sensor.T_sensor, superheated_steam_T) annotation (Line(points={{-10,64},{-30,64}}, color={0,0,127}));
  connect(P_hot_steam_1_sensor.P_sensor, P_hot_steam_1) annotation (Line(points={{-90,-10},{-90,10}}, color={0,0,127}));
  connect(P_hot_steam_2_sensor.P_sensor, P_hot_steam_2) annotation (Line(points={{-90,50},{-90,70}}, color={0,0,127}));
  connect(P_cold_steam_sensor.P_sensor, P_cold_steam) annotation (Line(points={{-105,-50},{-105,-30}}, color={0,0,127}));
  connect(Q_cold_sensor.Q_sensor, Q_cold) annotation (Line(points={{-75,-50},{-75,-30}}, color={0,0,127}));
  connect(Kth_1, superheater_1.Kth) annotation (Line(points={{-35,-15},{-12,-15},{-12,-11.6}}, color={0,0,127}));
  connect(Kfr_hot_1, drains_1_pipe.Kfr) annotation (Line(points={{25,-2},{27.52,-2},{27.52,-33.6},{23.2,-33.6},{23.2,-16}}, color={0,0,127}));
  connect(Kth_2, superheater_2.Kth) annotation (Line(points={{-35,45},{-12,45},{-12,48.4}}, color={0,0,127}));
  connect(Kfr_hot_2, drains_2_pipe.Kfr) annotation (Line(points={{25,58},{27.52,58},{27.52,26.4},{23.2,26.4},{23.2,44}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-140},{160,140}}), graphics={
        Polygon(
          points={{-156,80},{-156,60},{-156,-62.5},{-156,-80},{-116,-80},{14,-80},{164,-80},{164,80},{14,80},{-116,80},{-156,80}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-94,58},{-94,36},{-94,-44},{-94,-64},{-74,-64},{18,-64},{146,-64},{146,58},{14,58},{-74,58},{-94,58}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={205,225,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{-104,40},{-112,42},{-112,-46},{-104,-46},{-94,-46},{20,-46},{128,-46},{130,40},{16,40},{-96,40},{-104,40}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-110,46},{-116,46},{-116,-52},{-108,-52},{-96,-52},{14,-52},{136,-52},{136,46},{14,46},{-96,46},{-110,46}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-110,28},{-116,28},{-116,-34},{-108,-34},{-96,-34},{18,-34},{116,-34},{114,26},{16,28},{-96,28},{-110,28}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-114,34},{-120,34},{-120,-38},{-112,-40},{-102,-40},{10,-40},{120,-42},{124,32},{12,34},{-100,34},{-114,34}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Rectangle(
          extent={{-144,50},{-94,-56}},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-144,58},{-98,22}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=10),
        Rectangle(
          extent={{-144,50},{-96,16}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius = 0),
        Rectangle(
          extent={{-116,58},{-96,36}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius = 0),
        Rectangle(
          extent={{10.5,11.5},{-10.5,-11.5}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius = 0,
          origin={-107.5,-51.5},
          rotation=90),
        Rectangle(
          extent={{14,23},{-14,-23}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius = 0,
          origin={-119,-36},
          rotation=90),
        Rectangle(
          extent={{20,23},{-20,-23}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=10,
          origin={-119,-42},
          rotation=90)}),                                        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-160,-140},{160,140}})));
end MoistureSeparatorReheater_reverse;
