within MetroscopeModelingLibrary.Examples.Nuclear.MainSteam;
model MoistureSeparatorReheater_reverse
  // Initialization parameters
  parameter Units.PositiveMassFlowRate Q_cold_0 = 660 "kg/s";
  parameter Units.PositiveMassFlowRate Q_hot_1_0 = 33 "kg/s";
  parameter Units.PositiveMassFlowRate Q_hot_2_0 = 23 "kg/s";
  parameter Units.Temperature T_cold_in_1_0 = 202 + 273.15 "K";
  parameter Units.Temperature T_cold_in_2_0 = 234 + 273.15 "K";
  parameter Units.Temperature T_hot_in_1_0 = 249 + 273.15 "K";
  parameter Units.Temperature T_hot_in_2_0 = 282 + 273.15 "K";

  // Boundary conditions
  input Real P_hot_steam_1(start=39.4, min=0, nominal=11) "bar";
  input Real P_hot_steam_2(start=66.6, min=0, nominal=11) "bar";
  input Real P_cold_steam(start=16.25, min=0, nominal=15) "bar";
  input Units.PositiveMassFlowRate Q_cold(start=Q_cold_0) "kg/s";
  input Real h_cold_steam(start=2.560e6) "J/kg";
  input Real h_hot_steam_1(start=2.696e6) "J/kg";
  input Real h_hot_steam_2(start=2.769e6) "J/kg";

  // Parameters
  parameter Units.Area S = 100;
  parameter Units.FrictionCoefficient Kfr_cold = 0;
  parameter Units.PositiveMassFlowRate Q_vent=1;
  parameter Units.Fraction steam_dryer_x_steam_out = 0.99;

  // Inputs for calibration
  input Real inter_superheater_T(start=235, min=0, nominal = 200) "degC";
  input Real superheated_steam_T(start=260.3, min=0, nominal = 200) "degC";
  input Real drains_pressure_1(start=39.1, min=0, nominal = 60) "barA";
  input Real drains_pressure_2(start=66.1, min=0, nominal = 60) "barA";

  // Calibrated parameters
  output Units.HeatExchangeCoefficient Kth_1;
  output Units.FrictionCoefficient Kfr_hot_1;
  output Units.HeatExchangeCoefficient Kth_2;
  output Units.FrictionCoefficient Kfr_hot_2;

  // Components
  WaterSteam.BoundaryConditions.Source hot_steam_source_1(Q_out(start=-Q_hot_1_0)) annotation (Placement(transformation(extent={{-80,-30},{-60,-10}})));
  WaterSteam.BoundaryConditions.Sink drains_1_sink annotation (Placement(transformation(extent={{70,-30},{90,-10}})));
  WaterSteam.BoundaryConditions.Source cold_steam_source annotation (Placement(transformation(extent={{-80,-70},{-60,-50}})));
  WaterSteam.BoundaryConditions.Sink superheated_steam_sink annotation (Placement(transformation(extent={{70,70},{90,90}})));
  WaterSteam.HeatExchangers.Superheater superheater_1(Q_cold_0=Q_cold_0, Q_hot_0=Q_hot_1_0, T_cold_in_0=T_cold_in_1_0, T_hot_in_0=T_hot_in_1_0) annotation (Placement(transformation(extent={{-16,-28},{16,-12}})));
  WaterSteam.BoundaryConditions.Sink vent_1_sink annotation (Placement(transformation(extent={{70,-50},{90,-30}})));
  Sensors.WaterSteam.TemperatureSensor inter_superheater_T_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,8})));
  Sensors.WaterSteam.PressureSensor drains_1_P_sensor annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
  WaterSteam.HeatExchangers.Superheater superheater_2(Q_cold_0=Q_cold_0, Q_hot_0=Q_hot_2_0, T_cold_in_0=T_cold_in_2_0, T_hot_in_0=T_hot_in_2_0) annotation (Placement(transformation(extent={{-16,32},{16,48}})));
  WaterSteam.BoundaryConditions.Sink drains_2_sink annotation (Placement(transformation(extent={{70,30},{90,50}})));
  WaterSteam.BoundaryConditions.Sink vent_2_sink annotation (Placement(transformation(extent={{70,10},{90,30}})));
  Sensors.WaterSteam.PressureSensor drains_2_P_sensor annotation (Placement(transformation(extent={{40,30},{60,50}})));
  WaterSteam.BoundaryConditions.Source hot_steam_source_2(Q_out(start=-Q_hot_2_0)) annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
  Sensors.WaterSteam.TemperatureSensor superheated_steam_T_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,64})));
  WaterSteam.Volumes.SteamDryer steam_dryer annotation (Placement(transformation(extent={{-45.5,-87.5041},{-10.5,-44.595}})));
  WaterSteam.BoundaryConditions.Sink dryer_liq_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={80,-86})));
equation
  // Boundary conditions
  hot_steam_source_1.P_out = P_hot_steam_1*1e5;
  hot_steam_source_1.h_out = h_hot_steam_1;

  hot_steam_source_2.P_out = P_hot_steam_2*1e5;
  hot_steam_source_2.h_out = h_hot_steam_2;

  cold_steam_source.P_out = P_cold_steam*1e5;
  cold_steam_source.h_out = h_cold_steam;
  cold_steam_source.Q_out = -Q_cold;

  // Component parameters
  steam_dryer.x_steam_out = steam_dryer_x_steam_out;
  superheater_1.S = S;
  superheater_1.Kfr_cold = Kfr_cold;
  superheater_1.Q_vent = Q_vent;

  superheater_2.S = S;
  superheater_2.Kfr_cold = Kfr_cold;
  superheater_2.Q_vent = Q_vent;

  // Inputs for calibration
  drains_1_P_sensor.P_barA = drains_pressure_1;
  drains_2_P_sensor.P_barA = drains_pressure_2;
  inter_superheater_T_sensor.T_degC = inter_superheater_T;
  superheated_steam_T_sensor.T_degC = superheated_steam_T;
  //inter_superheater_T_sensor.h = 2.8821e6;
  //superheated_steam_T_sensor.h = 2.9435e6;

  // Calibrated parameters
  superheater_1.Kth = Kth_1;
  superheater_1.Kfr_hot = Kfr_hot_1;
  superheater_2.Kth = Kth_2;
  superheater_2.Kfr_hot = Kfr_hot_2;
  connect(hot_steam_source_1.C_out, superheater_1.C_hot_in) annotation (Line(points={{-65,-20},{-16,-20}},                             color={28,108,200}));
  connect(vent_1_sink.C_in, superheater_1.C_vent) annotation (Line(points={{75,-40},{20,-40},{20,-27.8},{16,-27.8}}, color={28,108,200}));
  connect(superheater_1.C_cold_out, inter_superheater_T_sensor.C_in) annotation (Line(points={{0,-12},{0,-12},{0,-2}},    color={28,108,200}));
  connect(superheater_1.C_hot_out, drains_1_P_sensor.C_in) annotation (Line(points={{16,-20},{40,-20}}, color={28,108,200}));
  connect(drains_1_P_sensor.C_out, drains_1_sink.C_in) annotation (Line(points={{60,-20},{75,-20}}, color={28,108,200}));
  connect(inter_superheater_T_sensor.C_out, superheater_2.C_cold_in) annotation (Line(points={{0,18},{0,32}}, color={28,108,200}));
  connect(drains_2_P_sensor.C_out, drains_2_sink.C_in) annotation (Line(points={{60,40},{75,40}}, color={28,108,200}));
  connect(vent_2_sink.C_in, superheater_2.C_vent) annotation (Line(points={{75,20},{20,20},{20,32.2},{16,32.2}}, color={28,108,200}));
  connect(drains_2_P_sensor.C_in, superheater_2.C_hot_out) annotation (Line(points={{40,40},{16,40}}, color={28,108,200}));
  connect(superheater_2.C_hot_in, hot_steam_source_2.C_out) annotation (Line(points={{-16,40},{-65,40}},                       color={28,108,200}));
  connect(superheater_2.C_cold_out, superheated_steam_T_sensor.C_in) annotation (Line(points={{0,48},{-5.55112e-16,48},{-5.55112e-16,54}},              color={28,108,200}));
  connect(superheated_steam_T_sensor.C_out, superheated_steam_sink.C_in) annotation (Line(points={{0,74},{0,80},{75,80}}, color={28,108,200}));
  connect(cold_steam_source.C_out, steam_dryer.C_in) annotation (Line(points={{-65,-60},{-56.5,-60},{-56.5,-60.1983},{-45.5,-60.1983}}, color={28,108,200}));
  connect(steam_dryer.C_hot_steam, superheater_1.C_cold_in) annotation (Line(points={{-10.5,-60.1983},{0,-60.1983},{0,-28}}, color={28,108,200}));
  connect(steam_dryer.C_hot_liquid, dryer_liq_sink.C_in) annotation (Line(points={{-10.5,-75.8016},{-10.5,-74},{-10,-74},{-10,-86},{75,-86}},  color={28,108,200}));
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
