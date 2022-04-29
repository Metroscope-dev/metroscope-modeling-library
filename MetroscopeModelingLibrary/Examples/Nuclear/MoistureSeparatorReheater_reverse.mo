within MetroscopeModelingLibrary.Examples.Nuclear;
model MoistureSeparatorReheater_reverse
  // Boundary conditions
  input Real P_hot_steam(start=60, min=0, nominal=11) "bar";
  input Real P_cold_steam(start=11, min=0, nominal=50) "bar";
  input Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
  input Real h_cold_steam(start=2.75e6) "J/kg"; // slightly humid cold steam
  input Real h_hot_steam(start=2.8e6) "J/kg"; // slightly superheated hot steam

  // Parameters
  parameter Units.Area S = 100;
  parameter Units.FrictionCoefficient Kfr_cold = 0;
  parameter Units.PositiveMassFlowRate Q_vent=1;

  // Inputs for calibration
  input Real inter_superheater_T(start=200, min=0, nominal = 200) "degC";
  input Real superheated_steam_T(start=260, min=0, nominal = 200) "degC";
  input Real drains_pressure(start=59.5, min=0, nominal = 60) "barA";

  // Calibrated parameters
  output Units.HeatExchangeCoefficient Kth_1;
  output Units.FrictionCoefficient Kfr_hot_1;
  output Units.HeatExchangeCoefficient Kth_2;
  output Units.FrictionCoefficient Kfr_hot_2;

  // Components
  WaterSteam.BoundaryConditions.Source hot_steam_source_1 annotation (Placement(transformation(extent={{-78,-30},{-58,-10}})));
  WaterSteam.BoundaryConditions.Sink drains_1_sink annotation (Placement(transformation(extent={{70,-30},{90,-10}})));
  WaterSteam.BoundaryConditions.Source cold_steam_source annotation (Placement(transformation(extent={{-78,-70},{-58,-50}})));
  WaterSteam.BoundaryConditions.Sink superheated_steam_sink annotation (Placement(transformation(extent={{72,70},{92,90}})));
  WaterSteam.HeatExchangers.Superheater superheater_1 annotation (Placement(transformation(extent={{-16,-28},{16,-12}})));
  WaterSteam.BoundaryConditions.Sink vent_1_sink annotation (Placement(transformation(extent={{70,-50},{90,-30}})));
  Sensors.WaterSteam.TemperatureSensor inter_superheater_T_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,8})));
  Sensors.WaterSteam.PressureSensor drains_1_P_sensor annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
  WaterSteam.HeatExchangers.Superheater superheater_2 annotation (Placement(transformation(extent={{-16,28},{16,44}})));
  WaterSteam.BoundaryConditions.Sink drains_2_sink annotation (Placement(transformation(extent={{68,26},{88,46}})));
  WaterSteam.BoundaryConditions.Sink vent_2_sink annotation (Placement(transformation(extent={{68,6},{88,26}})));
  Sensors.WaterSteam.PressureSensor drains_2_P_sensor annotation (Placement(transformation(extent={{38,26},{58,46}})));
  WaterSteam.BoundaryConditions.Source hot_steam_source_2 annotation (Placement(transformation(extent={{-78,26},{-58,46}})));
  Sensors.WaterSteam.TemperatureSensor superheated_steam_T_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,62})));
equation
  // Boundary conditions
  hot_steam_source_1.P_out = P_hot_steam*1e5;
  hot_steam_source_1.h_out = h_hot_steam;

  hot_steam_source_2.P_out = P_hot_steam*1e5;
  hot_steam_source_2.h_out = h_hot_steam;

  cold_steam_source.P_out = P_cold_steam*1e5;
  cold_steam_source.h_out = h_cold_steam;
  cold_steam_source.Q_out = -Q_cold;

  // Component parameters
  superheater_1.S = S;
  superheater_1.Kfr_cold = Kfr_cold;
  superheater_1.Q_vent = Q_vent;

  superheater_2.S = S;
  superheater_2.Kfr_cold = Kfr_cold;
  superheater_2.Q_vent = Q_vent;

  // Inputs for calibration
  drains_1_P_sensor.P_barA = drains_pressure;
  drains_2_P_sensor.P_barA = drains_pressure;
  inter_superheater_T_sensor.T_degC = inter_superheater_T;
  superheated_steam_T_sensor.T_degC = superheated_steam_T;

  // Calibrated parameters
  superheater_1.Kth = Kth_1;
  superheater_1.Kfr_hot = Kfr_hot_1;
  superheater_2.Kth = Kth_2;
  superheater_2.Kfr_hot = Kfr_hot_2;
  connect(cold_steam_source.C_out, superheater_1.C_cold_in) annotation (Line(points={{-63,-60},{0,-60},{0,-28}}, color={28,108,200}));
  connect(hot_steam_source_1.C_out, superheater_1.C_hot_in) annotation (Line(points={{-63,-20},{-39.5,-20},{-39.5,-19.8},{-16,-19.8}}, color={28,108,200}));
  connect(vent_1_sink.C_in, superheater_1.C_vent) annotation (Line(points={{75,-40},{20,-40},{20,-27.8},{16,-27.8}}, color={28,108,200}));
  connect(superheater_1.C_cold_out, inter_superheater_T_sensor.C_in) annotation (Line(points={{-0.2,-12},{0,-12},{0,-2}}, color={28,108,200}));
  connect(superheater_1.C_hot_out, drains_1_P_sensor.C_in) annotation (Line(points={{16,-20},{40,-20}}, color={28,108,200}));
  connect(drains_1_P_sensor.C_out, drains_1_sink.C_in) annotation (Line(points={{60,-20},{75,-20}}, color={28,108,200}));
  connect(inter_superheater_T_sensor.C_out, superheater_2.C_cold_in) annotation (Line(points={{0,18},{0,28}}, color={28,108,200}));
  connect(drains_2_P_sensor.C_out, drains_2_sink.C_in) annotation (Line(points={{58,36},{73,36}}, color={28,108,200}));
  connect(vent_2_sink.C_in, superheater_2.C_vent) annotation (Line(points={{73,16},{20,16},{20,28.2},{16,28.2}}, color={28,108,200}));
  connect(drains_2_P_sensor.C_in, superheater_2.C_hot_out) annotation (Line(points={{38,36},{16,36}}, color={28,108,200}));
  connect(superheater_2.C_hot_in, hot_steam_source_2.C_out) annotation (Line(points={{-16,36.2},{-40,36.2},{-40,36},{-63,36}}, color={28,108,200}));
  connect(superheater_2.C_cold_out, superheated_steam_T_sensor.C_in) annotation (Line(points={{-0.2,44},{-0.2,48},{-5.55112e-16,48},{-5.55112e-16,52}}, color={28,108,200}));
  connect(superheated_steam_T_sensor.C_out, superheated_steam_sink.C_in) annotation (Line(points={{0,72},{0,80},{77,80}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end MoistureSeparatorReheater_reverse;
