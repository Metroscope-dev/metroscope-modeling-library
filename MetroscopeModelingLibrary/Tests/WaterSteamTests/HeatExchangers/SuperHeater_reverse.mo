within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model SuperHeater_reverse

  extends Modelica.Icons.Example;

    // Boundary conditions
  input Real P_hot_steam(start=60, min=0, nominal=11) "bar";
  input Real P_cold_steam(start=11, min=0, nominal=50) "bar";
  input Units.InletMassFlowRate Q_cold(start=500) "kg/s";
  input Real h_cold_steam(start=2.75e6) "J/kg"; // slightly humid cold steam
  input Real h_hot_steam(start=2.8e6) "J/kg"; // slightly superheated hot steam

  // Parameters
  parameter Units.Area S = 100;

  parameter Units.FrictionCoefficient Kfr_cold = 0;

  parameter Units.InletMassFlowRate Q_vent = 1;

  // Inputs for calibration
  input Real superheated_steam_temperature(start=224, min=0, nominal = 200) "degC";
  input Real drains_pressure(start=59.5, min=0, nominal = 100e5) "barA";

  // Calibrated parameters
  output Units.HeatExchangeCoefficient Kth;
  output Units.FrictionCoefficient Kfr_hot;

  WaterSteam.BoundaryConditions.WaterSource hot_steam_source
    annotation (Placement(transformation(extent={{-70,-2},{-50,18}})));
  WaterSteam.BoundaryConditions.WaterSink drains_sink
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  WaterSteam.BoundaryConditions.WaterSource cold_steam_source
    annotation (Placement(transformation(extent={{-42,-50},{-22,-30}})));
  WaterSteam.BoundaryConditions.WaterSink superheated_steam_sink
    annotation (Placement(transformation(extent={{16,30},{36,50}})));
  WaterSteam.HeatExchangers.SuperHeater superheater
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  WaterSteam.BoundaryConditions.WaterSink vent_sink
    annotation (Placement(transformation(extent={{70,-30},{90,-10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor
    superheated_steam_temperature_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,24})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor
    drains_pressure_sensor
    annotation (Placement(transformation(extent={{38,-10},{58,10}})));
equation

  // Boundary conditions
  hot_steam_source.P_out = P_hot_steam*1e5;
  hot_steam_source.h_out = h_hot_steam;
  cold_steam_source.P_out = P_cold_steam*1e5;
  cold_steam_source.h_out = h_cold_steam;
  cold_steam_source.Q_out = - Q_cold;

  // Component parameters
  superheater.S = S;
  superheater.Kfr_cold=Kfr_cold;
  superheater.Q_vent = Q_vent;

  // Inputs for calibration
  drains_pressure_sensor.P_barA = drains_pressure;
  superheated_steam_temperature_sensor.T_degC = superheated_steam_temperature;

  // Calibrated parameters
  superheater.Kth = Kth;
  superheater.Kfr_hot=Kfr_hot;

  connect(cold_steam_source.C_out,superheater. C_cold_in)
    annotation (Line(points={{-27,-40},{0,-40},{0,-8}}, color={28,108,200}));
  connect(hot_steam_source.C_out,superheater. C_hot_in) annotation (Line(points={{-55,8},
          {-34.5,8},{-34.5,0.2},{-16,0.2}},         color={28,108,200}));
  connect(vent_sink.C_in, superheater.C_vent) annotation (Line(points={{75,-20},
          {20,-20},{20,-7.8},{16,-7.8}},
                               color={28,108,200}));
  connect(superheater.C_cold_out, superheated_steam_temperature_sensor.C_in)
    annotation (Line(points={{-0.2,8},{-5.55112e-16,8},{-5.55112e-16,14}},
        color={28,108,200}));
  connect(superheated_steam_temperature_sensor.C_out, superheated_steam_sink.C_in)
    annotation (Line(points={{5.55112e-16,34},{5.55112e-16,40},{21,40}}, color={
          28,108,200}));
  connect(superheater.C_hot_out, drains_pressure_sensor.C_in)
    annotation (Line(points={{16,0},{38,0}}, color={28,108,200}));
  connect(drains_pressure_sensor.C_out, drains_sink.C_in)
    annotation (Line(points={{58,0},{75,0}}, color={28,108,200}));
end SuperHeater_reverse;
