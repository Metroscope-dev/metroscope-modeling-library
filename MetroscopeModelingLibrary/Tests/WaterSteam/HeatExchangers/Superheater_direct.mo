within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Superheater_direct

  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

    // Boundary conditions
  input Real P_hot_steam(start=60, min=0, nominal=11) "bar";
  input Real P_cold_steam(start=11, min=0, nominal=50) "bar";
  input Units.PositiveMassFlowRate Q_cold(start=1300) "kg/s";
  input Real h_cold_steam(start=2.75e6) "J/kg"; // slightly humid cold steam
  input Real h_hot_steam(start=2.8e6) "J/kg"; // slightly superheated hot steam

  // Parameters
  parameter Units.Area S = 100;
  parameter Units.HeatExchangeCoefficient Kth = 7e3;
  parameter Units.FrictionCoefficient Kfr_cold = 0;
  parameter Units.FrictionCoefficient Kfr_hot = 500; // About 1 bar of pressure loss in the reheater
  parameter Units.PositiveMassFlowRate Q_vent=1;

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_steam_source annotation (Placement(transformation(extent={{-68,-10},{-48,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink drains_sink annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_steam_source annotation (Placement(transformation(extent={{-42,-50},{-22,-30}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink superheated_steam_sink annotation (Placement(transformation(extent={{16,30},{36,50}})));
  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Superheater superheater annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink vent_sink annotation (Placement(transformation(extent={{48,-30},{68,-10}})));
equation

  // Boundary conditions
  hot_steam_source.P_out = P_hot_steam*1e5;
  hot_steam_source.h_out = h_hot_steam;

  cold_steam_source.P_out = P_cold_steam*1e5;
  cold_steam_source.h_out = h_cold_steam;
  cold_steam_source.Q_out = - Q_cold;

  // Component parameters
  superheater.Kth = Kth;
  superheater.S = S;
  superheater.Kfr_cold=Kfr_cold;
  superheater.Kfr_hot=Kfr_hot;
  superheater.Q_vent = Q_vent;

  connect(superheater.C_cold_out, superheated_steam_sink.C_in) annotation (Line(
        points={{0,8},{0,8},{0,40},{21,40}},    color={28,108,200}));
  connect(superheater.C_hot_out, drains_sink.C_in)
    annotation (Line(points={{16,0},{53,0}}, color={28,108,200}));
  connect(cold_steam_source.C_out,superheater. C_cold_in)
    annotation (Line(points={{-27,-40},{0,-40},{0,-8}}, color={28,108,200}));
  connect(hot_steam_source.C_out,superheater. C_hot_in) annotation (Line(points={{-53,0},{-34.5,0},{-34.5,0},{-16,0}},
                                                    color={28,108,200}));
  connect(vent_sink.C_in, superheater.C_vent) annotation (Line(points={{53,-20},
          {16,-20},{16,-7.8}}, color={28,108,200}));
end Superheater_direct;
