within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model SteamGenerator

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure P_steam(start=70e5) "Pa";
  input Utilities.Units.PositiveMassFlowRate Q_feedwater(start=500) "kg/s";
  input Utilities.Units.Pressure P_feedwater(start=80e5) "Pa";
  input Utilities.Units.Temperature T_feedwater(start=225 + 273.15) "K";
  input Utilities.Units.PositiveMassFlowRate Q_purge(start=1) "kg/s";

  // Parameters
  input Real vapor_fraction(start=0.99);

  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.SteamGenerator steamGenerator annotation (Placement(transformation(extent={{-32,-60},{32,60}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source feedwater_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={58,0})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink steam_sink annotation (Placement(transformation(extent={{48,70},{68,90}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink purge_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-78})));
equation

  // Boundary conditions
  steam_sink.P_in = P_steam;
  feedwater_source.Q_out = - Q_feedwater;
  feedwater_source.P_out = P_feedwater;
  feedwater_source.T_out = T_feedwater;
  purge_sink.Q_in = Q_purge;

  // Parameters
  steamGenerator.vapor_fraction = 0.99;

  // Hypothesis
  steamGenerator.P_purge = P_steam; // Steam generator blowdown is assumed to be at the saturation pressure

  connect(feedwater_source.C_out, steamGenerator.feedwater_inlet) annotation (
      Line(points={{53,5.55112e-17},{34.5,5.55112e-17},{34.5,0},{16,0}},
                                                         color={28,108,200}));
  connect(steamGenerator.steam_outlet, steam_sink.C_in)
    annotation (Line(points={{0,60},{0,80},{53,80}},   color={28,108,200}));
  connect(purge_sink.C_in, steamGenerator.purge_outlet) annotation (Line(points={{8.88178e-16,-73},{8.88178e-16,-59},{0,-59}},
                                                               color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SteamGenerator;
