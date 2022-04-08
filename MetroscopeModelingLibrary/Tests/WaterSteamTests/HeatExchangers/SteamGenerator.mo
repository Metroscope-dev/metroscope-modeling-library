within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model SteamGenerator

  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;
  WaterSteam.HeatExchangers.SteamGenerator steamGenerator
    annotation (Placement(transformation(extent={{-34,-60},{30,58}})));
  WaterSteam.BoundaryConditions.WaterSource feedwater_source annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={68,-8})));
  WaterSteam.BoundaryConditions.WaterSink steam_sink
    annotation (Placement(transformation(extent={{40,64},{60,84}})));
  WaterSteam.BoundaryConditions.WaterSink purge_sink annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-2,-84})));
equation

  purge_sink.Q_in = 1e-3;
  purge_sink.P_in = 80e5;

  feedwater_source.Q_out = -500;
  feedwater_source.P_out = 80e5;
  feedwater_source.T_out = 273.15 + 225;

  steam_sink.P_in = 70e5;
  steamGenerator.vapor_fraction = 0.99;

  connect(feedwater_source.C_out, steamGenerator.feedwater_inlet) annotation (
      Line(points={{63,-8},{38.5,-8},{38.5,-1},{14,-1}}, color={28,108,200}));
  connect(steamGenerator.steam_outlet, steam_sink.C_in)
    annotation (Line(points={{-2,58},{-2,74},{45,74}}, color={28,108,200}));
  connect(purge_sink.C_in, steamGenerator.purge_outlet) annotation (Line(points={{-2,-79},{-2,-69.0083},{-2,-59.0167},{-2,-59.0167}},
                                                               color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SteamGenerator;
