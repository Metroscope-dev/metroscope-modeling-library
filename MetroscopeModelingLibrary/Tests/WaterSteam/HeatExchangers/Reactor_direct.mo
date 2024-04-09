within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Reactor_direct

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure P_steam(start=70e5) "Pa";
  input Utilities.Units.Pressure P_feedwater(start=80e5) "Pa";
  input Utilities.Units.Temperature T_feedwater(start=225 + 273.15) "K";
  input Utilities.Units.Power thermal_power(start=900e6) "W";

  // Parameters
  input Real vapor_fraction(start=0.99);

  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reactor         steam_generator annotation (Placement(transformation(extent={{-32,-60},{32,60}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source feedwater_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={58,0})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink steam_sink annotation (Placement(transformation(extent={{48,70},{68,90}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
equation

  // Boundary conditions
  steam_generator.thermal_power = thermal_power;
  steam_sink.P_in = P_steam;
  feedwater_source.P_out = P_feedwater;
  feedwater_source.T_out = T_feedwater;

  // Parameters
  steam_generator.vapor_fraction = 0.99;

  connect(feedwater_source.C_out, steam_generator.feedwater_inlet) annotation (Line(points={{53,5.55112e-17},{34.5,5.55112e-17},{34.5,0},{16,0}}, color={28,108,200}));
  connect(steam_generator.steam_outlet, steam_sink.C_in) annotation (Line(points={{0,60},{0,80},{53,80}}, color={28,108,200}));
  connect(steam_generator.C_thermal_power, source.C_out) annotation (Line(points={{-16,0},{-65.2,0}}, color={244,125,35}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Reactor_direct;
