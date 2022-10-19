within MetroscopeModelingLibrary.Tests.WaterSteam.Volumes;
model SteamDryer
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary Conditions
  input Units.Pressure P_source(start = 10e5) "Pa";
  input Units.NegativeMassFlowRate Q_source(start=-500) "kg/s";
  input Units.SpecificEnthalpy h_source(start=2e6) "J/kg";

  // Component parameters
  parameter Real x_steam_out = 0.9;

  .MetroscopeModelingLibrary.WaterSteam.Volumes.SteamDryer steamDryer annotation (Placement(transformation(extent={{-32,-26},{28,34}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-80,2},{-60,22}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink steam_sink annotation (Placement(transformation(extent={{56,2},{76,22}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink liquid_sink annotation (Placement(transformation(extent={{56,-50},{76,-30}})));
equation

  // Boundary conditions
  source.P_out = P_source;
  source.Q_out = Q_source;
  source.h_out = h_source;

  // Component parameters
  steamDryer.x_steam_out = x_steam_out;

  connect(steamDryer.C_in, source.C_out) annotation (Line(points={{-32,16},{-32,12},{-65,12}},
                                            color={28,108,200}));
  connect(steamDryer.C_out_steam, steam_sink.C_in) annotation (Line(points={{28,16},{28,12},{61,12}},
                                                  color={28,108,200}));
  connect(steamDryer.C_out_liquid, liquid_sink.C_in) annotation (Line(points={{28,-8},{28,-8},{52,-8},{52,-40},{61,-40}},
                                                      color={28,108,200}));
end SteamDryer;
