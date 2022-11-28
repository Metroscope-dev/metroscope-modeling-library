within MetroscopeModelingLibrary.Tests.WaterSteam.Volumes;
model FlashTank
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary Conditions
  input Units.Pressure P_source(start = 10e5) "Pa";
  input Units.NegativeMassFlowRate Q_source(start=-500) "kg/s";
  input Units.SpecificEnthalpy h_source(start=2e6) "J/kg";
  .MetroscopeModelingLibrary.WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-32,-30},{28,30}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-80,2},{-60,22}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink steam_sink annotation (Placement(transformation(extent={{56,2},{76,22}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink liquid_sink annotation (Placement(transformation(extent={{56,-22},{76,-2}})));
equation

  source.P_out = P_source;
  source.Q_out = Q_source;
  source.h_out = h_source;

  connect(source.C_out, flashTank.C_in) annotation (Line(points={{-65,12},{-32,12}}, color={28,108,200}));
  connect(flashTank.C_steam_out, steam_sink.C_in) annotation (Line(points={{28,12},{61,12}}, color={28,108,200}));
  connect(flashTank.C_liquid_out, liquid_sink.C_in) annotation (Line(points={{28,-12},{61,-12}}, color={28,108,200}));
end FlashTank;
