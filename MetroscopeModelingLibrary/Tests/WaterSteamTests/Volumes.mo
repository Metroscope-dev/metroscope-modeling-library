within MetroscopeModelingLibrary.Tests.WaterSteamTests;
package Volumes
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestPackageIcon;

  model FlashTank
    extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

    // Boundary Conditions
    input Units.Pressure P_source(start = 10e5) "Pa";
    input Units.NegativeMassFlowRate Q_source(start=-500) "kg/s";
    input Units.SpecificEnthalpy h_source(start=2e6) "J/kg";
    WaterSteam.Volumes.FlashTank flashTank
      annotation (Placement(transformation(extent={{-32,-30},{28,30}})));
    WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-80,2},{-60,22}})));
    WaterSteam.BoundaryConditions.Sink steam_sink annotation (Placement(transformation(extent={{56,2},{76,22}})));
    WaterSteam.BoundaryConditions.Sink liquid_sink annotation (Placement(transformation(extent={{56,-22},{76,-2}})));
  equation

    source.P_out = P_source;
    source.Q_out = Q_source;
    source.h_out = h_source;

    connect(flashTank.C_in, source.C_out)
      annotation (Line(points={{-32,12},{-65,12}}, color={28,108,200}));
    connect(flashTank.C_hot_steam, steam_sink.C_in)
      annotation (Line(points={{28,12},{61,12}}, color={28,108,200}));
    connect(flashTank.C_hot_liquid, liquid_sink.C_in)
      annotation (Line(points={{28,-12},{61,-12}}, color={28,108,200}));
  end FlashTank;

  model SteamDryer
    extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

    // Boundary Conditions
    input Units.Pressure P_source(start = 10e5) "Pa";
    input Units.NegativeMassFlowRate Q_source(start=-500) "kg/s";
    input Units.SpecificEnthalpy h_source(start=2e6) "J/kg";

    // Component parameters
    parameter Real x_steam_out = 0.9;

    WaterSteam.Volumes.SteamDryer steamDryer
      annotation (Placement(transformation(extent={{-32,-26},{28,34}})));
    WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-80,2},{-60,22}})));
    WaterSteam.BoundaryConditions.Sink steam_sink annotation (Placement(transformation(extent={{56,2},{76,22}})));
    WaterSteam.BoundaryConditions.Sink liquid_sink annotation (Placement(transformation(extent={{56,-50},{76,-30}})));
  equation

    // Boundary conditions
    source.P_out = P_source;
    source.Q_out = Q_source;
    source.h_out = h_source;

    // Component parameters
    steamDryer.x_steam_out = x_steam_out;

    connect(steamDryer.C_in, source.C_out) annotation (Line(points={{-32,12.1818},{-32,12},{-65,12}},
                                              color={28,108,200}));
    connect(steamDryer.C_hot_steam, steam_sink.C_in) annotation (Line(points={{28,12.1818},{28,12},{61,12}},
                                                    color={28,108,200}));
    connect(steamDryer.C_hot_liquid, liquid_sink.C_in) annotation (Line(points={{28,-9.63636},{28,-8},{52,-8},{52,-40},{61,-40}},
                                                        color={28,108,200}));
  end SteamDryer;
end Volumes;
