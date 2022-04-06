within MetroscopeModelingLibrary.Tests.WaterSteamTests;
package Volumes
  extends Modelica.Icons.ExamplesPackage;
  model FlashTank_direct
    extends Modelica.Icons.Example;

    // Boundary Conditions
    input Real P(start = 10, nominal=1e5) "barA";
    input Real Q_in(start=500, nominal=500) "kg/s";
    input Real h_in(start=2e6) "J/kg";
    WaterSteam.Volumes.FlashTank flashTank
      annotation (Placement(transformation(extent={{-32,-30},{28,30}})));
    WaterSteam.BoundaryConditions.WaterSource source
      annotation (Placement(transformation(extent={{-80,2},{-60,22}})));
    WaterSteam.BoundaryConditions.WaterSink steam_sink
      annotation (Placement(transformation(extent={{56,2},{76,22}})));
    WaterSteam.BoundaryConditions.WaterSink liquid_sink
      annotation (Placement(transformation(extent={{56,-22},{76,-2}})));
  equation

    source.P_out = P*1e5;
    source.Q_out = -Q_in;
    source.h_out = h_in;


    connect(flashTank.C_in, source.C_out)
      annotation (Line(points={{-32,12},{-65,12}}, color={28,108,200}));
    connect(flashTank.C_hot_steam, steam_sink.C_in)
      annotation (Line(points={{28,12},{61,12}}, color={28,108,200}));
    connect(flashTank.C_hot_liquid, liquid_sink.C_in)
      annotation (Line(points={{28,-12},{61,-12}}, color={28,108,200}));
  end FlashTank_direct;

  model SteamDryer_direct
    extends Modelica.Icons.Example;

    // Boundary Conditions
    input Real P(start = 10, nominal=1e5) "barA";
    input Real Q_in(start=500, nominal=500) "kg/s";
    input Real h_in(start=2e6) "J/kg";

    // Component parameters
    parameter Real x_steam_out = 0.9;

    WaterSteam.Volumes.SteamDryer steamDryer
      annotation (Placement(transformation(extent={{-32,-28},{28,32}})));
    WaterSteam.BoundaryConditions.WaterSource source
      annotation (Placement(transformation(extent={{-80,2},{-60,22}})));
    WaterSteam.BoundaryConditions.WaterSink steam_sink
      annotation (Placement(transformation(extent={{56,2},{76,22}})));
    WaterSteam.BoundaryConditions.WaterSink liquid_sink
      annotation (Placement(transformation(extent={{56,-22},{76,-2}})));
  equation

    // Boundary conditions
    source.P_out = P*1e5;
    source.Q_out = -Q_in;
    source.h_out = h_in;

    // Component parameters
    steamDryer.x_steam_out = x_steam_out;

    connect(steamDryer.C_in, source.C_out) annotation (Line(points={{-32,
            10.1818},{-48,10.1818},{-48,12},{-65,12}},
                                              color={28,108,200}));
    connect(steamDryer.C_hot_steam, steam_sink.C_in) annotation (Line(points={{28,
            10.1818},{44,10.1818},{44,12},{61,12}}, color={28,108,200}));
    connect(steamDryer.C_hot_liquid, liquid_sink.C_in) annotation (Line(points={{28,
            -11.6364},{44,-11.6364},{44,-12},{61,-12}}, color={28,108,200}));
  end SteamDryer_direct;
end Volumes;
