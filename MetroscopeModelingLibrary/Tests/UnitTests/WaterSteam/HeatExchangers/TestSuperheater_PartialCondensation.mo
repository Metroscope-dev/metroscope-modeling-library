within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.HeatExchangers;
model TestSuperheater_PartialCondensation
  extends Modelica.Icons.Example;
  replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Superheater_PartialCondensation Superheater
    annotation (Placement(transformation(extent={{-16,32},{16,48}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_cold_ReheatSt
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={0,72})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_hot_Cond
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={40,40})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sinkVent
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={30,14})));

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_cold_steam
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,10})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_hot_mainSt
    annotation (Placement(transformation(extent={{-50,30},{-30,50}})));
equation
  // Forward causality
  //inlets
  source_hot_mainSt.T_out = 280+273.15;
  source_hot_mainSt.P_out = 60e5;

  source_cold_steam.P_out = 11e5;
  source_cold_steam.h_out = 2800e3;
  source_cold_steam.Q_out = -275;

  //outlets
  sink_hot_Cond.h_vol = 1.2e6;
  sink_cold_ReheatSt.h_vol = 1e6;

  sinkVent.Q_in = 2.5;
  sinkVent.h_vol = 1.2e6;

  //heatexchangers
  Superheater.Kth = 6300;
  Superheater.S_tot = 100;
  Superheater.Kfr_cold=1;
  Superheater.Kfr_hot=1;
  Superheater.x_hot_out = 0.05;

  // Reverse causality
  // Determines the heat exchange coefficient by giving the outet temperature on the cold side
  // You can also determine the Kfr by giving the outlet pressures
  /*
  //inlets
  source_hot_mainSt.T_out = 280+273.15;
  source_hot_mainSt.P_out = 60e5;

  source_cold_steam.P_out = 11e5;
  source_cold_steam.h_out = 2800e3;
  source_cold_steam.Q_out = -275;

  //outlets
  sink_hot_Cond.h_vol = 1.2e6;

  sink_cold_ReheatSt.T_in = 246+273.15;
  sink_cold_ReheatSt.h_vol = 1e6;

  sinkVent.Q_in = 2.5;
  sinkVent.h_vol = 1.2e6;

  //heatexchangers
  Superheater.S_tot = 100;
  Superheater.Kfr_cold=1;
  Superheater.Kfr_hot=1;
  Superheater.x_hot_out = 0.05;
  */

  connect(Superheater.C_vent_out, sinkVent.C_in)
    annotation (Line(points={{16,32},{16,24},{30,24}},
                                                color={63,81,181}));
  connect(Superheater.C_hot_out, sink_hot_Cond.C_in)
    annotation (Line(points={{16.2,40},{30,40}}, color={63,81,181}));
  connect(sink_cold_ReheatSt.C_in, Superheater.C_cold_out)
    annotation (Line(points={{0,62},{0,48}},   color={63,81,181}));
  connect(source_hot_mainSt.C_out, Superheater.C_hot_in) annotation (Line(
        points={{-30,40},{-16,40},{-16,40.2}}, color={63,81,181}));
  connect(source_cold_steam.C_out, Superheater.C_cold_in)
    annotation (Line(points={{6.66134e-16,20},{6.66134e-16,28},{0,28},{0,32}},
                                                color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                           Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-60,0},{60,80}})));
end TestSuperheater_PartialCondensation;
