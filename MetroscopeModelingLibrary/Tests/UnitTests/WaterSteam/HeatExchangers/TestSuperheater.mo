within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.HeatExchangers;
model TestSuperheater
  extends Modelica.Icons.Example;
  replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Superheater Superheater
    annotation (Placement(transformation(extent={{-20,26},{12,42}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_cold_ReheatSt
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-4,66})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_hot_Cond
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={48,34})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sinkVent
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={18,10})));

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_cold_steam
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-4,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_hot_mainSt
    annotation (Placement(transformation(extent={{-50,24},{-30,44}})));
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
  */

  connect(Superheater.C_vent_out, sinkVent.C_in)
    annotation (Line(points={{12,26},{12,20},{18,20}},
                                                color={63,81,181}));
  connect(Superheater.C_hot_out, sink_hot_Cond.C_in)
    annotation (Line(points={{12.2,34},{38,34}}, color={63,81,181}));
  connect(sink_cold_ReheatSt.C_in, Superheater.C_cold_out)
    annotation (Line(points={{-4,56},{-4,42}}, color={63,81,181}));
  connect(source_hot_mainSt.C_out, Superheater.C_hot_in) annotation (Line(
        points={{-30,34},{-20,34},{-20,34.2}}, color={63,81,181}));
  connect(source_cold_steam.C_out, Superheater.C_cold_in)
    annotation (Line(points={{-4,10},{-4,26}},  color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-60,-20},{60,80}})));
end TestSuperheater;
