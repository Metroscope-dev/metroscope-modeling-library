within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.HeatExchangers;
model TestCondReheater
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_hot_CondSteam(Q_in(start=60))
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-2,-2})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_cold_hotwater
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-58,36})));

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_cold_coldwater
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={48,34})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_hot_SatSteam
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-4,78})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.CondReheater CondReheater
    annotation (Placement(transformation(extent={{-20,26},{12,42}})));
equation

  // Forward causality
  // Note that you don't need to specify the steam flow rate, since the energy echange is determined by the phase transition (see documentation)
  //inlets
  source_hot_SatSteam.h_out = 2.5e6;
  source_hot_SatSteam.P_out = 11e5;
  //source_hot_SatSteam.Q_out = -60;

  source_cold_coldwater.P_out = 50e5;
  source_cold_coldwater.h_out = 549552;
  source_cold_coldwater.Q_out = -500;

  //outlets
  sink_cold_hotwater.h_vol = 0.9e6;
  sink_hot_CondSteam.h_vol = 1e6;

  CondReheater.S_tot = 100;
  CondReheater.Kth = 61e3;
  CondReheater.Kfr_cold = 1e-3;
  CondReheater.Kfr_hot = 1e-3;

  // Reverse causality
  // Determines the Kth by giving the outlet temperature on the cold side
  // Determines both Kfr by giving the pressures at both outlets
  /*
  //inlets
  source_hot_SatSteam.h_out = 2.5e6;
  source_hot_SatSteam.P_out = 11e5;
  //source_hot_mainSt.Q_out = -60;

  source_cold_coldwater.P_out = 50e5;
  source_cold_coldwater.T_out = 130 + 273.15;
  source_cold_coldwater.Q_out = -500;

  //outlets
  sink_cold_hotwater.h_vol = 0.9e6;
  sink_cold_hotwater.T_in = 180 + 273.15;
  sink_cold_hotwater.P_in = 49.9e5;

  sink_hot_CondSteam.h_vol = 1e6;
  sink_hot_CondSteam.P_in = 10.9e5;
  
  CondReheater.S_tot = 100;
  */
  connect(sink_cold_hotwater.C_in,CondReheater. C_cold_out) annotation (Line(
        points={{-48,36},{-34,36},{-34,34},{-20,34}}, color={63,81,181}));
  connect(source_hot_SatSteam.C_out,CondReheater. C_hot_in) annotation (Line(
        points={{-4,68},{-4,56},{-4,42},{-3,42}}, color={63,81,181}));
  connect(CondReheater.C_cold_in, source_cold_coldwater.C_out)
    annotation (Line(points={{12,34},{38,34}}, color={63,81,181}));
  connect(CondReheater.C_hot_out, sink_hot_CondSteam.C_in)
    annotation (Line(points={{-4,26},{-4,8},{-2,8}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                   Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-20},{100,100}})));
end TestCondReheater;
