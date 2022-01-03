within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.HeatExchangers;
model TestReheater
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater Reheater
    annotation (Placement(transformation(extent={{12,26},{-20,42}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_hot_CondSteam
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
equation

  // Forward causality
  /*
  //inlets
  source_hot_SatSteam.h_out = 2.3e6;
  source_hot_SatSteam.P_out = 15e5;
  //source_hot_SatSteam.Q_out = -60;

  source_cold_coldwater.P_out = 70e5;
  source_cold_coldwater.T_out = 180 + 273.15;
  source_cold_coldwater.Q_out = -1500;

  //outlets
  sink_cold_hotwater.h_vol = 0.9e6;
  //sink_cold_hotwater.T_in = 190 + 273.15;

  sink_hot_CondSteam.h_vol = 1e6;
  //sink_hot_CondSteam.T_in = 185+273.15;


  //heatexchangers
  Reheater.Level=0.3;
  Reheater.S_tot=100;
  Reheater.Kfr_hot=1;
  Reheater.Kfr_cold=1;
  Reheater.Kth_cond=72e3;
  Reheater.Kth_purge=8500;
  */


  // Reverse causality
  // determine Kth_cond and Kth_purge by giving both outlet temperatures

  //inlets
  source_hot_SatSteam.h_out = 2.3e6;
  source_hot_SatSteam.P_out = 15e5;
  //source_hot_mainSt.Q_out = -60;

  source_cold_coldwater.P_out = 70e5;
  source_cold_coldwater.T_out = 180 + 273.15;
  source_cold_coldwater.Q_out = -1500;

  //outlets
  sink_cold_hotwater.h_vol = 0.9e6;
  sink_cold_hotwater.T_in = 190 + 273.15;

  sink_hot_CondSteam.h_vol = 1e6;
  sink_hot_CondSteam.T_in = 185+273.15;


  //heatexchangers
  Reheater.Level=0.3;
  Reheater.S_tot=100;
  Reheater.Kfr_hot=1;
  Reheater.Kfr_cold=1;

  connect(source_hot_SatSteam.C_out, Reheater.C_hot_in)
    annotation (Line(points={{-4,68},{-4,41.8},{-4,41.8}}, color={63,81,181}));
  connect(source_cold_coldwater.C_out, Reheater.C_cold_in)
    annotation (Line(points={{38,34},{12,34}}, color={63,81,181}));
  connect(sink_cold_hotwater.C_in, Reheater.C_cold_out) annotation (Line(points=
         {{-48,36},{-34,36},{-34,34},{-20,34}}, color={63,81,181}));
  connect(sink_hot_CondSteam.C_in, Reheater.C_hot_out)
    annotation (Line(points={{-2,8},{-4,8},{-4,26}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end TestReheater;
