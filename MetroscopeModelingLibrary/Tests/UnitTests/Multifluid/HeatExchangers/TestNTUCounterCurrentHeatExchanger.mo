within MetroscopeModelingLibrary.Tests.UnitTests.Multifluid.HeatExchangers;
model TestNTUCounterCurrentHeatExchanger
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  package MoistAirMedium =
      MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source moistAirSource(h_out(
        start=283945), h_vol(start=283945))
    annotation (Placement(transformation(extent={{-72,70},{-52,90}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink moistAirSink(h_in(
        start=283945), h_vol(start=283945))
    annotation (Placement(transformation(extent={{66,70},{86,90}})));
  MetroscopeModelingLibrary.Multifluid.HeatExchangers.NTUCounterCurrentHeatExchanger
    nTUCounterCurrentHeatExchanger(redeclare package HotMedium =
        MoistAirMedium,redeclare package ColdMedium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{-8,70},{12,90}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source waterSource
    annotation (Placement(transformation(extent={{-40,106},{-20,126}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink waterSink
    annotation (Placement(transformation(extent={{18,36},{38,56}})));
equation
  //inlets
  moistAirSource.P_out = 0.989e5;
  moistAirSource.Q_out = -2;
  moistAirSource.T_vol = 20 + 273.15;
  moistAirSource.relative_humidity = 0.4;
  waterSource.P_out = 10e5;
  waterSource.Q_out = -100;
  waterSource.T_vol = 5+273.15;
  //outlets
  moistAirSink.T_vol = 40 + 273.15;
  moistAirSink.Xi_vol = {0.01};
  waterSink.T_vol = 10+273.15;
  //exchanger
  nTUCounterCurrentHeatExchanger.S = 10;//1e4;//


  //*** direct parameters***
  //nTUCounterCurrentHeatExchanger.K = 1e2; // heat transfer coefficient is given by an outlet temperature or enthalpy (whether hot or cold)
  //nTUCounterCurrentHeatExchanger.K_friction_cold = 0;// pressure loss coefficient for the cold line is given by cold outlet pressure
  //nTUCounterCurrentHeatExchanger.K_friction_hot = 0;// same, but for the hot line
  //*** reverse parameters***
  nTUCounterCurrentHeatExchanger.hotSide.T_out = 287.3;
  //nTUCounterCurrentHeatExchanger.coldSide.T_out = 5.028 + 273.15;//ou h_out = 29042;
  nTUCounterCurrentHeatExchanger.coldSide.P_out =10e5;
  nTUCounterCurrentHeatExchanger.hotSide.P_out = 0.989e5;

  connect(moistAirSource.C_out, nTUCounterCurrentHeatExchanger.C_hot_in)
    annotation (Line(points={{-52,80},{-30,80},{-30,80.2},{-7.8,80.2}}, color={238,
          46,47}));
  connect(nTUCounterCurrentHeatExchanger.C_hot_out, moistAirSink.C_in)
    annotation (Line(points={{11.8,80},{66,80}}, color={238,46,47}));
  connect(waterSource.C_out, nTUCounterCurrentHeatExchanger.C_cold_in)
    annotation (Line(points={{-20,116},{2,116},{2,90}}, color={238,46,47}));
  connect(nTUCounterCurrentHeatExchanger.C_cold_out, waterSink.C_in)
    annotation (Line(points={{2,70.2},{2,46},{18,46}}, color={238,46,47}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,20},{100,140}})));
end TestNTUCounterCurrentHeatExchanger;
