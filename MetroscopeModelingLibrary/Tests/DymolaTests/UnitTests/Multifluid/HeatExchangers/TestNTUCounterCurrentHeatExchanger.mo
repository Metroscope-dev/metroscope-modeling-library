within MetroscopeModelingLibrary.Tests.DymolaTests.UnitTests.Multifluid.HeatExchangers;
model TestNTUCounterCurrentHeatExchanger
  import MetroscopeModelingLibrary;
  replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  replaceable package MoistAirMedium =
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
  moistAirSource.P_out = 0.989e5;
  moistAirSource.Q_out = -2;
  moistAirSource.T_vol = 20 + 273.15;
  moistAirSource.relative_humidity = 0.4;
  waterSource.P_out = 10e5;
  waterSource.Q_out = -100;
  waterSource.T_vol = 5+273.15;
  nTUCounterCurrentHeatExchanger.K = 1e4;
  nTUCounterCurrentHeatExchanger.S = 1e4;
  nTUCounterCurrentHeatExchanger.K_friction_cold = 1e2;
  nTUCounterCurrentHeatExchanger.K_friction_hot = 1e2;
  moistAirSink.T_vol = 40 + 273.15;
  moistAirSink.Xi_vol = {0.01};
  waterSink.T_vol = 10+273.15;
  connect(moistAirSource.C_out, nTUCounterCurrentHeatExchanger.C_hot_in)
    annotation (Line(points={{-52,80},{-30,80},{-30,80.2},{-7.8,80.2}}, color={238,
          46,47}));
  connect(nTUCounterCurrentHeatExchanger.C_hot_out, moistAirSink.C_in)
    annotation (Line(points={{11.8,80},{66,80}}, color={238,46,47}));
  connect(waterSource.C_out, nTUCounterCurrentHeatExchanger.C_cold_in)
    annotation (Line(points={{-20,116},{2,116},{2,90}}, color={238,46,47}));
  connect(nTUCounterCurrentHeatExchanger.C_cold_out, waterSink.C_in)
    annotation (Line(points={{2,70.2},{2,46},{18,46}}, color={238,46,47}));
    annotation (Placement(transformation(extent={{-16,42},{-4,54}})),
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,20},
            {100,140}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,20},{100,140}})));
end TestNTUCounterCurrentHeatExchanger;
