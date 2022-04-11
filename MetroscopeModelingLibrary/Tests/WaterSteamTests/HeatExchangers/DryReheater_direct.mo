within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model DryReheater_direct

  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Real P_hot_source(start=11, min=0, nominal=11) "bar";
  input Real P_cold_source(start=50, min=0, nominal=50) "bar";
  input Units.InletMassFlowRate Q_cold(start=500) "kg/s";
  input Real T_cold_in(start=50) "degC";
  input Units.SpecificEnthalpy hot_source_h_out(start=2.5e6) "J/kg";

  // Parameters
  parameter Units.Area S = 100;
  parameter Units.HeatExchangeCoefficient Kth = 50e3;
  parameter Units.FrictionCoefficient Kfr_hot = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 500; // About 1 bar of pressure loss in the reheater

  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  WaterSteam.HeatExchangers.DryReheater dryReheater
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,30})));
  WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-30})));
equation

  // Boundary conditions
  hot_source.P_out = P_hot_source*1e5;
  hot_source.h_out = hot_source_h_out;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = T_cold_in + 273.15;
  cold_source.Q_out = -Q_cold;

  // Component parameters
  dryReheater.S_condensing = S;
  dryReheater.Kth = Kth;
  dryReheater.Kfr_hot = Kfr_hot;
  dryReheater.Kfr_cold = Kfr_cold;


  connect(dryReheater.C_cold_out, cold_sink.C_in)
    annotation (Line(points={{16,0},{45,0}}, color={28,108,200}));
  connect(hot_sink.C_in, dryReheater.C_hot_out) annotation (Line(points={{8.88178e-16,
          -25},{8.88178e-16,-20.5},{0,-20.5},{0,-8}}, color={28,108,200}));
  connect(hot_source.C_out, dryReheater.C_hot_in) annotation (Line(points={{-8.88178e-16,
          25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
  connect(cold_source.C_out, dryReheater.C_cold_in)
    annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
end DryReheater_direct;
