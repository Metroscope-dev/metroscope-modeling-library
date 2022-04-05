within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model Reheater_direct

  extends Modelica.Icons.Example;

  // Boundary conditions
  input Units.Pressure P_hot_source(start=11e5, min=0, nominal=11e5) "Pa";
  input Units.Pressure P_cold_source(start=50e5, min=0, nominal=50e5) "Pa";
  input Units.OutletMassFlowRate Q_cold_source(start=-500) "kg/s";
  input Units.SpecificEnthalpy cold_source_h_out(start=5.5e5) "J/kg";
  input Units.SpecificEnthalpy hot_source_h_out(start=2.5e6) "J/kg";

  // Parameters
  parameter Units.Area S_tot = 100;
  parameter Units.Area level = 0.3;
  parameter Units.HeatExchangeCoefficient Kth_cond = 61e3;
  parameter Units.HeatExchangeCoefficient Kth_subc = 8e3;
  parameter Units.FrictionCoefficient Kfr_hot = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 0;

  WaterSteam.BoundaryConditions.WaterSource cold_source
    annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  WaterSteam.BoundaryConditions.WaterSink cold_sink
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  WaterSteam.HeatExchangers.Reheater    reheater
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  WaterSteam.BoundaryConditions.WaterSource hot_source annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,30})));
  WaterSteam.BoundaryConditions.WaterSink hot_sink annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-30})));
equation
  // Boundary conditions
  hot_source.P_out = P_hot_source;
  hot_source.h_out = hot_source_h_out;

  cold_source.P_out = P_cold_source;
  cold_source.h_out = cold_source_h_out;
  cold_source.Q_out = Q_cold_source;

  // Component parameters
  reheater.S_tot = S_tot;
  reheater.Kth_cond = Kth_cond;
  reheater.Kth_subc = Kth_subc;
  reheater.Kfr_hot = Kfr_hot;
  reheater.Kfr_cold = Kfr_cold;
  reheater.level = level;


  connect(reheater.C_cold_out, cold_sink.C_in)
    annotation (Line(points={{16,0},{45,0}}, color={28,108,200}));
  connect(hot_sink.C_in, reheater.C_hot_out) annotation (Line(points={{8.88178e-16,
          -25},{8.88178e-16,-20.5},{0,-20.5},{0,-8}}, color={28,108,200}));
  connect(hot_source.C_out, reheater.C_hot_in) annotation (Line(points={{-8.88178e-16,
          25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
  connect(cold_source.C_out, reheater.C_cold_in)
    annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
end Reheater_direct;
