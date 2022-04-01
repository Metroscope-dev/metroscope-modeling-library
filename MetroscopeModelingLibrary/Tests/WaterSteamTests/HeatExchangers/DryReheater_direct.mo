within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model DryReheater_direct

  extends Modelica.Icons.Example;

  /*  // Boundary conditions
  input Real P_hot_source(start=50, min=0, nominal=10) "barA";
  input Units.MassFlowRate Q_hot_source(start=50) "kg/s";
  input Real T_hot_source(start = 100, min = 0, nominal = 50) "degC";

  input Real P_cold_source(start=20, min=0, nominal=10) "barA";
  input Units.MassFlowRate Q_cold_source(start=100) "kg/s";
  input Real T_cold_source(start = 50, min = 0, nominal = 50) "degC";
  */

    // Parameters
  parameter Units.Area S = 100;
  parameter Units.HeatExchangeCoefficient Kth = 61e3;
  parameter Units.FrictionCoefficient Kfr_hot = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 0;

  WaterSteam.BoundaryConditions.WaterSource cold_source
    annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  WaterSteam.BoundaryConditions.WaterSink cold_sink
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  WaterSteam.HeatExchangers.DryReheater
                                     dryReheater
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

  hot_source.P_out = 11e5;
  hot_source.h_out = 2.5e6;

  cold_source.P_out = 50e5;
  cold_source.h_out = 549552;
  cold_source.Q_out = - 500;

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
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,80}})),Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end DryReheater_direct;
