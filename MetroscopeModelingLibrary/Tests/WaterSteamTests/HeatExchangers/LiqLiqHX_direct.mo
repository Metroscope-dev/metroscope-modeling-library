within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model LiqLiqHX_direct

  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

    // Boundary conditions
  input Real P_hot_source(start=50, min=0, nominal=10) "barA";
  input Units.MassFlowRate Q_hot_source(start=50) "kg/s";
  input Real T_hot_source(start = 100, min = 0, nominal = 50) "degC";

  input Real P_cold_source(start=20, min=0, nominal=10) "barA";
  input Units.MassFlowRate Q_cold_source(start=100) "kg/s";
  input Real T_cold_source(start = 50, min = 0, nominal = 50) "degC";

    // Parameters
  parameter Units.Area S = 100;
  parameter Units.HeatExchangeCoefficient Kth = 500;
  parameter Units.FrictionCoefficient Kfr_hot = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 20;

  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  WaterSteam.HeatExchangers.LiqLiqHX liqLiqHX
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

  hot_source.P_out = P_hot_source * 1e5;
  hot_source.T_out = 273.15 + T_hot_source;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  liqLiqHX.S = S;
  liqLiqHX.Kth = Kth;
  liqLiqHX.Kfr_hot = Kfr_hot;
  liqLiqHX.Kfr_cold = Kfr_cold;

  connect(liqLiqHX.C_cold_out, cold_sink.C_in) annotation (Line(
      points={{16,0},{45,0}},
      color={28,108,200}));
  connect(hot_sink.C_in, liqLiqHX.C_hot_out) annotation (Line(points={{8.88178e-16,
          -25},{8.88178e-16,-20.5},{0,-20.5},{0,-8}}, color={28,108,200}));
  connect(hot_source.C_out, liqLiqHX.C_hot_in) annotation (Line(points={{-8.88178e-16,
          25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
  connect(cold_source.C_out, liqLiqHX.C_cold_in)
    annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,80}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end LiqLiqHX_direct;
