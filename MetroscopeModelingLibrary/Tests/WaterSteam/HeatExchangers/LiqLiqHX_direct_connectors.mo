within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model LiqLiqHX_direct_connectors

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

    // Boundary conditions
  input Real P_hot_source(start=50, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_hot_source(start=50) "kg/s";
  input Real T_hot_source(start = 100, min = 0, nominal = 50) "degC";

  input Real P_cold_source(start=20, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start=100) "kg/s";
  input Real T_cold_source(start = 50, min = 0, nominal = 50) "degC";

    // Parameters
  parameter Utilities.Units.Area S=100;

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.LiqLiqHX liqLiqHX annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,30})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-30})));
  Utilities.Interfaces.RealOutput Kfr_cold annotation (Placement(transformation(
          extent={{-38,16},{-30,24}}), iconTransformation(extent={{-226,-82},{-206,
            -62}})));
  Utilities.Interfaces.RealOutput Kth annotation (Placement(transformation(
          extent={{-30,40},{-22,48}}), iconTransformation(extent={{-224,-62},{-204,
            -42}})));
  Utilities.Interfaces.RealOutput Kfr_hot annotation (Placement(transformation(
          extent={{32,26},{40,34}}), iconTransformation(extent={{-224,-54},{-204,
            -34}})));
equation

  hot_source.P_out = P_hot_source * 1e5;
  hot_source.T_out = 273.15 + T_hot_source;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  Kth = 500;
  Kfr_hot = 0;
  Kfr_cold = 20;

  connect(liqLiqHX.C_cold_out, cold_sink.C_in) annotation (Line(
      points={{16,0},{45,0}},
      color={28,108,200}));
  connect(hot_sink.C_in, liqLiqHX.C_hot_out) annotation (Line(points={{8.88178e-16,
          -25},{8.88178e-16,-20.5},{0,-20.5},{0,-8}}, color={28,108,200}));
  connect(hot_source.C_out, liqLiqHX.C_hot_in) annotation (Line(points={{-8.88178e-16,
          25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
  connect(cold_source.C_out, liqLiqHX.C_cold_in)
    annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
  connect(liqLiqHX.Kfr_cold, Kfr_cold)
    annotation (Line(points={{-18,3.2},{-34,3.2},{-34,20}}, color={0,0,127}));
  connect(liqLiqHX.Kth, Kth)
    annotation (Line(points={{-9.6,10},{-26,10},{-26,44}}, color={0,0,127}));
  connect(liqLiqHX.Kfr_hot, Kfr_hot)
    annotation (Line(points={{4.8,10},{36,10},{36,30}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,80}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end LiqLiqHX_direct_connectors;
