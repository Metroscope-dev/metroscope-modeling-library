within MetroscopeModelingLibrary.DynamicComponents.Tests.Simplified;
model Simplified_model_discretized_B
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

    // Boundary conditions
  input Real P_hot_source(start = 1, min = 0, nominal = 1) "barA";
  input Real Q_hot_source(start = 658.695) "kg/s";
  input Utilities.Units.Temperature T_hot_source(start = 633.7) "degC";

  input Real P_cold_source(start = 121.2, min = 1.5, nominal = 100) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start = 84.06) "kg/s";
  input Real T_cold_source(start = 498.8, min = 130, nominal = 150) "degC";

  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-86})));
  WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=270,
        origin={0,84})));
  FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
  FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{74,-10},{94,10}})));
  Modelica.Blocks.Sources.Step step(height=100, startTime=100) annotation (Placement(transformation(extent={{-72,66},{-52,86}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=-20,
    duration=60,
    startTime=300) annotation (Placement(transformation(extent={{58,70},{78,90}})));
  HeatExchangers.Simplified.Simplified_model_discretized HX(
    UA=155746.7,
    r_UA=8.596588,
    N=1,
    T_wall_0=790.15) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source*1e5;
  hot_source.T_out = T_hot_source + 273.15 + ramp.y;
  hot_source.Q_out = - Q_hot_source + step.y;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  connect(hot_source.C_out, HX.fg_inlet) annotation (Line(points={{-79,0},{-4,0}}, color={95,95,95}));
  connect(HX.fg_outlet, hot_sink.C_in) annotation (Line(points={{4,0},{79,0}}, color={95,95,95}));
  connect(HX.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-45.5},{2.77556e-16,-45.5},{2.77556e-16,-81}}, color={28,108,200}));
  connect(HX.water_outlet, cold_sink.C_in) annotation (Line(points={{0,10},{0,44.5},{-9.4369e-16,44.5},{-9.4369e-16,79}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor={0,140,72},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor={0,140,72},
                fillColor={0,140,72},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=500,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end Simplified_model_discretized_B;
