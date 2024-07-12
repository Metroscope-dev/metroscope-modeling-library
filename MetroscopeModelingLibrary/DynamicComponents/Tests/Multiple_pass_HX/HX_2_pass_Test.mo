within MetroscopeModelingLibrary.DynamicComponents.Tests.Multiple_pass_HX;
model HX_2_pass_Test
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

    // Boundary conditions
  input Real P_hot_source(start = 1, min = 0, nominal = 1) "barA";
  input Real Q_hot_source(start = 658.695) "kg/s";
  input Utilities.Units.Temperature T_hot_source(start = 633.7) "degC";

  input Real P_cold_source(start = 121.2, min = 1.5, nominal = 100) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start = 84.06) "kg/s";
  input Real T_cold_source(start = 498.8, min = 130, nominal = 150) "degC";

  FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
  FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
  Modelica.Blocks.Sources.Step step(height=100, startTime=100) annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=-20,
    duration=60,
    startTime=300) annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
  HeatExchangers.Multiple_pass_HX.HX_2_pass hX_2_pass annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-44,40})));
  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={44,40})));
equation
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source*1e5;
  hot_source.T_out = T_hot_source + 273.15 + ramp.y;
  hot_source.Q_out = - Q_hot_source + step.y;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  connect(hot_source.C_out, hX_2_pass.fg_inlet) annotation (Line(points={{-39,0},{-10,0}}, color={95,95,95}));
  connect(hX_2_pass.fg_outlet, hot_sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={95,95,95}));
  connect(hX_2_pass.inlet, cold_source.C_out) annotation (Line(points={{6,10},{6,40},{39,40}}, color={28,108,200}));
  connect(hX_2_pass.outlet, cold_sink.C_in) annotation (Line(points={{-6,10},{-6,40},{-39,40}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor={0,140,72},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor={0,140,72},
                fillColor={0,140,72},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-28,64},{72,4},{-28,-56},{-28,64}})}),  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},{60,60}})),
    experiment(
      StopTime=500,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end HX_2_pass_Test;
