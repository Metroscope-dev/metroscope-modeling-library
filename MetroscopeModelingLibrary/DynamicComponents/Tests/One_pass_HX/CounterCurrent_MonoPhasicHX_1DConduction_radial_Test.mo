within MetroscopeModelingLibrary.DynamicComponents.Tests.One_pass_HX;
model CounterCurrent_MonoPhasicHX_1DConduction_radial_Test
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  HeatExchangers.One_pass_HX.CounterCurrent_MonoPhasicHX_1DConduction_radial HX(
    N_tubes_row=184,
    Rows=2,
    Tubes_Config=2,
    fg_path_width=14.07,
    N=10,
    D_out=0.0381,
    e=0.003048,
    L=18.29,
    A_water=676.73035,
    T_wall_0=745.15) annotation (Placement(transformation(extent={{-10,-6},{10,16}})));

    // Boundary conditions
  input Real P_hot_source(start = 1, min = 0, nominal = 1) "barA";
  input Real Q_hot_source(start = 658.695) "kg/s";
  input Utilities.Units.Temperature T_hot_source(start = 633.7) "degC";

  input Real P_cold_source(start = 121.2, min = 1.5, nominal = 100) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start = 84.06) "kg/s";
  input Real T_cold_source(start = 498.8, min = 130, nominal = 150) "degC";

  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={64,20})));
  WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{-54,10},{-74,30}})));
  FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-76,-50},{-56,-30}})));
  FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{54,-50},{74,-30}})));
  Modelica.Blocks.Sources.Step step(height=100, startTime=100) annotation (Placement(transformation(extent={{-72,66},{-52,86}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=-20,
    duration=60,
    startTime=300) annotation (Placement(transformation(extent={{-6,66},{14,86}})));
equation
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source*1e5;
  hot_source.T_out = T_hot_source + 273.15 + ramp.y;
  hot_source.Q_out = - Q_hot_source + step.y;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;
  connect(HX.water_inlet, cold_source.C_out) annotation (Line(points={{10,9.4},{10,8},{48,8},{48,20},{59,20}}, color={28,108,200}));
  connect(HX.water_outlet, cold_sink.C_in) annotation (Line(points={{-10,9.62},{-10,8},{-48,8},{-48,20},{-59,20}}, color={28,108,200}));
  connect(HX.fg_inlet, hot_source.C_out) annotation (Line(points={{-10,0.6},{-50,0.6},{-50,-40},{-61,-40}}, color={95,95,95}));
  connect(HX.fg_outlet, hot_sink.C_in) annotation (Line(points={{9.8,0.6},{50,0.6},{50,-40},{59,-40}}, color={95,95,95}));
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
end CounterCurrent_MonoPhasicHX_1DConduction_radial_Test;
