within MetroscopeModelingLibrary.DynamicComponents.Tests.Monophasic_HeatExchanger;
model Monophasic_Dynamic_HX_1_Test
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  HeatExchangers.Monophasic_HeatExchanger.Monophasic_Dynamic_HX_1                HX(
    steady_state=false,
    N_tubes_row=184,
    Rows=2,
    Tubes_Config=2,
    fg_path_width=14.07,
    N=10,
    D_out=0.0381,
    e=0.003048,
    L=18.29,
    A_water=676.73035,
    T_wall_0=745.15) annotation (Placement(transformation(extent={{-10,-10},{10,12}})));

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
        origin={0,-82})));
  WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=270,
        origin={0,80})));
  FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
  FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{72,-10},{92,10}})));
  Sensors.Displayer.WaterDisplayer waterDisplayer annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-10,34})));
  Sensors.Displayer.WaterDisplayer waterDisplayer1 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-12,-30})));
  Sensors.Displayer.FlueGasesDisplayer flueGasesDisplayer annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  Sensors.Displayer.FlueGasesDisplayer flueGasesDisplayer1 annotation (Placement(transformation(extent={{34,-10},{54,10}})));
  Modelica.Blocks.Sources.Ramp ramp(
    height=-20,
    duration=2,
    startTime=50)  annotation (Placement(transformation(extent={{40,60},{60,80}})));
equation
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source*1e5;
  hot_source.T_out = T_hot_source + 273.15 - ramp.y;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  HX.k_corr = 1;

  connect(HX.water_outlet, waterDisplayer.C_in) annotation (Line(points={{0,12},{0,34},{-7,34}}, color={28,108,200}));
  connect(waterDisplayer.C_out, cold_sink.C_in) annotation (Line(points={{-13,34},{-13,54},{0,54},{0,75}}, color={28,108,200}));
  connect(HX.water_inlet, waterDisplayer1.C_out) annotation (Line(points={{0,-10},{0,-16},{-26,-16},{-26,-30},{-15,-30}}, color={28,108,200}));
  connect(waterDisplayer1.C_in, cold_source.C_out) annotation (Line(points={{-9,-30},{0,-30},{0,-77}},                 color={28,108,200}));
  connect(HX.fg_inlet, flueGasesDisplayer.C_out) annotation (Line(points={{-4,1},{-4,0},{-37,0}}, color={95,95,95}));
  connect(flueGasesDisplayer.C_in, hot_source.C_out) annotation (Line(points={{-43,0},{-73,0}}, color={95,95,95}));
  connect(HX.fg_outlet, flueGasesDisplayer1.C_in) annotation (Line(points={{4,1},{4,0},{41,0}}, color={95,95,95}));
  connect(flueGasesDisplayer1.C_out, hot_sink.C_in) annotation (Line(points={{47,0},{77,0}}, color={95,95,95}));
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
      StopTime=200,
      __Dymola_NumberOfIntervals=1000,
      __Dymola_Algorithm="Dassl"));
end Monophasic_Dynamic_HX_1_Test;
