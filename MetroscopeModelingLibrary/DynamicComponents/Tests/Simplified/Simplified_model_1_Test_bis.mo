within MetroscopeModelingLibrary.DynamicComponents.Tests.Simplified;
model Simplified_model_1_Test_bis
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  HeatExchangers.Simplified.Simplified_model_1                                   HX(
    T_wall_0=1073.15,
    T_water_out_0=790.6686,
    T_fg_out_0=901.4944)
                     annotation (Placement(transformation(extent={{-10,-10},{10,12}})));

    // Boundary conditions
  input Real P_hot_source(start = 1, min = 0, nominal = 1) "barA";
  input Real Q_hot_source(start = 658.695) "kg/s";
  input Utilities.Units.Temperature T_hot_source(start = 633.7) "degC";

  input Real P_cold_source(start = 121.2, min = 1.5, nominal = 100) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start = 84.06) "kg/s";
  input Real T_cold_source(start = 498.8, min = 130, nominal = 150) "degC";

  input Real T_water_out(start = 565) "degC";

  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-82})));
  WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=270,
        origin={0,84})));
  FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
  FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{72,-10},{92,10}})));
equation
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source*1e5;
  hot_source.T_out = T_hot_source + 273.15;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  HX.T_water_out = T_water_out + 273.15;

  connect(HX.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-77}},                        color={28,108,200}));
  connect(HX.water_outlet, cold_sink.C_in) annotation (Line(points={{0,12},{0,79}},                                color={28,108,200}));
  connect(HX.fg_inlet, hot_source.C_out) annotation (Line(points={{-4,1},{-4,0},{-73,0}},                   color={95,95,95}));
  connect(HX.fg_outlet, hot_sink.C_in) annotation (Line(points={{4,1},{4,0},{77,0}},                   color={95,95,95}));
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
end Simplified_model_1_Test_bis;
