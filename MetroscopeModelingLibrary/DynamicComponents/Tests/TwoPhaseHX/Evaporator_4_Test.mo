within MetroscopeModelingLibrary.DynamicComponents.Tests.TwoPhaseHX;
model Evaporator_4_Test
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  HeatExchangers.TwoPhaseHX.Evaporator_4                                         HX(
    N_tubes_row=1560,
    Rows=1,
    Tubes_Config=2,
    fg_path_width=11161e-3,
    S_T=89e-3,
    S_L=89e-3,
    S_f=1/245,
    H_fin=19e-3,
    e_fin=0.8e-3,
    eff_fins=0.7704,
    M_wall=65409 + 65468 + 65875.2 + 85765,
    N=10,
    D_out=38e-3,
    e=2.6e-3,
    L=21930e-3,
    T_wall_0=673.15) annotation (Placement(transformation(extent={{-10,-12},{10,10}})));

    // Boundary conditions
  input Real P_hot_source(start = 1, min = 0, nominal = 1) "barA";
  input Real Q_hot_source(start = 606.207) "kg/s"; // 658
  input Utilities.Units.Temperature T_hot_source(start = 400) "degC";

  input Real P_cold_source(start = 100e5, min = 1.5, nominal = 100) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start = 300) "kg/s";
  //input Real T_cold_source(start = 498.8, min = 130, nominal = 150) "degC";

  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={64,40})));
  WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-32,40})));
  FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
  FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{72,-10},{92,10}})));
  Modelica.Blocks.Sources.Step step(height=20, startTime=50)   annotation (Placement(transformation(extent={{-78,52},{-58,72}})));
equation
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source*1e5;
  hot_source.T_out = T_hot_source + 273.15 - step.y;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source;
  //cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;
  //cold_source.h_out = WaterSteamMedium.bubbleEnthalpy(sat.P_cold_source);
  //cold_source.h_out = HX.h_l;
  HX.x_in = 0;

  connect(HX.water_inlet, cold_source.C_out) annotation (Line(points={{4,10},{4,28},{48,28},{48,40},{59,40}},  color={28,108,200}));
  connect(HX.water_outlet, cold_sink.C_in) annotation (Line(points={{-4,10},{-4,26},{-32,26},{-32,35}},            color={28,108,200}));
  connect(HX.fg_inlet, hot_source.C_out) annotation (Line(points={{-8,-1},{-8,0},{-73,0}},                  color={95,95,95}));
  connect(HX.fg_outlet, hot_sink.C_in) annotation (Line(points={{8,-1},{8,0},{77,0}},                  color={95,95,95}));
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
end Evaporator_4_Test;
