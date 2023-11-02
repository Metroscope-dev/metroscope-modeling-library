within MetroscopeModelingLibrary;
package DynamicComponents
  model MonoPhasicHX
    import MetroscopeModelingLibrary.Utilities.Units;
    import MetroscopeModelingLibrary.Utilities.Units.Inputs;

    // Water side properties
    parameter Real A_water = 680 "m2";
    parameter Real h_water = 2400 "W/m2.K";

    // Flue gases properties
    parameter Real A_fg = 2800 "m2";
    parameter Real h_fg = 80 "W/m2.K";

    // Wall properties
    parameter Real M_wall = 17800 "kg";
    parameter Real Cp_wall = 420 "J/kg.K";

    parameter Units.Temperature T_wall_avg_0;

    Units.Temperature T_water_avg;
    Units.Temperature T_fg_avg;
    Units.Temperature T_wall_avg(start=T_wall_avg_0, fixed=true);


    WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{40,30},{60,50}})));
    WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
    WaterSteam.BaseClasses.IsoPFlowModel water_side annotation (Placement(transformation(extent={{10,30},{-10,50}})));
    FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
    FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
    FlueGases.BaseClasses.IsoPFlowModel fg_side annotation (Placement(transformation(extent={{-8,-30},{12,-10}})));
  equation

    // Energy balance
    water_side.W + fg_side.W = -M_wall*Cp_wall*der(T_wall_avg);
    water_side.W = -h_water*A_water*(T_water_avg - T_wall_avg);
    fg_side.W = -h_fg*A_fg*(T_fg_avg - T_wall_avg);

    // Average temperatures
    T_water_avg = 0.5*(water_side.T_in + water_side.T_out);
    T_fg_avg = 0.5*(fg_side.T_in + fg_side.T_out);


    connect(water_side.C_in, water_inlet) annotation (Line(points={{10,40},{50,40}}, color={28,108,200}));
    connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,40},{-50,40}}, color={28,108,200}));
    connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-8,-20},{-50,-20}}, color={95,95,95}));
    connect(fg_side.C_out, fg_outlet) annotation (Line(points={{12,-20},{50,-20}}, color={95,95,95}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end MonoPhasicHX;

  model MonoPhasicHX_01_discretization
    import MetroscopeModelingLibrary.Utilities.Units;
    import MetroscopeModelingLibrary.Utilities.Units.Inputs;

    // Discretization
    parameter Integer N = 10 "number of nodes";

    // Water side properties
    parameter Real A_water = 680 "m2 total heat exchange area of water";
    parameter Real hc_water = 2400 "W/m2.K convection heat transfer coefficient of water";

    // Flue gases properties
    parameter Real A_fg = 2800 "m2 total heat exchange area of flue gases";
    parameter Real hc_fg = 80 "W/m2.K convection heat transfer coefficient of flue gases";

    // Wall properties
    parameter Real M_wall = 17800 "kg total mass of tubes";
    parameter Real Cp_wall = 420 "J/kg.K heat capacity of tubes";

    // Initialization
    parameter Units.Temperature T_wall_avg_0;

    Units.Temperature T_water[N+2];
    Units.Temperature T_water_avg[N+1];
    Units.Temperature T_fg_avg;
    Units.Temperature T_wall_avg(start=T_wall_avg_0, fixed=true);

    Units.SpecificEnthalpy h_water[N+1];
    Units.SpecificEnthalpy h_water_avg[N];


    WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{40,30},{60,50}})));
    WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
    WaterSteam.BaseClasses.IsoPFlowModel water_side_in annotation (Placement(transformation(extent={{30,30},{10,50}})));
    FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
    FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
    FlueGases.BaseClasses.IsoPFlowModel fg_side_in annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));
    WaterSteam.BaseClasses.IsoPFlowModel water_side_out annotation (Placement(transformation(extent={{-10,30},{-30,50}})));
    FlueGases.BaseClasses.IsoPFlowModel fg_side_out annotation (Placement(transformation(extent={{10,-30},{30,-10}})));
  equation

    // Boundaries
    h_water[1] = water_side_in.h_in;
    h_water[N+1] = water_side_in.h_out;

    // Energy balance
    //for i in 2:N loop




    water_side_in.W + fg_side_in.W = -M_wall*Cp_wall*der(T_wall_avg);
    water_side_in.W = -hc_water*A_water*(T_water_avg - T_wall_avg);
    fg_side_in.W = -hc_fg*A_fg*(T_fg_avg - T_wall_avg);

    // Average temperatures
    T_water_avg =0.5*(water_side_in.T_in + water_side_in.T_out);
    T_fg_avg =0.5*(fg_side_in.T_in + fg_side_in.T_out);

    connect(water_side_in.C_in, water_inlet) annotation (Line(points={{30,40},{50,40}}, color={28,108,200}));
    connect(fg_side_in.C_in, fg_inlet) annotation (Line(points={{-30,-20},{-50,-20}}, color={95,95,95}));
    connect(water_side_in.C_out, water_side_out.C_in) annotation (Line(points={{10,40},{-10,40}}, color={28,108,200}));
    connect(water_side_out.C_out, water_outlet) annotation (Line(points={{-30,40},{-50,40}}, color={28,108,200}));
    connect(fg_side_in.C_out, fg_side_out.C_in) annotation (Line(points={{-10,-20},{10,-20}}, color={95,95,95}));
    connect(fg_side_out.C_out, fg_outlet) annotation (Line(points={{30,-20},{50,-20}}, color={95,95,95}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end MonoPhasicHX_01_discretization;

  model MonoPhasicHX_test

      // Boundary conditions
    input Real P_hot_source(start = 1.1, min = 0, nominal = 1) "barA";
    input Utilities.Units.MassFlowRate Q_hot_source(start = 640) "kg/s";
    input Utilities.Units.Temperature T_hot_source(start = 600) "degC";

    input Real P_cold_source(start = 130, min = 1.5, nominal = 100) "barA";
    input Utilities.Units.MassFlowRate Q_cold_source(start = 85) "kg/s";
    input Real T_cold_source(start = 410, min = 130, nominal = 150) "degC";


    MonoPhasicHX monoPhasicHX(Cp_wall=600, T_wall_avg_0=746.43146)
                              annotation (Placement(transformation(extent={{-10,-10},{10,12}})));
    WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={64,20})));
    WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{-54,10},{-74,30}})));
    FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-76,-50},{-56,-30}})));
    FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{54,-50},{74,-30}})));
    Modelica.Blocks.Sources.Step step(height=150, startTime=200) annotation (Placement(transformation(extent={{-38,56},{-18,76}})));
  equation
    hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
    hot_source.P_out = P_hot_source*1e5;
    hot_source.T_out = T_hot_source + 273.15;
    hot_source.Q_out = - Q_hot_source + step.y;

    cold_source.P_out = P_cold_source*1e5;
    cold_source.T_out = 273.15 + T_cold_source;
    cold_source.Q_out = - Q_cold_source;


    connect(monoPhasicHX.water_inlet, cold_source.C_out) annotation (Line(points={{5,5.4},{5,2},{54,2},{54,20},{59,20}}, color={28,108,200}));
    connect(monoPhasicHX.water_outlet, cold_sink.C_in) annotation (Line(points={{-5,5.4},{-32,5.4},{-32,20},{-59,20}}, color={28,108,200}));
    connect(monoPhasicHX.fg_inlet, hot_source.C_out) annotation (Line(points={{-5,-1.2},{-5,-2},{-50,-2},{-50,-40},{-61,-40}}, color={95,95,95}));
    connect(monoPhasicHX.fg_outlet, hot_sink.C_in) annotation (Line(points={{5,-1.2},{5,-2},{50,-2},{50,-40},{59,-40}}, color={95,95,95}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=500,
        __Dymola_NumberOfIntervals=1000,
        __Dymola_Algorithm="Dassl"));
  end MonoPhasicHX_test;

  model MonoPhasicHX_test_TSP

      // Boundary conditions
    input Real P_hot_source(start = 1.1, min = 0, nominal = 1) "barA";
    input Utilities.Units.MassFlowRate Q_hot_source(start = 610) "kg/s";
    input Utilities.Units.Temperature T_hot_source(start = 476.85) "degC";

    input Real P_cold_source(start = 130.4, min = 1.5, nominal = 100) "barA";
    input Utilities.Units.MassFlowRate Q_cold_source(start = 150) "kg/s";
    input Real T_cold_source(start = 320.34, min = 130, nominal = 150) "degC";


    MonoPhasicHX monoPhasicHX(
      A_water=3124.5,
      h_water=9265,
      A_fg=43586.8,
      h_fg=100,
      M_wall=68513,
      Cp_wall=1000,
      T_wall_avg_0=613.15)    annotation (Placement(transformation(extent={{-10,-10},{10,12}})));
    WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={64,20})));
    WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{-54,10},{-74,30}})));
    FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-76,-50},{-56,-30}})));
    FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{54,-50},{74,-30}})));
  equation
    hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
    hot_source.P_out = P_hot_source*1e5;
    hot_source.T_out = T_hot_source + 273.15;
    hot_source.Q_out = - Q_hot_source;

    cold_source.P_out = P_cold_source*1e5;
    cold_source.T_out = 273.15 + T_cold_source;
    cold_source.Q_out = - Q_cold_source;

    connect(monoPhasicHX.water_inlet, cold_source.C_out) annotation (Line(points={{5,5.4},{5,2},{54,2},{54,20},{59,20}}, color={28,108,200}));
    connect(monoPhasicHX.water_outlet, cold_sink.C_in) annotation (Line(points={{-5,5.4},{-32,5.4},{-32,20},{-59,20}}, color={28,108,200}));
    connect(monoPhasicHX.fg_inlet, hot_source.C_out) annotation (Line(points={{-5,-1.2},{-5,-2},{-50,-2},{-50,-40},{-61,-40}}, color={95,95,95}));
    connect(monoPhasicHX.fg_outlet, hot_sink.C_in) annotation (Line(points={{5,-1.2},{5,-2},{50,-2},{50,-40},{59,-40}}, color={95,95,95}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=500,
        __Dymola_NumberOfIntervals=1000,
        __Dymola_Algorithm="Dassl"));
  end MonoPhasicHX_test_TSP;
  annotation (Icon(graphics={Line(points={{-56,72}}, color={28,108,200}), Line(
          points={{-100,0},{-50,100},{50,-100},{100,0}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier)}));
end DynamicComponents;
