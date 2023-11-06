within MetroscopeModelingLibrary;
package DynamicComponents
  package HeatExchangers
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

    model MonoPhasicHX_nodes
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;

      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      // Discretization
      parameter Integer N = 10;

      // Water side properties
      parameter Real A_water = 680 "m2";
      parameter Real hc_water = 2400 "W/m2.K";

      // Flue gases properties
      parameter Real A_fg = 2800 "m2";
      parameter Real hc_fg = 80 "W/m2.K";

      // Wall properties
      parameter Real M_wall = 17800 "kg";
      parameter Real Cp_wall = 420 "J/kg.K";

      // Initialization
      parameter Units.Temperature T_wall_0 = 450;
      parameter Units.Pressure P_water_0 = 70e5;
      parameter Units.Pressure P_fg_0 = 1e5;
      parameter Units.PositiveMassFlowRate Q_water_0 = 85;
      parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
      parameter Units.Temperature T_water_out_0 = 500;
      parameter Units.Temperature T_fg_out_0 = 560;
      parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
      parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;

      // Discretization
      parameter Real dM_wall = M_wall/N;
      parameter Real dA_water = A_water/N;
      parameter Real dA_fg = A_fg/N;

      // Enthalpies
      Units.SpecificEnthalpy h_water[N+1] "Water specific enthalpy";
      Units.SpecificEnthalpy h_fg[N+1] "Flue gas specific enthalpy";
      // Mass flow rate
      Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
      Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
      // Pressures
      Units.Pressure P_water(start=P_water_0) "Water Pressure";
      Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
      // Mass fractions
      Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
      Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";

      // Temperatures
      Units.Temperature T_water[N+1] "Node boundary water temperature";
      Units.Temperature T_water_avg[N] "Node average water temperature";
      Units.Temperature T_fg[N+1] "Node boundary flue gas temperature";
      Units.Temperature T_fg_avg[N] "Node average flue gas temperature";
      Units.Temperature T_wall[N](each start=T_wall_0, fixed=true) "Node wall temperature";

      // ------ States ------
      WaterSteamMedium.ThermodynamicState state_water[N+1];
      FlueGasesMedium.ThermodynamicState state_fg[N+1];

      // ------ Conservation variables ------
      Units.Power dW_water[N] "Node water heat exchange";
      Units.Power dW_fg[N] "Node flue gas heat exchange";

      WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{40,20},{60,40}}), iconTransformation(extent={{40,20},{60,40}})));
      WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-60,20},{-40,40}}), iconTransformation(extent={{-60,20},{-40,40}})));
      FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{40,-40},{60,-20}}), iconTransformation(extent={{40,-40},{60,-20}})));
      FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-60,-40},{-40,-20}}), iconTransformation(extent={{-60,-40},{-40,-20}})));
      WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,20},{-10,40}})));
      FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
    equation

      // Boundaries
      // Enthalpies
      h_water[1] = water_side.h_in;
      h_water[N+1] = water_side.h_out;
      h_fg[1] = fg_side.h_out;
      h_fg[N+1] = fg_side.h_in;

      // Pressures
      P_water = water_side.P_in;
      P_fg = fg_side.P_in;

      // Mass flow rate
      Q_water = water_side.Q;
      Q_fg = fg_side.Q;

      // Mass Fractions
      Xi_water = water_side.Xi;
      Xi_fg = fg_side.Xi;

      // First node states
      state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
      state_fg[1] = FlueGasesMedium.setState_phX(P_fg, h_fg[1], Xi_fg);
      T_water[1] = WaterSteamMedium.temperature(state_water[1]);
      T_fg[1] = FlueGasesMedium.temperature(state_fg[1]);

      for i in 1:N loop
        // ------ States ------
        state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
        state_fg[i+1] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1], Xi_fg);

        // ------ Computed Quantities ------
        // Temperatures
        T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
        T_fg[i+1] = FlueGasesMedium.temperature(state_fg[i+1]);

        // ------ Conservation equations ------
        dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
        dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);

        // ------ Heat transfer equations ------
        dW_water[i] + dW_fg[i] = -dM_wall*Cp_wall*der(T_wall[i]);
        dW_water[i] = -hc_water*dA_water*(T_water_avg[i] - T_wall[i]);
        dW_fg[i] = -hc_fg*dA_fg*(T_fg_avg[i] - T_wall[i]);

        // ------- Average temperatures ------
        T_water_avg[i] = 0.5*(T_water[i] + T_water[i+1]);
        T_fg_avg[i] = 0.5*(T_fg[i] + T_fg[i+1]);
        end for;

      connect(water_side.C_in, water_inlet) annotation (Line(points={{10,30},{50,30}}, color={28,108,200}));
      connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,30},{-50,30}}, color={28,108,200}));
      connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,-30},{50,-30}}, color={95,95,95}));
      connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,-30},{-50,-30}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-50,50},{50,10}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-30,50},{-30,10}}, color={0,0,0}),
            Line(points={{-20,50},{-20,10}}, color={0,0,0}),
            Line(points={{-10,50},{-10,10}}, color={0,0,0}),
            Line(points={{0,50},{0,10}}, color={0,0,0}),
            Line(points={{10,50},{10,10}}, color={0,0,0}),
            Line(points={{20,50},{20,10}}, color={0,0,0}),
            Line(points={{30,50},{30,10}}, color={0,0,0}),
            Rectangle(
              extent={{-50,-10},{50,-50}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Line(points={{-30,-10},{-30,-50}}, color={0,0,0}),
            Line(points={{-20,-10},{-20,-50}}, color={0,0,0}),
            Line(points={{-10,-10},{-10,-50}}, color={0,0,0}),
            Line(points={{0,-10},{0,-50}}, color={0,0,0}),
            Line(points={{10,-10},{10,-50}}, color={0,0,0}),
            Line(points={{20,-10},{20,-50}}, color={0,0,0}),
            Line(points={{30,-10},{30,-50}}, color={0,0,0}),
            Line(points={{40,50},{40,10}}, color={0,0,0}),
            Line(points={{-40,50},{-40,10}}, color={0,0,0}),
            Line(points={{-40,-10},{-40,-50}}, color={0,0,0}),
            Line(points={{40,-10},{40,-50}}, color={0,0,0}),
            Rectangle(
              extent={{-50,10},{50,-10}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-50,10},{50,-10}},
              textColor={0,0,0},
              fontSize=20,
              textString="Tube Wall"),
            Line(
              points={{30,30},{-30,30}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Line(
              points={{-30,-30},{30,-30}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5)}),                                      Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MonoPhasicHX_nodes;

    model MonoPhasicHX_nodes_Geometry
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;

      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      // Discretization
      parameter Integer N = 10;

      // Geometry
      // Pipes
      parameter Units.DifferentialHeight D_out = 0.03 "Pipe outer diameter";
      parameter Units.DifferentialHeight e = 0.003 "Pipe wall thickness";
      parameter Units.DifferentialHeight D_in = D_out - 2*e "Pipe inner diameter";
      parameter Units.DifferentialHeight L = 22 "Tube length";
      parameter Integer N_tubes = 180 "Number of tubes";

      // Water side properties
      parameter Units.Area A_water = N_tubes*L*Modelica.Constants.pi*D_in "water side heat exchange surface";
      parameter Units.HeatExchangeCoefficient hc_water = 2430 "W/m2.K";

      // Flue gases properties
      parameter Units.Area A_fg = 2800 "m2";
      parameter Units.HeatExchangeCoefficient hc_fg = 82.06 "W/m2.K";

      // Wall properties
      parameter Real M_wall = 17800 "kg";
      parameter Real Cp_wall = 420 "J/kg.K";

      // Initialization
      parameter Units.Temperature T_wall_0 = 450;
      parameter Units.Pressure P_water_0 = 70e5;
      parameter Units.Pressure P_fg_0 = 1e5;
      parameter Units.PositiveMassFlowRate Q_water_0 = 85;
      parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
      parameter Units.Temperature T_water_out_0 = 500;
      parameter Units.Temperature T_fg_out_0 = 560;
      parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
      parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;

      // Discretization
      parameter Real dM_wall = M_wall/N;
      parameter Real dA_water = A_water/N;
      parameter Real dA_fg = A_fg/N;

      // Enthalpies
      Units.SpecificEnthalpy h_water[N+1] "Water specific enthalpy";
      Units.SpecificEnthalpy h_fg[N+1] "Flue gas specific enthalpy";
      // Mass flow rate
      Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
      Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
      // Pressures
      Units.Pressure P_water(start=P_water_0) "Water Pressure";
      Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
      // Mass fractions
      Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
      Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";

      // Temperatures
      Units.Temperature T_water[N+1] "Node boundary water temperature";
      Units.Temperature T_water_avg[N] "Node average water temperature";
      Units.Temperature T_fg[N+1] "Node boundary flue gas temperature";
      Units.Temperature T_fg_avg[N] "Node average flue gas temperature";
      Units.Temperature T_wall[N](each start=T_wall_0, fixed=true) "Node wall temperature"; // add an init equation

      // ------ States ------
      WaterSteamMedium.ThermodynamicState state_water[N+1];
      FlueGasesMedium.ThermodynamicState state_fg[N+1];

      // ------ Conservation variables ------
      Units.Power dW_water[N] "Node water heat exchange";
      Units.Power dW_fg[N] "Node flue gas heat exchange";

      WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{40,20},{60,40}}), iconTransformation(extent={{40,20},{60,40}})));
      WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-60,20},{-40,40}}), iconTransformation(extent={{-60,20},{-40,40}})));
      FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{40,-40},{60,-20}}), iconTransformation(extent={{40,-40},{60,-20}})));
      FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-60,-40},{-40,-20}}), iconTransformation(extent={{-60,-40},{-40,-20}})));
      WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,20},{-10,40}})));
      FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
    equation

      // Boundaries
      // Enthalpies
      h_water[1] = water_side.h_in;
      h_water[N+1] = water_side.h_out;
      h_fg[1] = fg_side.h_out;
      h_fg[N+1] = fg_side.h_in;

      // Pressures
      P_water = water_side.P_in;
      P_fg = fg_side.P_in;

      // Mass flow rate
      Q_water = water_side.Q;
      Q_fg = fg_side.Q;

      // Mass Fractions
      Xi_water = water_side.Xi;
      Xi_fg = fg_side.Xi;

      // First node states
      state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
      state_fg[1] = FlueGasesMedium.setState_phX(P_fg, h_fg[1], Xi_fg);
      T_water[1] = WaterSteamMedium.temperature(state_water[1]);
      T_fg[1] = FlueGasesMedium.temperature(state_fg[1]);

      for i in 1:N loop
        // ------ States ------
        state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
        state_fg[i+1] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1], Xi_fg);

        // ------ Computed Quantities ------
        // Temperatures
        T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
        T_fg[i+1] = FlueGasesMedium.temperature(state_fg[i+1]);

        // ------ Conservation equations ------
        dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
        dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);

        // ------ Heat transfer equations ------
        dW_water[i] + dW_fg[i] = -dM_wall*Cp_wall*der(T_wall[i]); // equal zero
        dW_water[i] = -hc_water*dA_water*(T_water_avg[i] - T_wall[i]); // naming of convection HTC
        dW_fg[i] = -hc_fg*dA_fg*(T_fg_avg[i] - T_wall[i]);

        // ------- Average temperatures ------
        T_water_avg[i] = 0.5*(T_water[i] + T_water[i+1]);
        T_fg_avg[i] = 0.5*(T_fg[i] + T_fg[i+1]);
        end for;

      connect(water_side.C_in, water_inlet) annotation (Line(points={{10,30},{50,30}}, color={28,108,200}));
      connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,30},{-50,30}}, color={28,108,200}));
      connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,-30},{50,-30}}, color={95,95,95}));
      connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,-30},{-50,-30}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-50,50},{50,10}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-30,50},{-30,10}}, color={0,0,0}),
            Line(points={{-20,50},{-20,10}}, color={0,0,0}),
            Line(points={{-10,50},{-10,10}}, color={0,0,0}),
            Line(points={{0,50},{0,10}}, color={0,0,0}),
            Line(points={{10,50},{10,10}}, color={0,0,0}),
            Line(points={{20,50},{20,10}}, color={0,0,0}),
            Line(points={{30,50},{30,10}}, color={0,0,0}),
            Rectangle(
              extent={{-50,-10},{50,-50}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Line(points={{-30,-10},{-30,-50}}, color={0,0,0}),
            Line(points={{-20,-10},{-20,-50}}, color={0,0,0}),
            Line(points={{-10,-10},{-10,-50}}, color={0,0,0}),
            Line(points={{0,-10},{0,-50}}, color={0,0,0}),
            Line(points={{10,-10},{10,-50}}, color={0,0,0}),
            Line(points={{20,-10},{20,-50}}, color={0,0,0}),
            Line(points={{30,-10},{30,-50}}, color={0,0,0}),
            Line(points={{40,50},{40,10}}, color={0,0,0}),
            Line(points={{-40,50},{-40,10}}, color={0,0,0}),
            Line(points={{-40,-10},{-40,-50}}, color={0,0,0}),
            Line(points={{40,-10},{40,-50}}, color={0,0,0}),
            Rectangle(
              extent={{-50,10},{50,-10}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-50,10},{50,-10}},
              textColor={0,0,0},
              fontSize=20,
              textString="Tube Wall"),
            Line(
              points={{30,30},{-30,30}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Line(
              points={{-30,-30},{30,-30}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5)}),                                      Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MonoPhasicHX_nodes_Geometry;
  end HeatExchangers;

  package Tests
    model MonoPhasicHX_test

        // Boundary conditions
      input Real P_hot_source(start = 1.1, min = 0, nominal = 1) "barA";
      input Utilities.Units.MassFlowRate Q_hot_source(start = 640) "kg/s";
      input Utilities.Units.Temperature T_hot_source(start = 600) "degC";

      input Real P_cold_source(start = 130, min = 1.5, nominal = 100) "barA";
      input Utilities.Units.MassFlowRate Q_cold_source(start = 85) "kg/s";
      input Real T_cold_source(start = 410, min = 130, nominal = 150) "degC";

      HeatExchangers.MonoPhasicHX monoPhasicHX(Cp_wall=600, T_wall_avg_0=746.43146) annotation (Placement(transformation(extent={{-10,-10},{10,12}})));
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
    end MonoPhasicHX_test;

    model MonoPhasicHX_TSP_test

        // Boundary conditions
      input Real P_hot_source(start = 1.1, min = 0, nominal = 1) "barA";
      input Utilities.Units.MassFlowRate Q_hot_source(start = 610) "kg/s";
      input Utilities.Units.Temperature T_hot_source(start = 476.85) "degC";

      input Real P_cold_source(start = 130.4, min = 1.5, nominal = 100) "barA";
      input Utilities.Units.MassFlowRate Q_cold_source(start = 150) "kg/s";
      input Real T_cold_source(start = 320.34, min = 130, nominal = 150) "degC";

      HeatExchangers.MonoPhasicHX monoPhasicHX(
        A_water=3124.5,
        h_water=9265,
        A_fg=43586.8,
        h_fg=100,
        M_wall=68513,
        Cp_wall=1000,
        T_wall_avg_0=613.15) annotation (Placement(transformation(extent={{-10,-10},{10,12}})));
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
    end MonoPhasicHX_TSP_test;

    model MonoPhasicHX_node_test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_nodes monoPhasicHX_nodes(N=10, T_wall_0=745.15) annotation (Placement(transformation(extent={{-10,-10},{10,12}})));

        // Boundary conditions
      input Real P_hot_source(start = 1.1, min = 0, nominal = 1) "barA";
      input Real Q_hot_source(start = 640) "kg/s";
      input Utilities.Units.Temperature T_hot_source(start = 600) "degC";

      input Real P_cold_source(start = 130, min = 1.5, nominal = 100) "barA";
      input Utilities.Units.MassFlowRate Q_cold_source(start = 85) "kg/s";
      input Real T_cold_source(start = 410, min = 130, nominal = 150) "degC";

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

      connect(monoPhasicHX_nodes.water_inlet, cold_source.C_out) annotation (Line(points={{5,4.3},{5,2},{54,2},{54,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_nodes.water_outlet, cold_sink.C_in) annotation (Line(points={{-5,4.3},{-32,4.3},{-32,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_nodes.fg_inlet, hot_source.C_out) annotation (Line(points={{-5,-2.3},{-5,-2},{-50,-2},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_nodes.fg_outlet, hot_sink.C_in) annotation (Line(points={{5,-2.3},{5,-2},{50,-2},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_node_test;

    model MonoPhasicHX_node_Geometry_test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_nodes_Geometry
                                        monoPhasicHX_nodes_Geometry(
                                                           N=10,
        D_out=0.0381,
        e=0.003048,
        L=18.29,
        N_tubes=2*184,
        A_water=676.73035,
        T_wall_0=745.15)                                                          annotation (Placement(transformation(extent={{-10,-10},{10,12}})));

        // Boundary conditions
      input Real P_hot_source(start = 1.1, min = 0, nominal = 1) "barA";
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

      connect(monoPhasicHX_nodes_Geometry.water_inlet, cold_source.C_out) annotation (Line(points={{5,4.3},{5,2},{54,2},{54,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_nodes_Geometry.water_outlet, cold_sink.C_in) annotation (Line(points={{-5,4.3},{-32,4.3},{-32,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_nodes_Geometry.fg_inlet, hot_source.C_out) annotation (Line(points={{-5,-2.3},{-5,-2},{-50,-2},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_nodes_Geometry.fg_outlet, hot_sink.C_in) annotation (Line(points={{5,-2.3},{5,-2},{50,-2},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_node_Geometry_test;
  end Tests;
  annotation (Icon(graphics={Line(points={{-56,72}}, color={28,108,200}), Line(
          points={{-100,0},{-50,100},{50,-100},{100,0}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier)}));
end DynamicComponents;
