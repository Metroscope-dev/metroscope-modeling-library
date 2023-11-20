within MetroscopeModelingLibrary;
package DynamicComponents
  package Correlations

    function Zukauskas "External flow heat transfer coefficient - Bare tubes"

      import MetroscopeModelingLibrary.Utilities.Units;

      // Input arguments
        input Real Re_fg_max;
        input Real Pr_fg_avg;
        input Real Pr_fg_avg_s;
        input Integer Tubes_Config;
        input Integer Rows;
        input Units.Length S_T;
        input Units.Length S_L;

      // Function output
        // Nusselt number
        output Real Nu_fg_avg;

    protected
      Real Constant_C "Constant used in the Zukauskas empirical relation";
      Real Constant_m "Constant used in the Zukauskas empirical relation";
      parameter Real C2[2,19] = {{0.7,0.8,0.86,0.9,0.92,0.935,0.95,0.956666667,0.963333333,0.97,0.973333333,0.976666667,0.98,0.983333333,0.986666667,0.99,0.99,0.99,0.99},
                                 {0.64,0.76,0.84,0.89,0.92,0.935,0.95,0.956666667,0.963333333,0.97,0.973333333,0.976666667,0.98,0.983333333,0.986666667,0.99,0.99,0.99,0.99}};

    algorithm

      if (Tubes_Config == 1) then
        if (Re_fg_max > 10 and Re_fg_max < 10^2) then
          Constant_C := 0.8;
          Constant_m := 0.4;
        elseif (Re_fg_max > 10^3 and Re_fg_max < 2*10^5) then
          Constant_C := 0.27;
          Constant_m := 0.63;
        elseif (Re_fg_max > 2*10^5 and Re_fg_max < 2*10^6) then
          Constant_C := 0.021;
          Constant_m := 0.84;
        end if;

      elseif  (Tubes_Config == 2) then
        if (Re_fg_max > 10 and Re_fg_max < 10^2) then
          Constant_C := 0.9;
          Constant_m := 0.4;
        elseif (Re_fg_max > 10^3 and Re_fg_max < 2*10^5) then
          if (S_T/S_L < 2) then
            Constant_C := 0.35*(S_T/S_L)^(1/5);
            Constant_m := 0.6;
          else
            Constant_C := 0.4;
            Constant_m := 0.6;
          end if;
        elseif (Re_fg_max > 2*10^5 and Re_fg_max < 2*10^6) then
          Constant_C := 0.022;
          Constant_m := 0.84;
        end if;
      end if;

      if (Rows < 20) then
        Nu_fg_avg :=Constant_C*Re_fg_max^Constant_m*Pr_fg_avg^0.36*(Pr_fg_avg/Pr_fg_avg_s)^0.25*C2[Tubes_Config, Rows];
      else
        Nu_fg_avg :=Constant_C*Re_fg_max^Constant_m*Pr_fg_avg^0.36*(Pr_fg_avg/Pr_fg_avg_s)^0.25;
      end if;

    end Zukauskas;

    function ESCOA "External flow heat transfer coefficient - Finned tubes"

      /* This function is valid for segmented finned-tubes at staggered arrangement with:
    2000 < Re_fg < 500000
    9.5 mm < H_fin < 38.1 mm
    0.9 mm < e_fin < 4.2 mm
    39.37 fin/m < S_fin < 275 fin/m
  */

      import MetroscopeModelingLibrary.Utilities.Units;

      // Input arguments
        input Real Re_fg "Flue gas Reynold's number";
        input Real Pr_fg "Flue gas Prandtl number";
        input Integer Rows "Number of tubes in flow direction";
        input Units.Temperature T_fg "Flue gas temperature";
        input Units.Temperature T_fin "Fin temperature";
        input Units.Length D_out "Tube outer diameter";
        input Units.Length H_fin "Fin height";
        input Units.Length e_fin "Fin thickness";
        input Units.Length S_fin "Fin pitch";
        input Units.Length S_T "Tube bundle transverse pitch";
        input Units.Length S_L "Tube bundle longitudanal pitch";

      // Function output
        // Nusselt number
        output Real Nu_fg;

    protected
      Real Constant_C1 "Constant used in ESCOA empirical relation";
      Real Constant_C3 "Constant used in ESCOA empirical relation";
      Real Constant_C5 "Constant used in ESCOA empirical relation";
      parameter Units.Length D_fin = D_out + 2*H_fin "Fin outer diameter";

    algorithm

       // Traditional ESCOA Correlation
      Constant_C1 := 0.25*Re_fg^(-0.35);
      Constant_C3 := 0.55 + 0.45*exp(-0.35*H_fin/(S_fin - e_fin));
      Constant_C5 := 0.7 + (0.7 - 0.8*exp(-0.15*Rows^2))*exp(-S_L/S_T);

      // Revised ESCOA Correlation

    //   Constant_C1 := 0.091*Re_fg^(-0.25);
    //   Constant_C3 := 0.35 + 0.65*exp(-0.17*H_fin/(S_fin - e_fin));
    //   Constant_C5 := 0.7 + (0.7 - 0.8*exp(-0.15*Rows^2))*exp(-S_L/S_T);


      Nu_fg := Constant_C1*Constant_C3*Constant_C5*Re_fg*Pr_fg^(1/3)*((T_fg + 273.15)/(T_fin + 273.15))^0.25*(D_fin/D_out)^0.5;


    end ESCOA;
  end Correlations;

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

    model MonoPhasicHX_nodes_InitEquations
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      import MetroscopeModelingLibrary.Utilities.Constants;

      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      // Geometry
      // Pipes
      parameter Units.Length D_out = 0.03 "Pipe outer diameter";
      parameter Units.Length e = 0.003 "Pipe wall thickness";
      parameter Units.Length D_in = D_out - 2*e "Pipe inner diameter";
      parameter Units.Length L = 22 "Tube's length";
      parameter Integer N_tubes = 180 "Number of tubes";

      // Water side properties
      parameter Units.Area A_water = N_tubes*L*Constants.pi*D_in "Water side heat exchange surface";
      parameter Units.HeatExchangeCoefficient K_conv_water = 2430 "Water side convection heat transfer coefficient";

      // Flue gases properties
      parameter Units.Area A_fg = 2800 "Flue gase side heat exchange surface";
      parameter Units.HeatExchangeCoefficient K_conv_fg = 82.06 "Flue gase convection heat transfer coefficient";

      // Wall properties
      parameter Units.Mass M_wall = 17800 "Tubes total mass";
      parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";

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
      parameter Integer N = 10;
      parameter Units.Mass dM_wall = M_wall/N "Tube mass of a single node";
      parameter Units.Area dA_water = A_water/N "Water side heat exchange surface of a single node";
      parameter Units.Area dA_fg = A_fg/N "Flue gas side heat exchange surface of a single node";

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
      Units.Temperature T_water_node[N] "Node average water temperature";
      Units.Temperature T_fg[N+1] "Node boundary flue gas temperature";
      Units.Temperature T_fg_node[N] "Node average flue gas temperature";
      Units.Temperature T_wall[N] "Node wall temperature"; // add an init equation

      // States
      WaterSteamMedium.ThermodynamicState state_water[N+1] "Water side node boundary state";
      FlueGasesMedium.ThermodynamicState state_fg[N+1] "Flue gas side node boundary states";

      // Conservation variables
      Units.Power dW_water[N] "Node water heat exchange";
      Units.Power dW_fg[N] "Node flue gas heat exchange";

      // Observables
      Units.Temperature T_water_in "Water inlet temperature";
      Units.Temperature T_water_out "Water outlet temperature";
      Units.Temperature T_fg_in "Flue gas inlet temperature";
      Units.Temperature T_fg_out "Flue gas outlet temperature";
      Units.Temperature T_water_avg "Water average temperature";
      Units.Temperature T_fg_avg "Flue gas average temperature";
      Units.Temperature T_wall_avg "Wall average temperature";

      WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{90,30},{110,50}}),iconTransformation(extent={{90,30},{110,50}})));
      WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-110,32},{-90,52}}),iconTransformation(extent={{-110,32},{-90,52}})));
      FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{88,-50},{108,-30}}),iconTransformation(extent={{88,-50},{108,-30}})));
      FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-50},{-90,-30}}),iconTransformation(extent={{-110,-50},{-90,-30}})));
      WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,10},{-10,30}})));
      FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
    equation

      // ------ Boundaries ------
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

      // ------ Observables ------
        // Temperatures
        T_water_in = water_side.T_in;
        T_water_out = water_side.T_out;
        T_fg_in = fg_side.T_in;
        T_fg_out = fg_side.T_out;
        T_water_avg = sum(T_water_node)/N;
        T_fg_avg = sum(T_fg_node)/N;
        T_wall_avg =  sum(T_wall)/N;

      // ------ Discretization computation loop ------

        for i in 1:N loop

          // States
          state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
          state_fg[i+1] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1], Xi_fg);

          // Computed Quantities
          // Temperatures
          T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
          T_fg[i+1] = FlueGasesMedium.temperature(state_fg[i+1]);

          // Conservation equations
          dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
          dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);

          // Heat transfer equations
          dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;
          dW_water[i] = -K_conv_water*dA_water*(T_water_node[i] - T_wall[i]);
          dW_fg[i] = -K_conv_fg*dA_fg*(T_fg_node[i] - T_wall[i]);

          // Average temperatures
          T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
          T_fg_node[i] = 0.5*(T_fg[i] + T_fg[i+1]);

        end for;

    initial equation

      for i in 1:N loop
        der(T_wall[i]) = 0;
      end for;

    equation
      connect(water_side.C_in, water_inlet) annotation (Line(points={{10,20},{100,20},{100,40}},
                                                                                       color={28,108,200}));
      connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,20},{-100,20},{-100,42}},
                                                                                           color={28,108,200}));
      connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,-20},{100,-20},{100,-40},{98,-40}},
                                                                                     color={95,95,95}));
      connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,-20},{-100,-20},{-100,-40}},
                                                                                     color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,-10},{100,-60}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,10},{100,-10}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,60},{100,10}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-60,60},{-60,-60}},color={0,0,0}),
            Line(points={{-40,60},{-40,-60}},color={0,0,0}),
            Line(points={{40,60},{40,-60}},color={0,0,0}),
            Line(points={{60,60},{60,-60}},color={0,0,0}),
            Line(points={{80,60},{80,-60}},color={0,0,0}),
            Line(points={{-80,60},{-80,-60}},color={0,0,0}),
            Text(
              extent={{-20,4},{16,-4}},
              textColor={0,0,0},
              fontSize=20,
              textString="Tube Wall"),
            Line(
              points={{30,40},{-30,40}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Line(
              points={{-26,-38},{34,-38}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Text(
              extent={{-22,32},{14,24}},
              textColor={0,0,0},
              fontSize=20,
              textString="Water side"),
            Text(
              extent={{-30,-24},{26,-32}},
              textColor={0,0,0},
              fontSize=20,
              textString="Flue gas side")}),                         Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MonoPhasicHX_nodes_InitEquations;

    model MonoPhasicHX_ConvHTC_WaterCalc
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      import MetroscopeModelingLibrary.Utilities.Constants;

      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      // ------ Geometry ------
        // Pipes
        parameter Units.Length D_out = 0.03 "Pipe outer diameter";
        parameter Units.Length e = 0.003 "Pipe wall thickness";
        parameter Units.Length D_in = D_out - 2*e "Pipe inner diameter";
        parameter Units.Length L = 22 "Tube's length";
        parameter Integer N_tubes_row = 184 "Number of tubes per row";
        parameter Integer Rows = 2 "Number of tubes rows";
        parameter Integer N_tubes = N_tubes_row*Rows "Total number of tubes";
        // Water side
        parameter Units.Area A_water = N_tubes*L*Constants.pi*D_in "Water side heat exchange surface";
        parameter Units.Area Ac_water = 0.25*Constants.pi*D_in^2*N_tubes_row "Water side cross sectional area";
        // Flue gases
        parameter Units.Area A_fg = 2800 "Flue gase side heat exchange surface";
        parameter Units.HeatExchangeCoefficient K_conv_fg = 82.06 "Flue gase convection heat transfer coefficient";
        // Wall
        parameter Units.Mass M_wall = 17800 "Tubes total mass";
        parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";

      // ------ Initialization ------
        parameter Units.Temperature T_wall_0 = 450;
        parameter Units.Pressure P_water_0 = 70e5;
        parameter Units.Pressure P_fg_0 = 1e5;
        parameter Units.PositiveMassFlowRate Q_water_0 = 85;
        parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
        parameter Units.Temperature T_water_out_0 = 500;
        parameter Units.Temperature T_fg_out_0 = 560;
        parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
        parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;
        parameter Units.HeatExchangeCoefficient K_conv_water_0 = 2400;

      // ------ Discretization ------
        parameter Integer N = 10;
        parameter Units.Mass dM_wall = M_wall/N "Tube mass of a single node";
        parameter Units.Area dA_water = A_water/N "Water side heat exchange surface of a single node";
        parameter Units.Area dA_fg = A_fg/N "Flue gas side heat exchange surface of a single node";

      // ------ Fluids properties ------
        // State
        WaterSteamMedium.ThermodynamicState state_water[N+1] "Water side node boundary state";
        FlueGasesMedium.ThermodynamicState state_fg[N+1] "Flue gas side node boundary states";
        // Enthalpy
        Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
        Units.SpecificEnthalpy h_fg[N+1] "Flue gas specific enthalpy";
        // Mass flow rate
        Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
        Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
        // Pressure
        Units.Pressure P_water(start=P_water_0) "Water Pressure";
        Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
        // Density
        Units.Density rho_water[N+1] "Node boundary water density";
        Units.Density rho_water_node[N] "Node average water density";
        Units.Density rho_fg[N+1] "Node boundary flue gas density";
        Units.Density rho_fg_node[N] "Node average flue gas density";
        // Mass fraction
        Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
        Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
        // Temperature
        Units.Temperature T_water[N+1] "Node boundary water temperature";
        Units.Temperature T_water_node[N] "Node average water temperature";
        Units.Temperature T_fg[N+1] "Node boundary flue gas temperature";
        Units.Temperature T_fg_node[N] "Node average flue gas temperature";
        Units.Temperature T_wall[N] "Node wall temperature"; // add an init equation
        // Dynamic viscosities
        Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
        // Heat capacities Cp
        Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
        Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
        // Thermal conductivity
        Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";

      // Heat transfer parameters
        // Average velocities
        Units.Velocity U_water_node[N] "Node average water velocity";
        // Prandtl Number
        Real Pr_water[N] "Node water Prandtl's number";
        // Reynold's Number
        Real Re_water[N] "Node water Reynold's number";
        // Nusselt Nymber
        Real Nu_water[N] "Node water Nusselt number";
        // Convection heat transfer coefficient
        Units.HeatExchangeCoefficient K_conv_water[N](each start=K_conv_water_0) "Water side convection heat transfer coefficient";
        // Discretized heat transfer power
        Units.Power dW_water[N] "Node water heat exchange";
        Units.Power dW_fg[N] "Node flue gas heat exchange";

      // Parameters of interest
        Units.Temperature T_water_in "Water inlet temperature";
        Units.Temperature T_water_out "Water outlet temperature";
        Units.Temperature T_fg_in "Flue gas inlet temperature";
        Units.Temperature T_fg_out "Flue gas outlet temperature";
        Units.Temperature T_water_avg "Water overall average temperature";
        Units.Temperature T_fg_avg "Flue gas overall average temperature";
        Units.Temperature T_wall_avg "Wall overall average temperature";
        Units.Velocity U_water_avg "Water overall average velocity";
        Real Pr_water_avg "Water overall average Prandtl number";
        Real Re_water_avg "Water overall average Reynold's number";
        Real Nu_water_avg "Water overall average Reynold's number";
        Real K_conv_water_avg "Water side overall average convection heat transfer coefficient";

      WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{90,30},{110,50}}),iconTransformation(extent={{90,30},{110,50}})));
      WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-110,32},{-90,52}}),iconTransformation(extent={{-110,32},{-90,52}})));
      FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{88,-50},{108,-30}}),iconTransformation(extent={{88,-50},{108,-30}})));
      FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-50},{-90,-30}}),iconTransformation(extent={{-110,-50},{-90,-30}})));
      WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,10},{-10,30}})));
      FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
    equation

      // ------ Boundaries ------
        // Enthalpy
        h_water[1] = water_side.h_in;
        h_water[N+1] = water_side.h_out;
        h_fg[1] = fg_side.h_out;
        h_fg[N+1] = fg_side.h_in;
        // Pressure
        P_water = water_side.P_in;
        P_fg = fg_side.P_in;
        // Mass flow rate
        Q_water = water_side.Q;
        Q_fg = fg_side.Q;
        // Mass Fraction
        Xi_water = water_side.Xi;
        Xi_fg = fg_side.Xi;

      // ------ First node properties ------
        state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
        state_fg[1] = FlueGasesMedium.setState_phX(P_fg, h_fg[1], Xi_fg);
        T_water[1] = WaterSteamMedium.temperature(state_water[1]);
        T_fg[1] = FlueGasesMedium.temperature(state_fg[1]);
        rho_water[1] = WaterSteamMedium.density(state_water[1]);
        rho_fg[1] = FlueGasesMedium.density(state_fg[1]);
        Mu_water[1] = WaterSteamMedium.dynamicViscosity(state_water[1]);
        Cp_water[1] = WaterSteamMedium.specificHeatCapacityCp(state_water[1]);
        k_water[1] = WaterSteamMedium.thermalConductivity(state_water[1]);

      // ------ Parameters of interest ------
        // Temperatures
        T_water_in = water_side.T_in;
        T_water_out = water_side.T_out;
        T_fg_in = fg_side.T_in;
        T_fg_out = fg_side.T_out;
        T_water_avg = sum(T_water_node)/N;
        T_fg_avg = sum(T_fg_node)/N;
        T_wall_avg =  sum(T_wall)/N;
        U_water_avg = sum(U_water_node)/N;
        Pr_water_avg = sum(Pr_water)/N;
        Re_water_avg = sum(Re_water)/N;
        Nu_water_avg = sum(Nu_water)/N;
        K_conv_water_avg = sum(K_conv_water)/N;

      // ------ Discretization computation loop ------
        for i in 1:N loop

          // Fluids Properties
            // State
            state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
            state_fg[i+1] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1], Xi_fg);
            // Temperature
            T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
            T_fg[i+1] = FlueGasesMedium.temperature(state_fg[i+1]);
            T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
            T_fg_node[i] = 0.5*(T_fg[i] + T_fg[i+1]);
            // Density
            rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
            rho_fg[i+1] = FlueGasesMedium.density(state_fg[i+1]);
            rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
            rho_fg_node[i] = 0.5*(rho_fg[i] + rho_fg[i+1]);
            // Dynamic viscosity
            Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
            Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
            // Specific heat capacities Cp
            Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
            Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
            // Thermal conductivity
            k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
            k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);

          // Node energy balance
            // Water side
            dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
            // Flue gas side
            dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);
            // Global with wall storage
            dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

          // Convection heat transfer coefficient calculation
            // Average velocities
            U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
            // Prandtl number
            Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
            // Reynold's number
            Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
            // Nusselt number
            Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
            // Convection correlation: Dittus-Boelter equation
            Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

          // Convection heat transfer equations
            // Water side
            dW_water[i] = -K_conv_water[i]*dA_water*(T_water_node[i] - T_wall[i]);
            // Flue gas side
            dW_fg[i] = -K_conv_fg*dA_fg*(T_fg_node[i] - T_wall[i]);

        end for;

    initial equation

      for i in 1:N loop
        der(T_wall[i]) = 0;
      end for;

    equation
      connect(water_side.C_in, water_inlet) annotation (Line(points={{10,20},{100,20},{100,40}},
                                                                                       color={28,108,200}));
      connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,20},{-100,20},{-100,42}},
                                                                                           color={28,108,200}));
      connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,-20},{100,-20},{100,-40},{98,-40}},
                                                                                     color={95,95,95}));
      connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,-20},{-100,-20},{-100,-40}},
                                                                                     color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,-10},{100,-60}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,10},{100,-10}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,60},{100,10}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-60,60},{-60,-60}},color={0,0,0}),
            Line(points={{-40,60},{-40,-60}},color={0,0,0}),
            Line(points={{40,60},{40,-60}},color={0,0,0}),
            Line(points={{60,60},{60,-60}},color={0,0,0}),
            Line(points={{80,60},{80,-60}},color={0,0,0}),
            Line(points={{-80,60},{-80,-60}},color={0,0,0}),
            Text(
              extent={{-20,4},{16,-4}},
              textColor={0,0,0},
              fontSize=20,
              textString="Tube Wall"),
            Line(
              points={{30,40},{-30,40}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Line(
              points={{-26,-38},{34,-38}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Text(
              extent={{-22,32},{14,24}},
              textColor={0,0,0},
              fontSize=20,
              textString="Water side"),
            Text(
              extent={{-30,-24},{26,-32}},
              textColor={0,0,0},
              fontSize=20,
              textString="Flue gas side")}),                         Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MonoPhasicHX_ConvHTC_WaterCalc;

    model MonoPhasicHX_ConvHTC_FGCalc

      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      import MetroscopeModelingLibrary.Utilities.Constants;
      import CorrelationConstants =
             MetroscopeModelingLibrary.DynamicComponents.Correlations;

      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      // ------ Geometry ------
        // Pipes
        parameter Units.Length D_out = 0.03 "Pipe outer diameter";
        parameter Units.Length e = 0.003 "Pipe wall thickness";
        parameter Units.Length D_in = D_out - 2*e "Pipe inner diameter";
        parameter Units.Length L = 22 "Tube's length";
        parameter Integer N_tubes_row = 184 "Number of tubes per row";
        parameter Integer Rows = 2 "Number of tubes rows";
        parameter Integer N_tubes = N_tubes_row*Rows "Total number of tubes";
        parameter Integer Tubes_Config = 2 "1: aligned, 2: staggered";
        parameter Units.Length fg_path_width = 14.07 "Flue gas path transverse width";
        parameter Units.Length S_T = 76.25e-3 "Transverse pitch";
        parameter Units.Length S_L = 95.25e-3 "Longitudinal pitch";
        parameter Units.Length S_D = (S_L^2 + (S_T/2)^2)^0.5 "Diagonal pitch";
        // Water side
        parameter Units.Area A_water = N_tubes*L*Constants.pi*D_in "Water side heat exchange surface";
        parameter Units.Area Ac_water = 0.25*Constants.pi*D_in^2*N_tubes_row "Water side cross sectional area";
        // Flue gases
        parameter Units.Area A_fg = 2800 "Flue gase side heat exchange surface";
        parameter Units.Area Af_fg = L*fg_path_width "Flue gas frontal area";
        Units.HeatExchangeCoefficient K_conv_fg "Flue gase convection heat transfer coefficient"; //  = 82.06
        // Wall
        parameter Units.Mass M_wall = 17800 "Tubes total mass";
        parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";

      // ------ Initialization ------
        parameter Units.Temperature T_wall_0 = 450;
        parameter Units.Pressure P_water_0 = 70e5;
        parameter Units.Pressure P_fg_0 = 1e5;
        parameter Units.PositiveMassFlowRate Q_water_0 = 85;
        parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
        parameter Units.Temperature T_water_out_0 = 500;
        parameter Units.Temperature T_fg_out_0 = 560;
        parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
        parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;
        parameter Units.HeatExchangeCoefficient K_conv_water_0 = 2400;

      // ------ Discretization ------
        parameter Integer N = 10;
        parameter Units.Mass dM_wall = M_wall/N "Tube mass of a single node";
        parameter Units.Area dA_water = A_water/N "Water side heat exchange surface of a single node";
        parameter Units.Area dA_fg = A_fg/N "Flue gas side heat exchange surface of a single node";

      // ------ Fluids properties ------
        // State
        WaterSteamMedium.ThermodynamicState state_water[N+1] "Water side node boundary state";
        FlueGasesMedium.ThermodynamicState state_fg[N+1] "Flue gas side node boundary states";
        // Enthalpy
        Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
        Units.SpecificEnthalpy h_fg[N+1] "Flue gas specific enthalpy";
        // Mass flow rate
        Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
        Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
        // Pressure
        Units.Pressure P_water(start=P_water_0) "Water Pressure";
        Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
        // Density
        Units.Density rho_water[N+1] "Node boundary water density";
        Units.Density rho_water_node[N] "Node average water density";
        Units.Density rho_fg[N+1] "Node boundary flue gas density";
        Units.Density rho_fg_node[N] "Node average flue gas density";
        // Mass fraction
        Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
        Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
        // Temperature
        Units.Temperature T_water[N+1] "Node boundary water temperature";
        Units.Temperature T_water_node[N] "Node average water temperature";
        Units.Temperature T_fg[N+1] "Node boundary flue gas temperature";
        Units.Temperature T_fg_node[N] "Node average flue gas temperature";
        Units.Temperature T_wall[N] "Node wall temperature"; // add an init equation
        // Dynamic viscosities
        Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_fg[N+1] "Node boundary flue gas dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_fg_node[N] "Node average flue gas dynamic viscosity";
        // Heat capacities Cp
        Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
        Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
        Units.HeatCapacity Cp_fg[N+1] "Node boundary flue gas Cp";
        Units.HeatCapacity Cp_fg_node[N] "Node average flue gas Cp";
        // Thermal conductivity
        Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_fg[N+1] "Node boundary flue gas thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_fg_node[N] "Node average flue gas thermal conductivity";

      // ------ Tubes configuration parameters ------
        Units.Velocity U_fg_face "Flue gas face velocity";
        Units.Velocity U_fg_max "Flue gas maximum velocity";
        Real Re_fg_max "Flue gas maximum Reynold's number";
        Real Pr_fg "Flue gas overall average Prandtl number";
        FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
        Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
        Real Nu_fg_avg "Flue gass overall average Nusselt number";

      // ------ Heat transfer parameters ------
        // Average velocities
        Units.Velocity U_water_node[N] "Node average water velocity";
        // Prandtl Number
        Real Pr_water[N] "Node water Prandtl's number";
        // Reynold's Number
        Real Re_water[N] "Node water Reynold's number";
        // Nusselt Nymber
        Real Nu_water[N] "Node water Nusselt number";
        // Convection heat transfer coefficient
        Units.HeatExchangeCoefficient K_conv_water[N](each start=K_conv_water_0) "Water side convection heat transfer coefficient";
        // Discretized heat transfer power
        Units.Power dW_water[N] "Node water heat exchange";
        Units.Power dW_fg[N] "Node flue gas heat exchange";

      // Parameters of interest
        Units.Temperature T_water_in "Water inlet temperature";
        Units.Temperature T_water_out "Water outlet temperature";
        Units.Temperature T_fg_in "Flue gas inlet temperature";
        Units.Temperature T_fg_out "Flue gas outlet temperature";
        Units.Temperature T_water_avg "Water overall average temperature";
        Units.Temperature T_fg_avg "Flue gas overall average temperature";
        Units.Temperature T_wall_avg "Wall overall average temperature";
        Units.Velocity U_water_avg "Water overall average velocity";
        Real Pr_water_avg "Water overall average Prandtl number";
        Real Re_water_avg "Water overall average Reynold's number";
        Real Nu_water_avg "Water overall average Nusselt number";
        Real K_conv_water_avg "Water side overall average convection heat transfer coefficient";

      WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{90,30},{110,50}}),iconTransformation(extent={{90,30},{110,50}})));
      WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-110,32},{-90,52}}),iconTransformation(extent={{-110,32},{-90,52}})));
      FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{88,-50},{108,-30}}),iconTransformation(extent={{88,-50},{108,-30}})));
      FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-50},{-90,-30}}),iconTransformation(extent={{-110,-50},{-90,-30}})));
      WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,10},{-10,30}})));
      FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
    equation

      // ------ Boundaries ------
        // Enthalpy
        h_water[1] = water_side.h_in;
        h_water[N+1] = water_side.h_out;
        h_fg[1] = fg_side.h_out;
        h_fg[N+1] = fg_side.h_in;
        // Pressure
        P_water = water_side.P_in;
        P_fg = fg_side.P_in;
        // Mass flow rate
        Q_water = water_side.Q;
        Q_fg = fg_side.Q;
        // Mass Fraction
        Xi_water = water_side.Xi;
        Xi_fg = fg_side.Xi;

      // ------ Tubes configuration parameters ------
        // Flue gas face velocity
        U_fg_face = Q_fg/(rho_fg[N+1]*Af_fg);
        // Flue gas maximum velocity
        if (Tubes_Config == 1) then
          U_fg_max = S_T*U_fg_face/(S_T - D_out);
        elseif (S_D < (S_T + D_out)/2) then
          U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
        else
          U_fg_max = S_T*U_fg_face/(S_T - D_out);
        end if;
        // Flue gas maximum Reynold's number
        Re_fg_max = rho_fg[N+1]*U_fg_max*D_out/Mu_fg[N+1];
        // Flue gas Prandtl number
        Pr_fg = Cp_fg[N+1]*Mu_fg[N+1]/k_fg[N+1];
        // Flue gas Prandtl number at surface temperature
        state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
        Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
        // Convection coefficient calculation
        Nu_fg_avg = K_conv_fg*D_out/k_fg[N+1];
        Nu_fg_avg =CorrelationConstants.Zukauskas(
        Re_fg_max,
        Pr_fg,
        Pr_fg_s,
        Tubes_Config,
        Rows,
        S_T,
        S_L);


      // ------ First node properties ------
        state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
        state_fg[1] = FlueGasesMedium.setState_phX(P_fg, h_fg[1], Xi_fg);
        T_water[1] = WaterSteamMedium.temperature(state_water[1]);
        T_fg[1] = FlueGasesMedium.temperature(state_fg[1]);
        rho_water[1] = WaterSteamMedium.density(state_water[1]);
        rho_fg[1] = FlueGasesMedium.density(state_fg[1]);
        Mu_water[1] = WaterSteamMedium.dynamicViscosity(state_water[1]);
        Mu_fg[1] = FlueGasesMedium.dynamicViscosity(state_fg[1]);
        Cp_water[1] = WaterSteamMedium.specificHeatCapacityCp(state_water[1]);
        Cp_fg[1] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1]);
        k_water[1] = WaterSteamMedium.thermalConductivity(state_water[1]);
        k_fg[1] = FlueGasesMedium.thermalConductivity(state_fg[1]);

      // ------ Parameters of interest ------
        // Temperatures
        T_water_in = water_side.T_in;
        T_water_out = water_side.T_out;
        T_fg_in = fg_side.T_in;
        T_fg_out = fg_side.T_out;
        T_water_avg = sum(T_water_node)/N;
        T_fg_avg = sum(T_fg_node)/N;
        T_wall_avg =  sum(T_wall)/N;
        U_water_avg = sum(U_water_node)/N;
        Pr_water_avg = sum(Pr_water)/N;
        Re_water_avg = sum(Re_water)/N;
        Nu_water_avg = sum(Nu_water)/N;
        K_conv_water_avg = sum(K_conv_water)/N;

      // ------ Discretization computation loop ------
        for i in 1:N loop

          // Fluids Properties
            // State
            state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
            state_fg[i+1] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1], Xi_fg);
            // Temperature
            T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
            T_fg[i+1] = FlueGasesMedium.temperature(state_fg[i+1]);
            T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
            T_fg_node[i] = 0.5*(T_fg[i] + T_fg[i+1]);
            // Density
            rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
            rho_fg[i+1] = FlueGasesMedium.density(state_fg[i+1]);
            rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
            rho_fg_node[i] = 0.5*(rho_fg[i] + rho_fg[i+1]);
            // Dynamic viscosity
            Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
            Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
            Mu_fg[i+1] = FlueGasesMedium.dynamicViscosity(state_fg[i+1]);
            Mu_fg_node[i] = 0.5*(Mu_fg[i] + Mu_fg[i+1]);
            // Specific heat capacities Cp
            Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
            Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
            Cp_fg[i+1] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1]);
            Cp_fg_node[i] = 0.5*(Cp_fg[i] + Cp_fg[i+1]);
            // Thermal conductivity
            k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
            k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);
            k_fg[i+1] = FlueGasesMedium.thermalConductivity(state_fg[i+1]);
            k_fg_node[i] = 0.5*(k_fg[i] + k_fg[i+1]);

          // Node energy balance
            // Water side
            dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
            // Flue gas side
            dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);
            // Global with wall storage
            dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

          // Convection heat transfer coefficient calculation
            // Average velocities
            U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
            // Prandtl number
            Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
            // Reynold's number
            Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
            // Nusselt number
            Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
            // Convection correlation: Dittus-Boelter equation
            Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

          // Convection heat transfer equations
            // Water side
            dW_water[i] = -K_conv_water[i]*dA_water*(T_water_node[i] - T_wall[i]);
            // Flue gas side
            dW_fg[i] = -K_conv_fg*dA_fg*(T_fg_node[i] - T_wall[i]);

        end for;

    initial equation

      for i in 1:N loop
        der(T_wall[i]) = 0;
      end for;

    equation
      connect(water_side.C_in, water_inlet) annotation (Line(points={{10,20},{100,20},{100,40}},
                                                                                       color={28,108,200}));
      connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,20},{-100,20},{-100,42}},
                                                                                           color={28,108,200}));
      connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,-20},{100,-20},{100,-40},{98,-40}},
                                                                                     color={95,95,95}));
      connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,-20},{-100,-20},{-100,-40}},
                                                                                     color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,-10},{100,-60}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,10},{100,-10}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,60},{100,10}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-60,60},{-60,-60}},color={0,0,0}),
            Line(points={{-40,60},{-40,-60}},color={0,0,0}),
            Line(points={{40,60},{40,-60}},color={0,0,0}),
            Line(points={{60,60},{60,-60}},color={0,0,0}),
            Line(points={{80,60},{80,-60}},color={0,0,0}),
            Line(points={{-80,60},{-80,-60}},color={0,0,0}),
            Text(
              extent={{-20,4},{16,-4}},
              textColor={0,0,0},
              fontSize=20,
              textString="Tube Wall"),
            Line(
              points={{30,40},{-30,40}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Line(
              points={{-26,-38},{34,-38}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Text(
              extent={{-22,32},{14,24}},
              textColor={0,0,0},
              fontSize=20,
              textString="Water side"),
            Text(
              extent={{-30,-24},{26,-32}},
              textColor={0,0,0},
              fontSize=20,
              textString="Flue gas side")}),                         Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MonoPhasicHX_ConvHTC_FGCalc;

    model MonoPhasicHX_WaterHTC_Bi
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      import MetroscopeModelingLibrary.Utilities.Constants;

      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      // ------ Geometry ------
        // Pipes
        parameter Units.Length D_out = 0.03 "Pipe outer diameter";
        parameter Units.Length e = 0.003 "Pipe wall thickness";
        parameter Units.Length D_in = D_out - 2*e "Pipe inner diameter";
        parameter Units.Length L = 22 "Tube's length";
        parameter Integer N_tubes_row = 184 "Number of tubes per row";
        parameter Integer Rows = 2 "Number of tubes rows";
        parameter Integer N_tubes = N_tubes_row*Rows "Total number of tubes";
        parameter Modelica.Units.SI.ThermalConductivity K_cond_wall = 27 "Wall thermal conductivity";
        // Water side
        parameter Units.Area A_water = N_tubes*L*Constants.pi*D_in "Water side heat exchange surface";
        parameter Units.Area Ac_water = 0.25*Constants.pi*D_in^2*N_tubes_row "Water side cross sectional area";
        // Flue gases
        parameter Units.Area A_fg = 2800 "Flue gase side heat exchange surface";
        parameter Units.HeatExchangeCoefficient K_conv_fg = 82.06 "Flue gase convection heat transfer coefficient";
        // Wall
        parameter Units.Mass M_wall = 17800 "Tubes total mass";
        parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";

      // ------ Initialization ------
        parameter Units.Temperature T_wall_0 = 450;
        parameter Units.Pressure P_water_0 = 70e5;
        parameter Units.Pressure P_fg_0 = 1e5;
        parameter Units.PositiveMassFlowRate Q_water_0 = 85;
        parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
        parameter Units.Temperature T_water_out_0 = 500;
        parameter Units.Temperature T_fg_out_0 = 560;
        parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
        parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;
        parameter Units.HeatExchangeCoefficient K_conv_water_0 = 2400;

      // ------ Discretization ------
        parameter Integer N = 10;
        parameter Units.Mass dM_wall = M_wall/N "Tube mass of a single node";
        parameter Units.Area dA_water = A_water/N "Water side heat exchange surface of a single node";
        parameter Units.Area dA_fg = A_fg/N "Flue gas side heat exchange surface of a single node";

      // ------ Fluids properties ------
        // State
        WaterSteamMedium.ThermodynamicState state_water[N+1] "Water side node boundary state";
        FlueGasesMedium.ThermodynamicState state_fg[N+1] "Flue gas side node boundary states";
        // Enthalpy
        Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
        Units.SpecificEnthalpy h_fg[N+1] "Flue gas specific enthalpy";
        // Mass flow rate
        Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
        Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
        // Pressure
        Units.Pressure P_water(start=P_water_0) "Water Pressure";
        Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
        // Density
        Units.Density rho_water[N+1] "Node boundary water density";
        Units.Density rho_water_node[N] "Node average water density";
        Units.Density rho_fg[N+1] "Node boundary flue gas density";
        Units.Density rho_fg_node[N] "Node average flue gas density";
        // Mass fraction
        Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
        Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
        // Temperature
        Units.Temperature T_water[N+1] "Node boundary water temperature";
        Units.Temperature T_water_node[N] "Node average water temperature";
        Units.Temperature T_fg[N+1] "Node boundary flue gas temperature";
        Units.Temperature T_fg_node[N] "Node average flue gas temperature";
        Units.Temperature T_wall[N] "Node wall temperature"; // add an init equation
        // Dynamic viscosities
        Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
        // Heat capacities Cp
        Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
        Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
        // Thermal conductivity
        Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";

      // Heat transfer parameters
        // Average velocities
        Units.Velocity U_water_node[N] "Node average water velocity";
        // Prandtl Number
        Real Pr_water[N] "Node water Prandtl's number";
        // Reynold's Number
        Real Re_water[N] "Node water Reynold's number";
        // Nusselt Nymber
        Real Nu_water[N] "Node water Nusselt number";
        // Convection heat transfer coefficient
        Units.HeatExchangeCoefficient K_conv_water[N](each start=K_conv_water_0) "Water side convection heat transfer coefficient";
        // Discretized heat transfer power
        Units.Power dW_water[N] "Node water heat exchange";
        Units.Power dW_fg[N] "Node flue gas heat exchange";
        // Biot Number
        Real Bi[N] "Node Biot number";

      // Parameters of interest
        Units.Temperature T_water_in "Water inlet temperature";
        Units.Temperature T_water_out "Water outlet temperature";
        Units.Temperature T_fg_in "Flue gas inlet temperature";
        Units.Temperature T_fg_out "Flue gas outlet temperature";
        Units.Temperature T_water_avg "Water overall average temperature";
        Units.Temperature T_fg_avg "Flue gas overall average temperature";
        Units.Temperature T_wall_avg "Wall overall average temperature";
        Units.Velocity U_water_avg "Water overall average velocity";
        Real Pr_water_avg "Water overall average Prandtl number";
        Real Re_water_avg "Water overall average Reynold's number";
        Real Nu_water_avg "Water overall average Reynold's number";
        Real K_conv_water_avg "Water side overall average convection heat transfer coefficient";
        Real Bi_max "Maximum Biot number";

      WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{90,30},{110,50}}),iconTransformation(extent={{90,30},{110,50}})));
      WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-110,32},{-90,52}}),iconTransformation(extent={{-110,32},{-90,52}})));
      FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{88,-50},{108,-30}}),iconTransformation(extent={{88,-50},{108,-30}})));
      FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-50},{-90,-30}}),iconTransformation(extent={{-110,-50},{-90,-30}})));
      WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,10},{-10,30}})));
      FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
    equation

      // ------ Boundaries ------
        // Enthalpy
        h_water[1] = water_side.h_in;
        h_water[N+1] = water_side.h_out;
        h_fg[1] = fg_side.h_out;
        h_fg[N+1] = fg_side.h_in;
        // Pressure
        P_water = water_side.P_in;
        P_fg = fg_side.P_in;
        // Mass flow rate
        Q_water = water_side.Q;
        Q_fg = fg_side.Q;
        // Mass Fraction
        Xi_water = water_side.Xi;
        Xi_fg = fg_side.Xi;

      // ------ First node properties ------
        state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
        state_fg[1] = FlueGasesMedium.setState_phX(P_fg, h_fg[1], Xi_fg);
        T_water[1] = WaterSteamMedium.temperature(state_water[1]);
        T_fg[1] = FlueGasesMedium.temperature(state_fg[1]);
        rho_water[1] = WaterSteamMedium.density(state_water[1]);
        rho_fg[1] = FlueGasesMedium.density(state_fg[1]);
        Mu_water[1] = WaterSteamMedium.dynamicViscosity(state_water[1]);
        Cp_water[1] = WaterSteamMedium.specificHeatCapacityCp(state_water[1]);
        k_water[1] = WaterSteamMedium.thermalConductivity(state_water[1]);

      // ------ Parameters of interest ------
        // IN/OUT temperatures
        T_water_in = water_side.T_in;
        T_water_out = water_side.T_out;
        T_fg_in = fg_side.T_in;
        T_fg_out = fg_side.T_out;
        // Average Temperatures
        T_water_avg = sum(T_water_node)/N;
        T_fg_avg = sum(T_fg_node)/N;
        T_wall_avg =  sum(T_wall)/N;
        // Average velocity
        U_water_avg = sum(U_water_node)/N;
        // Average heat transfer parameters
        Pr_water_avg = sum(Pr_water)/N;
        Re_water_avg = sum(Re_water)/N;
        Nu_water_avg = sum(Nu_water)/N;
        K_conv_water_avg = sum(K_conv_water)/N;
        // Biot number and validity of LCM verification
        Bi_max = max(Bi);
        assert(Bi_max < 0.1, "LCM is not valid",  AssertionLevel.warning);

      // ------ Discretization computation loop ------
        for i in 1:N loop

          // Fluids Properties
            // State
            state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
            state_fg[i+1] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1], Xi_fg);
            // Temperature
            T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
            T_fg[i+1] = FlueGasesMedium.temperature(state_fg[i+1]);
            T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
            T_fg_node[i] = 0.5*(T_fg[i] + T_fg[i+1]);
            // Density
            rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
            rho_fg[i+1] = FlueGasesMedium.density(state_fg[i+1]);
            rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
            rho_fg_node[i] = 0.5*(rho_fg[i] + rho_fg[i+1]);
            // Dynamic viscosity
            Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
            Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
            // Specific heat capacities Cp
            Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
            Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
            // Thermal conductivity
            k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
            k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);

          // Node energy balance
            // Water side
            dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
            // Flue gas side
            dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);
            // Global with wall storage
            dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

          // Convection heat transfer coefficient calculation
            // Average velocities
            U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
            // Prandtl number
            Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
            // Reynold's number
            Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
            // Nusselt number
            Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
            // Convection correlation: Dittus-Boelter equation
            Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

          // Convection heat transfer equations
            // Water side
            dW_water[i] = -K_conv_water[i]*dA_water*(T_water_node[i] - T_wall[i]);
            // Flue gas side
            dW_fg[i] = -K_conv_fg*dA_fg*(T_fg_node[i] - T_wall[i]);

          // Biot number
            Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

        end for;

    initial equation

      for i in 1:N loop
        der(T_wall[i]) = 0;
      end for;

    equation
      connect(water_side.C_in, water_inlet) annotation (Line(points={{10,20},{100,20},{100,40}},
                                                                                       color={28,108,200}));
      connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,20},{-100,20},{-100,42}},
                                                                                           color={28,108,200}));
      connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,-20},{100,-20},{100,-40},{98,-40}},
                                                                                     color={95,95,95}));
      connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,-20},{-100,-20},{-100,-40}},
                                                                                     color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,-10},{100,-60}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,10},{100,-10}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,60},{100,10}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-60,60},{-60,-60}},color={0,0,0}),
            Line(points={{-40,60},{-40,-60}},color={0,0,0}),
            Line(points={{40,60},{40,-60}},color={0,0,0}),
            Line(points={{60,60},{60,-60}},color={0,0,0}),
            Line(points={{80,60},{80,-60}},color={0,0,0}),
            Line(points={{-80,60},{-80,-60}},color={0,0,0}),
            Text(
              extent={{-20,4},{16,-4}},
              textColor={0,0,0},
              fontSize=20,
              textString="Tube Wall"),
            Line(
              points={{30,40},{-30,40}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Line(
              points={{-26,-38},{34,-38}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Text(
              extent={{-22,32},{14,24}},
              textColor={0,0,0},
              fontSize=20,
              textString="Water side"),
            Text(
              extent={{-30,-24},{26,-32}},
              textColor={0,0,0},
              fontSize=20,
              textString="Flue gas side")}),                         Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MonoPhasicHX_WaterHTC_Bi;

    model MonoPhasicHX_WaterHTC_Conduction
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      import MetroscopeModelingLibrary.Utilities.Constants;

      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      // Constants
        parameter Real pi = Constants.pi;

      // ------ Geometry ------
        // Pipes
        parameter Units.Length D_out = 0.03 "Pipe outer diameter";
        parameter Units.Length e = 0.003 "Pipe wall thickness";
        parameter Units.Length D_in = D_out - 2*e "Pipe inner diameter";
        parameter Units.Length L = 22 "Tube's length";
        parameter Integer N_tubes_row = 184 "Number of tubes per row";
        parameter Integer Rows = 2 "Number of tubes rows";
        parameter Integer N_tubes = N_tubes_row*Rows "Total number of tubes";
        parameter Modelica.Units.SI.ThermalConductivity K_cond_wall = 27 "Wall thermal conductivity";
        // Water side
        parameter Units.Area A_water = N_tubes*L*pi*D_in "Water side heat exchange surface";
        parameter Units.Area Ac_water = 0.25*pi*D_in^2*N_tubes_row "Water side cross sectional area";
        // Flue gases
        parameter Units.Area A_fg = 2800 "Flue gase side heat exchange surface";
        parameter Units.HeatExchangeCoefficient K_conv_fg = 82.06 "Flue gase convection heat transfer coefficient";
        // Wall
        parameter Units.Mass M_wall = 17800 "Tubes total mass";
        parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";

      // ------ Initialization ------
        parameter Units.Temperature T_wall_0 = 450;
        parameter Units.Pressure P_water_0 = 70e5;
        parameter Units.Pressure P_fg_0 = 1e5;
        parameter Units.PositiveMassFlowRate Q_water_0 = 85;
        parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
        parameter Units.Temperature T_water_out_0 = 500;
        parameter Units.Temperature T_fg_out_0 = 560;
        parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
        parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;
        parameter Units.HeatExchangeCoefficient K_conv_water_0 = 2400;

      // ------ Discretization ------
        parameter Integer N = 10;
        parameter Units.Mass dM_wall = M_wall/N "Tube mass of a single node";
        parameter Units.Area dA_water = A_water/N "Water side heat exchange surface of a single node";
        parameter Units.Area dA_fg = A_fg/N "Flue gas side heat exchange surface of a single node";
        parameter Units.Length dx = L/N;

      // ------ Fluids properties ------
        // State
        WaterSteamMedium.ThermodynamicState state_water[N+1] "Water side node boundary state";
        FlueGasesMedium.ThermodynamicState state_fg[N+1] "Flue gas side node boundary states";
        // Enthalpy
        Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
        Units.SpecificEnthalpy h_fg[N+1] "Flue gas specific enthalpy";
        // Mass flow rate
        Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
        Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
        // Pressure
        Units.Pressure P_water(start=P_water_0) "Water Pressure";
        Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
        // Density
        Units.Density rho_water[N+1] "Node boundary water density";
        Units.Density rho_water_node[N] "Node average water density";
        Units.Density rho_fg[N+1] "Node boundary flue gas density";
        Units.Density rho_fg_node[N] "Node average flue gas density";
        // Mass fraction
        Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
        Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
        // Temperature
        Units.Temperature T_water[N+1] "Node boundary water temperature";
        Units.Temperature T_water_node[N] "Node average water temperature";
        Units.Temperature T_fg[N+1] "Node boundary flue gas temperature";
        Units.Temperature T_fg_node[N] "Node average flue gas temperature";
        // Dynamic viscosities
        Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
        // Heat capacities Cp
        Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
        Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
        // Thermal conductivity
        Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";

      // ------ Conduction variables ------
        Units.Temperature T_wall_water[N] "Wall temperature from the water side";
        Units.Temperature T_wall[N] "Node wall average temperature";
        Units.Temperature T_wall_fg[N] "Wall temperature from the flue gas side";

      // Heat transfer parameters
        // Average velocities
        Units.Velocity U_water_node[N] "Node average water velocity";
        // Prandtl Number
        Real Pr_water[N] "Node water Prandtl's number";
        // Reynold's Number
        Real Re_water[N] "Node water Reynold's number";
        // Nusselt Nymber
        Real Nu_water[N] "Node water Nusselt number";
        // Convection heat transfer coefficient
        Units.HeatExchangeCoefficient K_conv_water[N](each start=K_conv_water_0) "Water side convection heat transfer coefficient";
        // Discretized heat transfer power
        Units.Power dW_water[N] "Node water heat exchange";
        Units.Power dW_fg[N] "Node flue gas heat exchange";
        // Biot Number
        Real Bi[N] "Node Biot number";

      // Parameters of interest
        Units.Temperature T_water_in "Water inlet temperature";
        Units.Temperature T_water_out "Water outlet temperature";
        Units.Temperature T_fg_in "Flue gas inlet temperature";
        Units.Temperature T_fg_out "Flue gas outlet temperature";
        Units.Temperature T_water_avg "Water overall average temperature";
        Units.Temperature T_fg_avg "Flue gas overall average temperature";
        Units.Temperature T_wall_avg "Wall overall average temperature";
        Units.Velocity U_water_avg "Water overall average velocity";
        Real Pr_water_avg "Water overall average Prandtl number";
        Real Re_water_avg "Water overall average Reynold's number";
        Real Nu_water_avg "Water overall average Reynold's number";
        Real K_conv_water_avg "Water side overall average convection heat transfer coefficient";
        Real Bi_max "Maximum Biot number";

      WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{90,30},{110,50}}),iconTransformation(extent={{90,30},{110,50}})));
      WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-110,32},{-90,52}}),iconTransformation(extent={{-110,32},{-90,52}})));
      FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{88,-50},{108,-30}}),iconTransformation(extent={{88,-50},{108,-30}})));
      FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-50},{-90,-30}}),iconTransformation(extent={{-110,-50},{-90,-30}})));
      WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,10},{-10,30}})));
      FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
    equation

      // ------ Boundaries ------
        // Enthalpy
        h_water[1] = water_side.h_in;
        h_water[N+1] = water_side.h_out;
        h_fg[1] = fg_side.h_out;
        h_fg[N+1] = fg_side.h_in;
        // Pressure
        P_water = water_side.P_in;
        P_fg = fg_side.P_in;
        // Mass flow rate
        Q_water = water_side.Q;
        Q_fg = fg_side.Q;
        // Mass Fraction
        Xi_water = water_side.Xi;
        Xi_fg = fg_side.Xi;

      // ------ First node properties ------
        state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
        state_fg[1] = FlueGasesMedium.setState_phX(P_fg, h_fg[1], Xi_fg);
        T_water[1] = WaterSteamMedium.temperature(state_water[1]);
        T_fg[1] = FlueGasesMedium.temperature(state_fg[1]);
        rho_water[1] = WaterSteamMedium.density(state_water[1]);
        rho_fg[1] = FlueGasesMedium.density(state_fg[1]);
        Mu_water[1] = WaterSteamMedium.dynamicViscosity(state_water[1]);
        Cp_water[1] = WaterSteamMedium.specificHeatCapacityCp(state_water[1]);
        k_water[1] = WaterSteamMedium.thermalConductivity(state_water[1]);

      // ------ Parameters of interest ------
        // IN/OUT temperatures
        T_water_in = water_side.T_in;
        T_water_out = water_side.T_out;
        T_fg_in = fg_side.T_in;
        T_fg_out = fg_side.T_out;
        // Average Temperatures
        T_water_avg = sum(T_water_node)/N;
        T_fg_avg = sum(T_fg_node)/N;
        T_wall_avg =  sum(T_wall)/N;
        // Average velocity
        U_water_avg = sum(U_water_node)/N;
        // Average heat transfer parameters
        Pr_water_avg = sum(Pr_water)/N;
        Re_water_avg = sum(Re_water)/N;
        Nu_water_avg = sum(Nu_water)/N;
        K_conv_water_avg = sum(K_conv_water)/N;
        // Biot number and validity of LCM verification
        Bi_max = max(Bi);
        assert(Bi_max < 0.1, "LCM is not valid",  AssertionLevel.warning);

      // ------ Discretization computation loop ------
        for i in 1:N loop

          // Fluids Properties
            // State
            state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
            state_fg[i+1] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1], Xi_fg);
            // Temperature
            T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
            T_fg[i+1] = FlueGasesMedium.temperature(state_fg[i+1]);
            T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
            T_fg_node[i] = 0.5*(T_fg[i] + T_fg[i+1]);
            // Density
            rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
            rho_fg[i+1] = FlueGasesMedium.density(state_fg[i+1]);
            rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
            rho_fg_node[i] = 0.5*(rho_fg[i] + rho_fg[i+1]);
            // Dynamic viscosity
            Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
            Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
            // Specific heat capacities Cp
            Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
            Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
            // Thermal conductivity
            k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
            k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);

          // Conduction heat transfer
            dW_water[i] = 2*pi*K_cond_wall*dx*L*(T_wall[i] - T_wall_water[i])/(Modelica.Math.log(1 + e/D_in));
            dW_fg[i] = 2*pi*K_cond_wall*dx*L*(T_wall[i] - T_wall_fg[i])/(Modelica.Math.log(1 + e/(e + D_in)));

          // Node energy balance
            // Water side
            dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
            // Flue gas side
            dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);
            // Global with wall storage
            dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

          // Convection heat transfer coefficient calculation
            // Average velocities
            U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
            // Prandtl number
            Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
            // Reynold's number
            Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
            // Nusselt number
            Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
            // Convection correlation: Dittus-Boelter equation
            Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

          // Convection heat transfer equations
            // Water side
            dW_water[i] = K_conv_water[i]*dA_water*(T_wall_water[i] - T_water_node[i]);
            // Flue gas side
            dW_fg[i] = K_conv_fg*dA_fg*(T_wall_fg[i] - T_fg_node[i]);

          // Biot number
            Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

        end for;

    initial equation

      for i in 1:N loop
        der(T_wall[i]) = 0;
      end for;

    equation
      connect(water_side.C_in, water_inlet) annotation (Line(points={{10,20},{100,20},{100,40}},
                                                                                       color={28,108,200}));
      connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,20},{-100,20},{-100,42}},
                                                                                           color={28,108,200}));
      connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,-20},{100,-20},{100,-40},{98,-40}},
                                                                                     color={95,95,95}));
      connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,-20},{-100,-20},{-100,-40}},
                                                                                     color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,-10},{100,-60}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,10},{100,-10}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,60},{100,10}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-60,60},{-60,-60}},color={0,0,0}),
            Line(points={{-40,60},{-40,-60}},color={0,0,0}),
            Line(points={{40,60},{40,-60}},color={0,0,0}),
            Line(points={{60,60},{60,-60}},color={0,0,0}),
            Line(points={{80,60},{80,-60}},color={0,0,0}),
            Line(points={{-80,60},{-80,-60}},color={0,0,0}),
            Text(
              extent={{-20,4},{16,-4}},
              textColor={0,0,0},
              fontSize=20,
              textString="Tube Wall"),
            Line(
              points={{30,40},{-30,40}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Line(
              points={{-26,-38},{34,-38}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Text(
              extent={{-22,32},{14,24}},
              textColor={0,0,0},
              fontSize=20,
              textString="Water side"),
            Text(
              extent={{-30,-24},{26,-32}},
              textColor={0,0,0},
              fontSize=20,
              textString="Flue gas side")}),                         Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MonoPhasicHX_WaterHTC_Conduction;

    model MonoPhasicHX_v1
     import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      import MetroscopeModelingLibrary.Utilities.Constants;
      import CorrelationConstants =
             MetroscopeModelingLibrary.DynamicComponents.Correlations;

      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      // Constants
        parameter Real pi = Constants.pi;

      // ------ Geometry ------
        // Pipes
        parameter Units.Length D_out = 0.03 "Pipe outer diameter";
        parameter Units.Length e = 0.003 "Pipe wall thickness";
        parameter Units.Length D_in = D_out - 2*e "Pipe inner diameter";
        parameter Units.Length L = 22 "Tube's length";
        parameter Integer N_tubes_row = 184 "Number of tubes per row";
        parameter Integer Rows = 2 "Number of tubes rows";
        parameter Integer N_tubes = N_tubes_row*Rows "Total number of tubes";
        parameter Modelica.Units.SI.ThermalConductivity K_cond_wall = 27 "Wall thermal conductivity";
        parameter Integer Tubes_Config = 2 "1: aligned, 2: staggered";
        parameter Units.Length fg_path_width = 14.07 "Flue gas path transverse width";
        parameter Units.Length S_T = 76.25e-3 "Transverse pitch";
        parameter Units.Length S_L = 95.25e-3 "Longitudinal pitch";
        parameter Units.Length S_D = (S_L^2 + (S_T/2)^2)^0.5 "Diagonal pitch";
        // Water side
        parameter Units.Area A_water = N_tubes*L*pi*D_in "Water side heat exchange surface";
        parameter Units.Area Ac_water = 0.25*pi*D_in^2*N_tubes_row "Water side cross sectional area";
        // Flue gases
        parameter Units.Area A_fg = 2800 "Flue gase side heat exchange surface";
        parameter Units.Area Af_fg = L*fg_path_width "Flue gas frontal area";
        Units.HeatExchangeCoefficient K_conv_fg "Flue gase convection heat transfer coefficient"; //  = 82.06
        // Wall
        parameter Units.Mass M_wall = 17800 "Tubes total mass";
        parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";

      // ------ Initialization ------
        parameter Units.Temperature T_wall_0 = 450;
        parameter Units.Pressure P_water_0 = 70e5;
        parameter Units.Pressure P_fg_0 = 1e5;
        parameter Units.PositiveMassFlowRate Q_water_0 = 85;
        parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
        parameter Units.Temperature T_water_out_0 = 500;
        parameter Units.Temperature T_fg_out_0 = 560;
        parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
        parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;
        parameter Units.HeatExchangeCoefficient K_conv_water_0 = 2400;

      // ------ Discretization ------
        parameter Integer N = 10;
        parameter Units.Mass dM_wall = M_wall/N "Tube mass of a single node";
        parameter Units.Area dA_water = A_water/N "Water side heat exchange surface of a single node";
        parameter Units.Area dA_fg = A_fg/N "Flue gas side heat exchange surface of a single node";
        parameter Units.Length dx = L/N;

      // ------ Fluids properties ------
        // State
        WaterSteamMedium.ThermodynamicState state_water[N+1] "Water side node boundary state";
        FlueGasesMedium.ThermodynamicState state_fg[N+1] "Flue gas side node boundary states";
        // Enthalpy
        Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
        Units.SpecificEnthalpy h_fg[N+1] "Flue gas specific enthalpy";
        // Mass flow rate
        Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
        Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
        // Pressure
        Units.Pressure P_water(start=P_water_0) "Water Pressure";
        Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
        // Density
        Units.Density rho_water[N+1] "Node boundary water density";
        Units.Density rho_water_node[N] "Node average water density";
        Units.Density rho_fg[N+1] "Node boundary flue gas density";
        Units.Density rho_fg_node[N] "Node average flue gas density";
        // Mass fraction
        Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
        Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
        // Temperature
        Units.Temperature T_water[N+1] "Node boundary water temperature";
        Units.Temperature T_water_node[N] "Node average water temperature";
        Units.Temperature T_fg[N+1] "Node boundary flue gas temperature";
        Units.Temperature T_fg_node[N] "Node average flue gas temperature";
        // Dynamic viscosities
        Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_fg[N+1] "Node boundary flue gas dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_fg_node[N] "Node average flue gas dynamic viscosity";
        // Heat capacities Cp
        Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
        Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
        Units.HeatCapacity Cp_fg[N+1] "Node boundary flue gas Cp";
        Units.HeatCapacity Cp_fg_node[N] "Node average flue gas Cp";
        // Thermal conductivity
        Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_fg[N+1] "Node boundary flue gas thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_fg_node[N] "Node average flue gas thermal conductivity";

      // ------ Conduction variables ------
        Units.Temperature T_wall_water[N] "Wall temperature from the water side";
        Units.Temperature T_wall[N] "Node wall average temperature";
        Units.Temperature T_wall_fg[N] "Wall temperature from the flue gas side";

      // ------ Tubes configuration parameters ------
        Units.Velocity U_fg_face "Flue gas face velocity";
        Units.Velocity U_fg_max "Flue gas maximum velocity";
        Real Re_fg_max "Flue gas maximum Reynold's number";
        Real Pr_fg "Flue gas overall average Prandtl number";
        FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
        Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
        Real Nu_fg_avg "Flue gass overall average Nusselt number";

      // ------ Heat transfer parameters ------
        // Average velocities
        Units.Velocity U_water_node[N] "Node average water velocity";
        // Prandtl Number
        Real Pr_water[N] "Node water Prandtl's number";
        // Reynold's Number
        Real Re_water[N] "Node water Reynold's number";
        // Nusselt Nymber
        Real Nu_water[N] "Node water Nusselt number";
        // Convection heat transfer coefficient
        Units.HeatExchangeCoefficient K_conv_water[N](each start=K_conv_water_0) "Water side convection heat transfer coefficient";
        // Discretized heat transfer power
        Units.Power dW_water[N] "Node water heat exchange";
        Units.Power dW_fg[N] "Node flue gas heat exchange";
        // Biot Number
        Real Bi[N] "Node Biot number";

      // Parameters of interest
        Units.Temperature T_water_in "Water inlet temperature";
        Units.Temperature T_water_out "Water outlet temperature";
        Units.Temperature T_fg_in "Flue gas inlet temperature";
        Units.Temperature T_fg_out "Flue gas outlet temperature";
        Units.Temperature T_water_avg "Water overall average temperature";
        Units.Temperature T_fg_avg "Flue gas overall average temperature";
        Units.Temperature T_wall_avg "Wall overall average temperature";
        Units.Velocity U_water_avg "Water overall average velocity";
        Real Pr_water_avg "Water overall average Prandtl number";
        Real Re_water_avg "Water overall average Reynold's number";
        Real Nu_water_avg "Water overall average Nusselt number";
        Real K_conv_water_avg "Water side overall average convection heat transfer coefficient";
        Real Bi_max "Maximum Biot number";

      WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{90,30},{110,50}}),iconTransformation(extent={{90,30},{110,50}})));
      WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-110,32},{-90,52}}),iconTransformation(extent={{-110,32},{-90,52}})));
      FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{88,-50},{108,-30}}),iconTransformation(extent={{88,-50},{108,-30}})));
      FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-50},{-90,-30}}),iconTransformation(extent={{-110,-50},{-90,-30}})));
      WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,10},{-10,30}})));
      FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
    equation

      // ------ Boundaries ------
        // Enthalpy
        h_water[1] = water_side.h_in;
        h_water[N+1] = water_side.h_out;
        h_fg[1] = fg_side.h_out;
        h_fg[N+1] = fg_side.h_in;
        // Pressure
        P_water = water_side.P_in;
        P_fg = fg_side.P_in;
        // Mass flow rate
        Q_water = water_side.Q;
        Q_fg = fg_side.Q;
        // Mass Fraction
        Xi_water = water_side.Xi;
        Xi_fg = fg_side.Xi;

      // ------ Tubes configuration parameters ------
        // Flue gas face velocity
        U_fg_face = Q_fg/(rho_fg[N+1]*Af_fg);
        // Flue gas maximum velocity
        if (Tubes_Config == 1) then
          U_fg_max = S_T*U_fg_face/(S_T - D_out);
        elseif (S_D < (S_T + D_out)/2) then
          U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
        else
          U_fg_max = S_T*U_fg_face/(S_T - D_out);
        end if;
        // Flue gas maximum Reynold's number
        Re_fg_max = rho_fg[N+1]*U_fg_max*D_out/Mu_fg[N+1];
        // Flue gas Prandtl number
        Pr_fg = Cp_fg[N+1]*Mu_fg[N+1]/k_fg[N+1];
        // Flue gas Prandtl number at surface temperature
        state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
        Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
        // Convection coefficient calculation
        Nu_fg_avg = K_conv_fg*D_out/k_fg[N+1];
        Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);


      // ------ First node properties ------
        state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
        state_fg[1] = FlueGasesMedium.setState_phX(P_fg, h_fg[1], Xi_fg);
        T_water[1] = WaterSteamMedium.temperature(state_water[1]);
        T_fg[1] = FlueGasesMedium.temperature(state_fg[1]);
        rho_water[1] = WaterSteamMedium.density(state_water[1]);
        rho_fg[1] = FlueGasesMedium.density(state_fg[1]);
        Mu_water[1] = WaterSteamMedium.dynamicViscosity(state_water[1]);
        Mu_fg[1] = FlueGasesMedium.dynamicViscosity(state_fg[1]);
        Cp_water[1] = WaterSteamMedium.specificHeatCapacityCp(state_water[1]);
        Cp_fg[1] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1]);
        k_water[1] = WaterSteamMedium.thermalConductivity(state_water[1]);
        k_fg[1] = FlueGasesMedium.thermalConductivity(state_fg[1]);

      // ------ Parameters of interest ------
        // IN/OUT temperatures
        T_water_in = water_side.T_in;
        T_water_out = water_side.T_out;
        T_fg_in = fg_side.T_in;
        T_fg_out = fg_side.T_out;
        // Average Temperatures
        T_water_avg = sum(T_water_node)/N;
        T_fg_avg = sum(T_fg_node)/N;
        T_wall_avg =  sum(T_wall)/N;
        // Average velocity
        U_water_avg = sum(U_water_node)/N;
        // Average heat transfer parameters
        Pr_water_avg = sum(Pr_water)/N;
        Re_water_avg = sum(Re_water)/N;
        Nu_water_avg = sum(Nu_water)/N;
        K_conv_water_avg = sum(K_conv_water)/N;
        // Biot number and validity of LCM verification
        Bi_max = max(Bi);
        assert(Bi_max < 0.1, "LCM is not valid",  AssertionLevel.warning);

      // ------ Discretization computation loop ------
        for i in 1:N loop

          // Fluids Properties
            // State
            state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
            state_fg[i+1] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1], Xi_fg);
            // Temperature
            T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
            T_fg[i+1] = FlueGasesMedium.temperature(state_fg[i+1]);
            T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
            T_fg_node[i] = 0.5*(T_fg[i] + T_fg[i+1]);
            // Density
            rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
            rho_fg[i+1] = FlueGasesMedium.density(state_fg[i+1]);
            rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
            rho_fg_node[i] = 0.5*(rho_fg[i] + rho_fg[i+1]);
            // Dynamic viscosity
            Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
            Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
            Mu_fg[i+1] = FlueGasesMedium.dynamicViscosity(state_fg[i+1]);
            Mu_fg_node[i] = 0.5*(Mu_fg[i] + Mu_fg[i+1]);
            // Specific heat capacities Cp
            Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
            Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
            Cp_fg[i+1] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1]);
            Cp_fg_node[i] = 0.5*(Cp_fg[i] + Cp_fg[i+1]);
            // Thermal conductivity
            k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
            k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);
            k_fg[i+1] = FlueGasesMedium.thermalConductivity(state_fg[i+1]);
            k_fg_node[i] = 0.5*(k_fg[i] + k_fg[i+1]);

          // Conduction heat transfer
            dW_water[i] = 2*pi*K_cond_wall*dx*L*(T_wall[i] - T_wall_water[i])/(Modelica.Math.log(1 + e/D_in));
            dW_fg[i] = 2*pi*K_cond_wall*dx*L*(T_wall[i] - T_wall_fg[i])/(Modelica.Math.log(1 + e/(e + D_in)));

          // Node energy balance
            // Water side
            dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
            // Flue gas side
            dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);
            // Global with wall storage
            dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

          // Convection heat transfer coefficient calculation
            // Average velocities
            U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
            // Prandtl number
            Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
            // Reynold's number
            Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
            // Nusselt number
            Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
            // Convection correlation: Dittus-Boelter equation
            Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

          // Convection heat transfer equations
            // Water side
            dW_water[i] = K_conv_water[i]*dA_water*(T_wall_water[i] - T_water_node[i]);
            // Flue gas side
            dW_fg[i] = K_conv_fg*dA_fg*(T_wall_fg[i] - T_fg_node[i]);

          // Biot number
            Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

        end for;

    initial equation

      for i in 1:N loop
        der(T_wall[i]) = 0;
      end for;

    equation
      connect(water_side.C_in, water_inlet) annotation (Line(points={{10,20},{100,20},{100,40}},
                                                                                       color={28,108,200}));
      connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,20},{-100,20},{-100,42}},
                                                                                           color={28,108,200}));
      connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,-20},{100,-20},{100,-40},{98,-40}},
                                                                                     color={95,95,95}));
      connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,-20},{-100,-20},{-100,-40}},
                                                                                     color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,-10},{100,-60}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,10},{100,-10}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,60},{100,10}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-60,60},{-60,-60}},color={0,0,0}),
            Line(points={{-40,60},{-40,-60}},color={0,0,0}),
            Line(points={{40,60},{40,-60}},color={0,0,0}),
            Line(points={{60,60},{60,-60}},color={0,0,0}),
            Line(points={{80,60},{80,-60}},color={0,0,0}),
            Line(points={{-80,60},{-80,-60}},color={0,0,0}),
            Text(
              extent={{-20,4},{16,-4}},
              textColor={0,0,0},
              fontSize=20,
              textString="Tube Wall"),
            Line(
              points={{30,40},{-30,40}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Line(
              points={{-26,-38},{34,-38}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Text(
              extent={{-22,32},{14,24}},
              textColor={0,0,0},
              fontSize=20,
              textString="Water side"),
            Text(
              extent={{-30,-24},{26,-32}},
              textColor={0,0,0},
              fontSize=20,
              textString="Flue gas side")}),                         Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MonoPhasicHX_v1;

    model MonoPhasicHX_v2 "Added more info on fins and used ESCOA correlation"
     import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      import MetroscopeModelingLibrary.Utilities.Constants;
      import CorrelationConstants =
             MetroscopeModelingLibrary.DynamicComponents.Correlations;

      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      // Constants
        parameter Real pi = Constants.pi;

      // ------ Geometry ------
        // Pipes
        parameter Units.Length D_out = 0.03 "Pipe outer diameter";
        parameter Units.Length e = 0.003 "Pipe wall thickness";
        parameter Units.Length D_in = D_out - 2*e "Pipe inner diameter";
        parameter Units.Length L = 22 "Tube's length";
        parameter Integer N_tubes_row = 184 "Number of tubes per row";
        parameter Integer Rows = 2 "Number of tubes rows";
        parameter Integer N_tubes = N_tubes_row*Rows "Total number of tubes";
        parameter Modelica.Units.SI.ThermalConductivity K_cond_wall = 27 "Wall thermal conductivity";
        parameter Integer Tubes_Config = 2 "1: aligned, 2: staggered";
        parameter Units.Length fg_path_width = 14.07 "Flue gas path transverse width";
        parameter Units.Length S_T = 76.25e-3 "Transverse pitch";
        parameter Units.Length S_L = 95.25e-3 "Longitudinal pitch";
        parameter Units.Length S_D = (S_L^2 + (S_T/2)^2)^0.5 "Diagonal pitch";
        parameter Units.Length S_f = 0.009506 "Fins pitch";
        parameter Units.Length H_fin = 0.009525 "Fin height";
        parameter Units.Length e_fin = 0.0009906 "Fin thickness";
        // Water side
        parameter Units.Area A_water = N_tubes*L*pi*D_in "Water side heat exchange surface";
        parameter Units.Area Ac_water = 0.25*pi*D_in^2*N_tubes_row "Water side cross sectional area";
        // Flue gases
        parameter Units.Area A_fg = 2800 "Flue gase side heat exchange surface";
        parameter Units.Area Af_fg = L*fg_path_width "Flue gas frontal area";
        Units.HeatExchangeCoefficient K_conv_fg "Flue gase convection heat transfer coefficient"; //  = 82.06
        // Wall
        parameter Units.Mass M_wall = 17800 "Tubes total mass";
        parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";

      // ------ Initialization ------
        parameter Units.Temperature T_wall_0 = 450;
        parameter Units.Pressure P_water_0 = 70e5;
        parameter Units.Pressure P_fg_0 = 1e5;
        parameter Units.PositiveMassFlowRate Q_water_0 = 85;
        parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
        parameter Units.Temperature T_water_out_0 = 500;
        parameter Units.Temperature T_fg_out_0 = 560;
        parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
        parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;
        parameter Units.HeatExchangeCoefficient K_conv_water_0 = 2400;

      // ------ Discretization ------
        parameter Integer N = 10;
        parameter Units.Mass dM_wall = M_wall/N "Tube mass of a single node";
        parameter Units.Area dA_water = A_water/N "Water side heat exchange surface of a single node";
        parameter Units.Area dA_fg = A_fg/N "Flue gas side heat exchange surface of a single node";
        parameter Units.Length dx = L/N;

      // ------ Fluids properties ------
        // State
        WaterSteamMedium.ThermodynamicState state_water[N+1] "Water side node boundary state";
        FlueGasesMedium.ThermodynamicState state_fg[N+1] "Flue gas side node boundary states";
        // Enthalpy
        Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
        Units.SpecificEnthalpy h_fg[N+1] "Flue gas specific enthalpy";
        // Mass flow rate
        Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
        Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
        // Pressure
        Units.Pressure P_water(start=P_water_0) "Water Pressure";
        Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
        // Density
        Units.Density rho_water[N+1] "Node boundary water density";
        Units.Density rho_water_node[N] "Node average water density";
        Units.Density rho_fg[N+1] "Node boundary flue gas density";
        Units.Density rho_fg_node[N] "Node average flue gas density";
        // Mass fraction
        Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
        Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
        // Temperature
        Units.Temperature T_water[N+1] "Node boundary water temperature";
        Units.Temperature T_water_node[N] "Node average water temperature";
        Units.Temperature T_fg[N+1] "Node boundary flue gas temperature";
        Units.Temperature T_fg_node[N] "Node average flue gas temperature";
        // Dynamic viscosities
        Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_fg[N+1] "Node boundary flue gas dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_fg_node[N] "Node average flue gas dynamic viscosity";
        // Heat capacities Cp
        Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
        Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
        Units.HeatCapacity Cp_fg[N+1] "Node boundary flue gas Cp";
        Units.HeatCapacity Cp_fg_node[N] "Node average flue gas Cp";
        // Thermal conductivity
        Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_fg[N+1] "Node boundary flue gas thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_fg_node[N] "Node average flue gas thermal conductivity";

      // ------ Conduction variables ------
        Units.Temperature T_wall_water[N] "Wall temperature from the water side";
        Units.Temperature T_wall[N] "Node wall average temperature";
        Units.Temperature T_wall_fg[N] "Wall temperature from the flue gas side";

      // ------ Tubes configuration parameters ------
        Units.Velocity U_fg_face "Flue gas face velocity";
        Units.Velocity U_fg_max "Flue gas maximum velocity";
        Real Re_fg_max "Flue gas maximum Reynold's number";
        Real Pr_fg "Flue gas overall average Prandtl number";
        FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
        Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
        Real Nu_fg_avg "Flue gass overall average Nusselt number";

      // ------ Heat transfer parameters ------
        // Average velocities
        Units.Velocity U_water_node[N] "Node average water velocity";
        // Prandtl Number
        Real Pr_water[N] "Node water Prandtl's number";
        // Reynold's Number
        Real Re_water[N] "Node water Reynold's number";
        // Nusselt Nymber
        Real Nu_water[N] "Node water Nusselt number";
        // Convection heat transfer coefficient
        Units.HeatExchangeCoefficient K_conv_water[N](each start=K_conv_water_0) "Water side convection heat transfer coefficient";
        // Discretized heat transfer power
        Units.Power dW_water[N] "Node water heat exchange";
        Units.Power dW_fg[N] "Node flue gas heat exchange";
        // Biot Number
        Real Bi[N] "Node Biot number";

      // Parameters of interest
        Units.Temperature T_water_in "Water inlet temperature";
        Units.Temperature T_water_out "Water outlet temperature";
        Units.Temperature T_fg_in "Flue gas inlet temperature";
        Units.Temperature T_fg_out "Flue gas outlet temperature";
        Units.Temperature T_water_avg "Water overall average temperature";
        Units.Temperature T_fg_avg "Flue gas overall average temperature";
        Units.Temperature T_wall_avg "Wall overall average temperature";
        Units.Velocity U_water_avg "Water overall average velocity";
        Real Pr_water_avg "Water overall average Prandtl number";
        Real Re_water_avg "Water overall average Reynold's number";
        Real Nu_water_avg "Water overall average Nusselt number";
        Real K_conv_water_avg "Water side overall average convection heat transfer coefficient";
        Real Bi_max "Maximum Biot number";

      WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{90,30},{110,50}}),iconTransformation(extent={{90,30},{110,50}})));
      WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-110,32},{-90,52}}),iconTransformation(extent={{-110,32},{-90,52}})));
      FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{88,-50},{108,-30}}),iconTransformation(extent={{88,-50},{108,-30}})));
      FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-50},{-90,-30}}),iconTransformation(extent={{-110,-50},{-90,-30}})));
      WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,10},{-10,30}})));
      FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
    equation

      // ------ Boundaries ------
        // Enthalpy
        h_water[1] = water_side.h_in;
        h_water[N+1] = water_side.h_out;
        h_fg[1] = fg_side.h_out;
        h_fg[N+1] = fg_side.h_in;
        // Pressure
        P_water = water_side.P_in;
        P_fg = fg_side.P_in;
        // Mass flow rate
        Q_water = water_side.Q;
        Q_fg = fg_side.Q;
        // Mass Fraction
        Xi_water = water_side.Xi;
        Xi_fg = fg_side.Xi;

      // ------ Tubes configuration parameters ------
        // Flue gas face velocity
        U_fg_face = Q_fg/(rho_fg[N+1]*Af_fg);
        // Flue gas maximum velocity
        if (Tubes_Config == 1) then
          U_fg_max = S_T*U_fg_face/(S_T - D_out);
        elseif (S_D < (S_T + D_out)/2) then
          U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
        else
          U_fg_max = S_T*U_fg_face/(S_T - D_out);
        end if;
        // Flue gas maximum Reynold's number
        Re_fg_max = rho_fg[N+1]*U_fg_max*D_out/Mu_fg[N+1];
        // Flue gas Prandtl number
        Pr_fg = Cp_fg[N+1]*Mu_fg[N+1]/k_fg[N+1];
        // Flue gas Prandtl number at surface temperature
        state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
        Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
        // Convection coefficient calculation
        Nu_fg_avg= K_conv_fg*D_out/k_fg[N+1];
        // Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);
        Nu_fg_avg =CorrelationConstants.ESCOA(Re_fg_max, Pr_fg, Rows, T_fg_avg, T_wall_avg, D_out, H_fin, e_fin, S_f, S_T, S_L);

      // ------ First node properties ------
        state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
        state_fg[1] = FlueGasesMedium.setState_phX(P_fg, h_fg[1], Xi_fg);
        T_water[1] = WaterSteamMedium.temperature(state_water[1]);
        T_fg[1] = FlueGasesMedium.temperature(state_fg[1]);
        rho_water[1] = WaterSteamMedium.density(state_water[1]);
        rho_fg[1] = FlueGasesMedium.density(state_fg[1]);
        Mu_water[1] = WaterSteamMedium.dynamicViscosity(state_water[1]);
        Mu_fg[1] = FlueGasesMedium.dynamicViscosity(state_fg[1]);
        Cp_water[1] = WaterSteamMedium.specificHeatCapacityCp(state_water[1]);
        Cp_fg[1] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1]);
        k_water[1] = WaterSteamMedium.thermalConductivity(state_water[1]);
        k_fg[1] = FlueGasesMedium.thermalConductivity(state_fg[1]);

      // ------ Parameters of interest ------
        // IN/OUT temperatures
        T_water_in = water_side.T_in;
        T_water_out = water_side.T_out;
        T_fg_in = fg_side.T_in;
        T_fg_out = fg_side.T_out;
        // Average Temperatures
        T_water_avg = sum(T_water_node)/N;
        T_fg_avg = sum(T_fg_node)/N;
        T_wall_avg =  sum(T_wall)/N;
        // Average velocity
        U_water_avg = sum(U_water_node)/N;
        // Average heat transfer parameters
        Pr_water_avg = sum(Pr_water)/N;
        Re_water_avg = sum(Re_water)/N;
        Nu_water_avg = sum(Nu_water)/N;
        K_conv_water_avg = sum(K_conv_water)/N;
        // Biot number and validity of LCM verification
        Bi_max = max(Bi);
        assert(Bi_max < 0.1, "LCM is not valid",  AssertionLevel.warning);

      // ------ Discretization computation loop ------
        for i in 1:N loop

          // Fluids Properties
            // State
            state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
            state_fg[i+1] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1], Xi_fg);
            // Temperature
            T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
            T_fg[i+1] = FlueGasesMedium.temperature(state_fg[i+1]);
            T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
            T_fg_node[i] = 0.5*(T_fg[i] + T_fg[i+1]);
            // Density
            rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
            rho_fg[i+1] = FlueGasesMedium.density(state_fg[i+1]);
            rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
            rho_fg_node[i] = 0.5*(rho_fg[i] + rho_fg[i+1]);
            // Dynamic viscosity
            Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
            Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
            Mu_fg[i+1] = FlueGasesMedium.dynamicViscosity(state_fg[i+1]);
            Mu_fg_node[i] = 0.5*(Mu_fg[i] + Mu_fg[i+1]);
            // Specific heat capacities Cp
            Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
            Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
            Cp_fg[i+1] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1]);
            Cp_fg_node[i] = 0.5*(Cp_fg[i] + Cp_fg[i+1]);
            // Thermal conductivity
            k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
            k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);
            k_fg[i+1] = FlueGasesMedium.thermalConductivity(state_fg[i+1]);
            k_fg_node[i] = 0.5*(k_fg[i] + k_fg[i+1]);

          // Conduction heat transfer
            dW_water[i] = 2*pi*K_cond_wall*dx*L*(T_wall[i] - T_wall_water[i])/(Modelica.Math.log(1 + e/D_in));
            dW_fg[i] = 2*pi*K_cond_wall*dx*L*(T_wall[i] - T_wall_fg[i])/(Modelica.Math.log(1 + e/(e + D_in)));

          // Node energy balance
            // Water side
            dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
            // Flue gas side
            dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);
            // Global with wall storage
            dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

          // Convection heat transfer coefficient calculation
            // Average velocities
            U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
            // Prandtl number
            Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
            // Reynold's number
            Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
            // Nusselt number
            Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
            // Convection correlation: Dittus-Boelter equation
            Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

          // Convection heat transfer equations
            // Water side
            dW_water[i] = K_conv_water[i]*dA_water*(T_wall_water[i] - T_water_node[i]);
            // Flue gas side
            dW_fg[i] = (K_conv_fg+4)*dA_fg*(T_wall_fg[i] - T_fg_node[i]);

          // Biot number
            Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

        end for;

    initial equation

      for i in 1:N loop
        der(T_wall[i]) = 0;
      end for;

    equation
      connect(water_side.C_in, water_inlet) annotation (Line(points={{10,20},{100,20},{100,40}},
                                                                                       color={28,108,200}));
      connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,20},{-100,20},{-100,42}},
                                                                                           color={28,108,200}));
      connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,-20},{100,-20},{100,-40},{98,-40}},
                                                                                     color={95,95,95}));
      connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,-20},{-100,-20},{-100,-40}},
                                                                                     color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,-10},{100,-60}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,10},{100,-10}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,60},{100,10}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-60,60},{-60,-60}},color={0,0,0}),
            Line(points={{-40,60},{-40,-60}},color={0,0,0}),
            Line(points={{40,60},{40,-60}},color={0,0,0}),
            Line(points={{60,60},{60,-60}},color={0,0,0}),
            Line(points={{80,60},{80,-60}},color={0,0,0}),
            Line(points={{-80,60},{-80,-60}},color={0,0,0}),
            Text(
              extent={{-20,4},{16,-4}},
              textColor={0,0,0},
              fontSize=20,
              textString="Tube Wall"),
            Line(
              points={{30,40},{-30,40}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Line(
              points={{-26,-38},{34,-38}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Text(
              extent={{-22,32},{14,24}},
              textColor={0,0,0},
              fontSize=20,
              textString="Water side"),
            Text(
              extent={{-30,-24},{26,-32}},
              textColor={0,0,0},
              fontSize=20,
              textString="Flue gas side")}),                         Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MonoPhasicHX_v2;

    model MonoPhasicHX_LCM
     import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      import MetroscopeModelingLibrary.Utilities.Constants;
      import CorrelationConstants =
             MetroscopeModelingLibrary.DynamicComponents.Correlations;

      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      // Constants
        parameter Real pi = Constants.pi;

      // ------ Geometry ------
        // Pipes
        parameter Units.Length D_out = 0.03 "Pipe outer diameter";
        parameter Units.Length e = 0.003 "Pipe wall thickness";
        parameter Units.Length D_in = D_out - 2*e "Pipe inner diameter";
        parameter Units.Length L = 22 "Tube's length";
        parameter Integer N_tubes_row = 184 "Number of tubes per row";
        parameter Integer Rows = 2 "Number of tubes rows";
        parameter Integer N_tubes = N_tubes_row*Rows "Total number of tubes";
        parameter Modelica.Units.SI.ThermalConductivity K_cond_wall = 27 "Wall thermal conductivity";
        parameter Integer Tubes_Config = 2 "1: aligned, 2: staggered";
        parameter Units.Length fg_path_width = 14.07 "Flue gas path transverse width";
        parameter Units.Length S_T = 76.25e-3 "Transverse pitch";
        parameter Units.Length S_L = 95.25e-3 "Longitudinal pitch";
        parameter Units.Length S_D = (S_L^2 + (S_T/2)^2)^0.5 "Diagonal pitch";
        // Water side
        parameter Units.Area A_water = N_tubes*L*pi*D_in "Water side heat exchange surface";
        parameter Units.Area Ac_water = 0.25*pi*D_in^2*N_tubes_row "Water side cross sectional area";
        // Flue gases
        parameter Units.Area A_fg = 2800 "Flue gase side heat exchange surface";
        parameter Units.Area Af_fg = L*fg_path_width "Flue gas frontal area";
        Units.HeatExchangeCoefficient K_conv_fg "Flue gase convection heat transfer coefficient"; //  = 82.06
        // Wall
        parameter Units.Mass M_wall = 25379 "Tubes total mass";
        parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";

      // ------ Initialization ------
        parameter Units.Temperature T_wall_0 = 450;
        parameter Units.Pressure P_water_0 = 70e5;
        parameter Units.Pressure P_fg_0 = 1e5;
        parameter Units.PositiveMassFlowRate Q_water_0 = 85;
        parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
        parameter Units.Temperature T_water_out_0 = 500;
        parameter Units.Temperature T_fg_out_0 = 560;
        parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
        parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;
        parameter Units.HeatExchangeCoefficient K_conv_water_0 = 2400;

      // ------ Discretization ------
        parameter Integer N = 10;
        parameter Units.Mass dM_wall = M_wall/N "Tube mass of a single node";
        parameter Units.Area dA_water = A_water/N "Water side heat exchange surface of a single node";
        parameter Units.Area dA_fg = A_fg/N "Flue gas side heat exchange surface of a single node";
        parameter Units.Length dx = L/N;

      // ------ Fluids properties ------
        // State
        WaterSteamMedium.ThermodynamicState state_water[N+1] "Water side node boundary state";
        FlueGasesMedium.ThermodynamicState state_fg[N+1] "Flue gas side node boundary states";
        // Enthalpy
        Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
        Units.SpecificEnthalpy h_fg[N+1] "Flue gas specific enthalpy";
        // Mass flow rate
        Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
        Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
        // Pressure
        Units.Pressure P_water(start=P_water_0) "Water Pressure";
        Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
        // Density
        Units.Density rho_water[N+1] "Node boundary water density";
        Units.Density rho_water_node[N] "Node average water density";
        Units.Density rho_fg[N+1] "Node boundary flue gas density";
        Units.Density rho_fg_node[N] "Node average flue gas density";
        // Mass fraction
        Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
        Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
        // Temperature
        Units.Temperature T_water[N+1] "Node boundary water temperature";
        Units.Temperature T_water_node[N] "Node average water temperature";
        Units.Temperature T_fg[N+1] "Node boundary flue gas temperature";
        Units.Temperature T_fg_node[N] "Node average flue gas temperature";
        // Dynamic viscosities
        Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_fg[N+1] "Node boundary flue gas dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_fg_node[N] "Node average flue gas dynamic viscosity";
        // Heat capacities Cp
        Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
        Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
        Units.HeatCapacity Cp_fg[N+1] "Node boundary flue gas Cp";
        Units.HeatCapacity Cp_fg_node[N] "Node average flue gas Cp";
        // Thermal conductivity
        Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_fg[N+1] "Node boundary flue gas thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_fg_node[N] "Node average flue gas thermal conductivity";

      // ------ Conduction variables ------
        Units.Temperature T_wall[N] "Node wall average temperature";

      // ------ Tubes configuration parameters ------
        Units.Velocity U_fg_face "Flue gas face velocity";
        Units.Velocity U_fg_max "Flue gas maximum velocity";
        Real Re_fg_max "Flue gas maximum Reynold's number";
        Real Pr_fg "Flue gas overall average Prandtl number";
        FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
        Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
        Real Nu_fg_avg "Flue gass overall average Nusselt number";

      // ------ Heat transfer parameters ------
        // Average velocities
        Units.Velocity U_water_node[N] "Node average water velocity";
        // Prandtl Number
        Real Pr_water[N] "Node water Prandtl's number";
        // Reynold's Number
        Real Re_water[N] "Node water Reynold's number";
        // Nusselt Nymber
        Real Nu_water[N] "Node water Nusselt number";
        // Convection heat transfer coefficient
        Units.HeatExchangeCoefficient K_conv_water[N](each start=K_conv_water_0) "Water side convection heat transfer coefficient";
        // Discretized heat transfer power
        Units.Power dW_water[N] "Node water heat exchange";
        Units.Power dW_fg[N] "Node flue gas heat exchange";
        // Biot Number
        Real Bi[N] "Node Biot number";

      // Parameters of interest
        Units.Temperature T_water_in "Water inlet temperature";
        Units.Temperature T_water_out "Water outlet temperature";
        Units.Temperature T_fg_in "Flue gas inlet temperature";
        Units.Temperature T_fg_out "Flue gas outlet temperature";
        Units.Temperature T_water_avg "Water overall average temperature";
        Units.Temperature T_fg_avg "Flue gas overall average temperature";
        Units.Temperature T_wall_avg "Wall overall average temperature";
        Units.Velocity U_water_avg "Water overall average velocity";
        Real Pr_water_avg "Water overall average Prandtl number";
        Real Re_water_avg "Water overall average Reynold's number";
        Real Nu_water_avg "Water overall average Nusselt number";
        Real K_conv_water_avg "Water side overall average convection heat transfer coefficient";
        Real Bi_max "Maximum Biot number";

      WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{90,30},{110,50}}),iconTransformation(extent={{90,30},{110,50}})));
      WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-110,32},{-90,52}}),iconTransformation(extent={{-110,32},{-90,52}})));
      FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{88,-50},{108,-30}}),iconTransformation(extent={{88,-50},{108,-30}})));
      FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-50},{-90,-30}}),iconTransformation(extent={{-110,-50},{-90,-30}})));
      WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,10},{-10,30}})));
      FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
    equation

      // ------ Boundaries ------
        // Enthalpy
        h_water[1] = water_side.h_in;
        h_water[N+1] = water_side.h_out;
        h_fg[1] = fg_side.h_out;
        h_fg[N+1] = fg_side.h_in;
        // Pressure
        P_water = water_side.P_in;
        P_fg = fg_side.P_in;
        // Mass flow rate
        Q_water = water_side.Q;
        Q_fg = fg_side.Q;
        // Mass Fraction
        Xi_water = water_side.Xi;
        Xi_fg = fg_side.Xi;

      // ------ Tubes configuration parameters ------
        // Flue gas face velocity
        U_fg_face = Q_fg/(rho_fg[N+1]*Af_fg);
        // Flue gas maximum velocity
        if (Tubes_Config == 1) then
          U_fg_max = S_T*U_fg_face/(S_T - D_out);
        elseif (S_D < (S_T + D_out)/2) then
          U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
        else
          U_fg_max = S_T*U_fg_face/(S_T - D_out);
        end if;
        // Flue gas maximum Reynold's number
        Re_fg_max = rho_fg[N+1]*U_fg_max*D_out/Mu_fg[N+1];
        // Flue gas Prandtl number
        Pr_fg = Cp_fg[N+1]*Mu_fg[N+1]/k_fg[N+1];
        // Flue gas Prandtl number at surface temperature
        state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
        Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
        // Convection coefficient calculation
        Nu_fg_avg = K_conv_fg*D_out/k_fg[N+1];
        Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);

      // ------ First node properties ------
        state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
        state_fg[1] = FlueGasesMedium.setState_phX(P_fg, h_fg[1], Xi_fg);
        T_water[1] = WaterSteamMedium.temperature(state_water[1]);
        T_fg[1] = FlueGasesMedium.temperature(state_fg[1]);
        rho_water[1] = WaterSteamMedium.density(state_water[1]);
        rho_fg[1] = FlueGasesMedium.density(state_fg[1]);
        Mu_water[1] = WaterSteamMedium.dynamicViscosity(state_water[1]);
        Mu_fg[1] = FlueGasesMedium.dynamicViscosity(state_fg[1]);
        Cp_water[1] = WaterSteamMedium.specificHeatCapacityCp(state_water[1]);
        Cp_fg[1] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1]);
        k_water[1] = WaterSteamMedium.thermalConductivity(state_water[1]);
        k_fg[1] = FlueGasesMedium.thermalConductivity(state_fg[1]);

      // ------ Parameters of interest ------
        // IN/OUT temperatures
        T_water_in = water_side.T_in;
        T_water_out = water_side.T_out;
        T_fg_in = fg_side.T_in;
        T_fg_out = fg_side.T_out;
        // Average Temperatures
        T_water_avg = sum(T_water_node)/N;
        T_fg_avg = sum(T_fg_node)/N;
        T_wall_avg =  sum(T_wall)/N;
        // Average velocity
        U_water_avg = sum(U_water_node)/N;
        // Average heat transfer parameters
        Pr_water_avg = sum(Pr_water)/N;
        Re_water_avg = sum(Re_water)/N;
        Nu_water_avg = sum(Nu_water)/N;
        K_conv_water_avg = sum(K_conv_water)/N;
        // Biot number and validity of LCM verification
        Bi_max = max(Bi);
        assert(Bi_max < 0.1, "LCM is not valid",  AssertionLevel.warning);

      // ------ Discretization computation loop ------
        for i in 1:N loop

          // Fluids Properties
            // State
            state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
            state_fg[i+1] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1], Xi_fg);
            // Temperature
            T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
            T_fg[i+1] = FlueGasesMedium.temperature(state_fg[i+1]);
            T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
            T_fg_node[i] = 0.5*(T_fg[i] + T_fg[i+1]);
            // Density
            rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
            rho_fg[i+1] = FlueGasesMedium.density(state_fg[i+1]);
            rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
            rho_fg_node[i] = 0.5*(rho_fg[i] + rho_fg[i+1]);
            // Dynamic viscosity
            Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
            Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
            Mu_fg[i+1] = FlueGasesMedium.dynamicViscosity(state_fg[i+1]);
            Mu_fg_node[i] = 0.5*(Mu_fg[i] + Mu_fg[i+1]);
            // Specific heat capacities Cp
            Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
            Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
            Cp_fg[i+1] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1]);
            Cp_fg_node[i] = 0.5*(Cp_fg[i] + Cp_fg[i+1]);
            // Thermal conductivity
            k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
            k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);
            k_fg[i+1] = FlueGasesMedium.thermalConductivity(state_fg[i+1]);
            k_fg_node[i] = 0.5*(k_fg[i] + k_fg[i+1]);

          // Conduction heat transfer
    //         dW_water[i] = 2*pi*K_cond_wall*dx*L*(T_wall[i] - T_wall_water[i])/(Modelica.Math.log(1 + e/D_in));
    //         dW_fg[i] = 2*pi*K_cond_wall*dx*L*(T_wall[i] - T_wall_fg[i])/(Modelica.Math.log(1 + e/(e + D_in)));

          // Node energy balance
            // Water side
            dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
            // Flue gas side
            dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);
            // Global with wall storage
            dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

          // Convection heat transfer coefficient calculation
            // Average velocities
            U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
            // Prandtl number
            Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
            // Reynold's number
            Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
            // Nusselt number
            Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
            // Convection correlation: Dittus-Boelter equation
            Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

          // Convection heat transfer equations
            // Water side
            dW_water[i] = K_conv_water[i]*dA_water*(T_wall[i] - T_water_node[i]);
            // Flue gas side
            dW_fg[i] = K_conv_fg*dA_fg*(T_wall[i] - T_fg_node[i]);

          // Biot number
            Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

        end for;

    initial equation

      for i in 1:N loop
        der(T_wall[i]) = 0;
      end for;

    equation
      connect(water_side.C_in, water_inlet) annotation (Line(points={{10,20},{100,20},{100,40}},
                                                                                       color={28,108,200}));
      connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,20},{-100,20},{-100,42}},
                                                                                           color={28,108,200}));
      connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,-20},{100,-20},{100,-40},{98,-40}},
                                                                                     color={95,95,95}));
      connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,-20},{-100,-20},{-100,-40}},
                                                                                     color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,-10},{100,-60}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,10},{100,-10}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,60},{100,10}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-60,60},{-60,-60}},color={0,0,0}),
            Line(points={{-40,60},{-40,-60}},color={0,0,0}),
            Line(points={{40,60},{40,-60}},color={0,0,0}),
            Line(points={{60,60},{60,-60}},color={0,0,0}),
            Line(points={{80,60},{80,-60}},color={0,0,0}),
            Line(points={{-80,60},{-80,-60}},color={0,0,0}),
            Text(
              extent={{-20,4},{16,-4}},
              textColor={0,0,0},
              fontSize=20,
              textString="Tube Wall"),
            Line(
              points={{30,40},{-30,40}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Line(
              points={{-26,-38},{34,-38}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Text(
              extent={{-22,32},{14,24}},
              textColor={0,0,0},
              fontSize=20,
              textString="Water side"),
            Text(
              extent={{-30,-24},{26,-32}},
              textColor={0,0,0},
              fontSize=20,
              textString="Flue gas side")}),                         Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MonoPhasicHX_LCM;

    model MonoPhasicHX_LCM_1 "Added more info on fins and used ESCOA correlation"
     import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      import MetroscopeModelingLibrary.Utilities.Constants;
      import CorrelationConstants =
             MetroscopeModelingLibrary.DynamicComponents.Correlations;

      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      // Constants
        parameter Real pi = Constants.pi;

      // ------ Geometry ------
        // Pipes
        parameter Units.Length D_out = 0.03 "Pipe outer diameter";
        parameter Units.Length e = 0.003 "Pipe wall thickness";
        parameter Units.Length D_in = D_out - 2*e "Pipe inner diameter";
        parameter Units.Length L = 22 "Tube's length";
        parameter Integer N_tubes_row = 184 "Number of tubes per row";
        parameter Integer Rows = 2 "Number of tubes rows";
        parameter Integer N_tubes = N_tubes_row*Rows "Total number of tubes";
        parameter Modelica.Units.SI.ThermalConductivity K_cond_wall = 27 "Wall thermal conductivity";
        parameter Integer Tubes_Config = 2 "1: aligned, 2: staggered";
        parameter Units.Length fg_path_width = 14.07 "Flue gas path transverse width";
        parameter Units.Length S_T = 76.25e-3 "Transverse pitch";
        parameter Units.Length S_L = 95.25e-3 "Longitudinal pitch";
        parameter Units.Length S_D = (S_L^2 + (S_T/2)^2)^0.5 "Diagonal pitch";
        parameter Units.Length S_f = 0.009506 "Fins pitch";
        parameter Units.Length H_fin = 0.009525 "Fin height";
        parameter Units.Length e_fin = 0.0009906 "Fin thickness";
        parameter Real Fin_per_meter = 1/S_f "Number of fins per meter";
        parameter Units.Length D_fin = D_out + 2*H_fin "Fin outer diameter";
        parameter Real N_fins = N_tubes*L/S_f "Number of fins";
        parameter Real eff_fins = 0.7742 "Fins efficiency";
        // Water side
        parameter Units.Area A_water = N_tubes*L*pi*D_in "Water side heat exchange surface";
        parameter Units.Area Ac_water = 0.25*pi*D_in^2*N_tubes_row "Water side cross sectional area";
        // Flue gas side
        parameter Units.Area A_fin_cb = pi*D_out*e_fin*N_fins "Fins cross-sectional area at the base";
        parameter Units.Area A_fg_tubes = N_tubes*L*pi*D_out - A_fin_cb "Outer tubes surface";
        parameter Units.Area A_fg_fins = 0.25*pi*(D_fin^2 - D_out^2)*2*N_fins + pi*D_fin*e_fin*N_fins "Fins outer surface";
        parameter Units.Area Af_fg = L*fg_path_width "Flue gas frontal area";

        // Wall
        parameter Units.Mass M_wall = 25379 "Tubes total mass";
        parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";

      // ------ Initialization ------
        parameter Units.Temperature T_wall_0 = 450;
        parameter Units.Pressure P_water_0 = 70e5;
        parameter Units.Pressure P_fg_0 = 1e5;
        parameter Units.PositiveMassFlowRate Q_water_0 = 85;
        parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
        parameter Units.Temperature T_water_out_0 = 500;
        parameter Units.Temperature T_fg_out_0 = 560;
        parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
        parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;
        parameter Units.HeatExchangeCoefficient K_conv_water_0 = 2400;

      // ------ Discretization ------
        parameter Integer N = 10;
        parameter Units.Mass dM_wall = M_wall/N "Tube mass of a single node";
        parameter Units.Area dA_water = A_water/N "Water side heat exchange surface of a single node";
        parameter Units.Area dA_fg_tubes = A_fg_tubes/N "Flue gas side heat exchange surface of a single node";
        parameter Units.Area dA_fin_cb = A_fin_cb/N;
        parameter Units.Area dA_fg_fin = A_fg_fins/N;
        parameter Units.Length dx = L/N;

      // ------ Fluids properties ------
        // State
        WaterSteamMedium.ThermodynamicState state_water[N+1] "Water side node boundary state";
        FlueGasesMedium.ThermodynamicState state_fg[N+1] "Flue gas side node boundary states";
        // Enthalpy
        Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
        Units.SpecificEnthalpy h_fg[N+1] "Flue gas specific enthalpy";
        // Mass flow rate
        Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
        Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
        // Pressure
        Units.Pressure P_water(start=P_water_0) "Water Pressure";
        Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
        // Density
        Units.Density rho_water[N+1] "Node boundary water density";
        Units.Density rho_water_node[N] "Node average water density";
        Units.Density rho_fg[N+1] "Node boundary flue gas density";
        Units.Density rho_fg_node[N] "Node average flue gas density";
        // Mass fraction
        Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
        Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
        // Temperature
        Units.Temperature T_water[N+1] "Node boundary water temperature";
        Units.Temperature T_water_node[N] "Node average water temperature";
        Units.Temperature T_fg[N+1] "Node boundary flue gas temperature";
        Units.Temperature T_fg_node[N] "Node average flue gas temperature";
        // Dynamic viscosities
        Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_fg[N+1] "Node boundary flue gas dynamic viscosity";
        Modelica.Units.SI.DynamicViscosity Mu_fg_node[N] "Node average flue gas dynamic viscosity";
        // Heat capacities Cp
        Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
        Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
        Units.HeatCapacity Cp_fg[N+1] "Node boundary flue gas Cp";
        Units.HeatCapacity Cp_fg_node[N] "Node average flue gas Cp";
        // Thermal conductivity
        Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_fg[N+1] "Node boundary flue gas thermal conductivity";
        Modelica.Units.SI.ThermalConductivity k_fg_node[N] "Node average flue gas thermal conductivity";

      // ------ Conduction variables ------
        //Units.Temperature T_wall_water[N] "Wall temperature from the water side";
        Units.Temperature T_wall[N] "Node wall average temperature";
        //Units.Temperature T_wall_fg[N] "Wall temperature from the flue gas side";

      // ------ Tubes configuration parameters ------
        Units.Velocity U_fg_face "Flue gas face velocity";
        Units.Velocity U_fg_max "Flue gas maximum velocity";
        Real Re_fg_max "Flue gas maximum Reynold's number";
        Real Pr_fg "Flue gas overall average Prandtl number";
        FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
        Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
        Real Nu_fg_avg "Flue gass overall average Nusselt number";
        Units.HeatExchangeCoefficient K_conv_fg "Flue gase convection heat transfer coefficient"; //  = 82.06

      // ------ Heat transfer parameters ------
        // Average velocities
        Units.Velocity U_water_node[N] "Node average water velocity";
        // Prandtl Number
        Real Pr_water[N] "Node water Prandtl's number";
        // Reynold's Number
        Real Re_water[N] "Node water Reynold's number";
        // Nusselt Nymber
        Real Nu_water[N] "Node water Nusselt number";
        // Convection heat transfer coefficient
        Units.HeatExchangeCoefficient K_conv_water[N](each start=K_conv_water_0) "Water side convection heat transfer coefficient";
        // Discretized heat transfer power
        Units.Power dW_water[N] "Node water heat exchange";
        Units.Power dW_fg[N] "Node flue gas heat exchange";
        // Biot Number
        Real Bi[N] "Node Biot number";

      // Parameters of interest
        Units.Temperature T_water_in "Water inlet temperature";
        Units.Temperature T_water_out "Water outlet temperature";
        Units.Temperature T_fg_in "Flue gas inlet temperature";
        Units.Temperature T_fg_out "Flue gas outlet temperature";
        Units.Temperature T_water_avg "Water overall average temperature";
        Units.Temperature T_fg_avg "Flue gas overall average temperature";
        Units.Temperature T_wall_avg "Wall overall average temperature";
        Units.Velocity U_water_avg "Water overall average velocity";
        Real Pr_water_avg "Water overall average Prandtl number";
        Real Re_water_avg "Water overall average Reynold's number";
        Real Nu_water_avg "Water overall average Nusselt number";
        Real K_conv_water_avg "Water side overall average convection heat transfer coefficient";
        Real Bi_max "Maximum Biot number";

      WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{90,30},{110,50}}),iconTransformation(extent={{90,30},{110,50}})));
      WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-110,32},{-90,52}}),iconTransformation(extent={{-110,32},{-90,52}})));
      FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{88,-50},{108,-30}}),iconTransformation(extent={{88,-50},{108,-30}})));
      FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-50},{-90,-30}}),iconTransformation(extent={{-110,-50},{-90,-30}})));
      WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,10},{-10,30}})));
      FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
    equation

      // ------ Boundaries ------
        // Enthalpy
        h_water[1] = water_side.h_in;
        h_water[N+1] = water_side.h_out;
        h_fg[1] = fg_side.h_out;
        h_fg[N+1] = fg_side.h_in;
        // Pressure
        P_water = water_side.P_in;
        P_fg = fg_side.P_in;
        // Mass flow rate
        Q_water = water_side.Q;
        Q_fg = fg_side.Q;
        // Mass Fraction
        Xi_water = water_side.Xi;
        Xi_fg = fg_side.Xi;

      // ------ Tubes configuration parameters ------
        // Flue gas face velocity
        U_fg_face = Q_fg/(rho_fg[N+1]*Af_fg);
        // Flue gas maximum velocity
        if (Tubes_Config == 1) then
          U_fg_max = S_T*U_fg_face/(S_T - D_out);
        elseif (S_D < (S_T + D_out)/2) then
          U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
        else
          U_fg_max = S_T*U_fg_face/(S_T - D_out);
        end if;
        // Flue gas maximum Reynold's number
        Re_fg_max = rho_fg[N+1]*U_fg_max*D_out/Mu_fg[N+1];
        // Flue gas Prandtl number
        Pr_fg = Cp_fg[N+1]*Mu_fg[N+1]/k_fg[N+1];
        // Flue gas Prandtl number at surface temperature
        state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
        Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
        // Convection coefficient calculation
        Nu_fg_avg = K_conv_fg*D_out/k_fg[N+1];
        // Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);
        Nu_fg_avg = CorrelationConstants.ESCOA(Re_fg_max, Pr_fg, Rows, T_fg_avg, T_wall_avg, D_out, H_fin, e_fin, S_f, S_T, S_L);

      // ------ First node properties ------
        state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
        state_fg[1] = FlueGasesMedium.setState_phX(P_fg, h_fg[1], Xi_fg);
        T_water[1] = WaterSteamMedium.temperature(state_water[1]);
        T_fg[1] = FlueGasesMedium.temperature(state_fg[1]);
        rho_water[1] = WaterSteamMedium.density(state_water[1]);
        rho_fg[1] = FlueGasesMedium.density(state_fg[1]);
        Mu_water[1] = WaterSteamMedium.dynamicViscosity(state_water[1]);
        Mu_fg[1] = FlueGasesMedium.dynamicViscosity(state_fg[1]);
        Cp_water[1] = WaterSteamMedium.specificHeatCapacityCp(state_water[1]);
        Cp_fg[1] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1]);
        k_water[1] = WaterSteamMedium.thermalConductivity(state_water[1]);
        k_fg[1] = FlueGasesMedium.thermalConductivity(state_fg[1]);

      // ------ Parameters of interest ------
        // IN/OUT temperatures
        T_water_in = water_side.T_in;
        T_water_out = water_side.T_out;
        T_fg_in = fg_side.T_in;
        T_fg_out = fg_side.T_out;
        // Average Temperatures
        T_water_avg = sum(T_water_node)/N;
        T_fg_avg = sum(T_fg_node)/N;
        T_wall_avg =  sum(T_wall)/N;
        // Average velocity
        U_water_avg = sum(U_water_node)/N;
        // Average heat transfer parameters
        Pr_water_avg = sum(Pr_water)/N;
        Re_water_avg = sum(Re_water)/N;
        Nu_water_avg = sum(Nu_water)/N;
        K_conv_water_avg = sum(K_conv_water)/N;
        // Biot number and validity of LCM verification
        Bi_max = max(Bi);
        assert(Bi_max < 0.1, "LCM is not valid",  AssertionLevel.warning);

      // ------ Discretization computation loop ------
        for i in 1:N loop

          // Fluids Properties
            // State
            state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
            state_fg[i+1] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1], Xi_fg);
            // Temperature
            T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
            T_fg[i+1] = FlueGasesMedium.temperature(state_fg[i+1]);
            T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
            T_fg_node[i] = 0.5*(T_fg[i] + T_fg[i+1]);
            // Density
            rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
            rho_fg[i+1] = FlueGasesMedium.density(state_fg[i+1]);
            rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
            rho_fg_node[i] = 0.5*(rho_fg[i] + rho_fg[i+1]);
            // Dynamic viscosity
            Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
            Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
            Mu_fg[i+1] = FlueGasesMedium.dynamicViscosity(state_fg[i+1]);
            Mu_fg_node[i] = 0.5*(Mu_fg[i] + Mu_fg[i+1]);
            // Specific heat capacities Cp
            Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
            Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
            Cp_fg[i+1] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1]);
            Cp_fg_node[i] = 0.5*(Cp_fg[i] + Cp_fg[i+1]);
            // Thermal conductivity
            k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
            k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);
            k_fg[i+1] = FlueGasesMedium.thermalConductivity(state_fg[i+1]);
            k_fg_node[i] = 0.5*(k_fg[i] + k_fg[i+1]);

          // Node energy balance
            // Water side
            dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
            // Flue gas side
            dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);
            // Global with wall storage
            dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

          // Convection heat transfer coefficient calculation
            // Average velocities
            U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
            // Prandtl number
            Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
            // Reynold's number
            Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
            // Nusselt number
            Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
            // Convection correlation: Dittus-Boelter equation
            Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

          // Convection heat transfer equations
            // Water side
            dW_water[i] = K_conv_water[i]*dA_water*(T_wall[i] - T_water_node[i]);
            // Flue gas side
            dW_fg[i] = K_conv_fg*(T_wall[i] - T_fg_node[i])*(dA_fg_tubes + eff_fins*dA_fg_fin);

          // Biot number
            Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

        end for;

    initial equation

      for i in 1:N loop
        der(T_wall[i]) = 0;
      end for;

    equation
      connect(water_side.C_in, water_inlet) annotation (Line(points={{10,20},{100,20},{100,40}},
                                                                                       color={28,108,200}));
      connect(water_side.C_out, water_outlet) annotation (Line(points={{-10,20},{-100,20},{-100,42}},
                                                                                           color={28,108,200}));
      connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,-20},{100,-20},{100,-40},{98,-40}},
                                                                                     color={95,95,95}));
      connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,-20},{-100,-20},{-100,-40}},
                                                                                     color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
              extent={{-100,-10},{100,-60}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,10},{100,-10}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-100,60},{100,10}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(points={{-60,60},{-60,-60}},color={0,0,0}),
            Line(points={{-40,60},{-40,-60}},color={0,0,0}),
            Line(points={{40,60},{40,-60}},color={0,0,0}),
            Line(points={{60,60},{60,-60}},color={0,0,0}),
            Line(points={{80,60},{80,-60}},color={0,0,0}),
            Line(points={{-80,60},{-80,-60}},color={0,0,0}),
            Text(
              extent={{-20,4},{16,-4}},
              textColor={0,0,0},
              fontSize=20,
              textString="Tube Wall"),
            Line(
              points={{30,40},{-30,40}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Line(
              points={{-26,-38},{34,-38}},
              color={0,0,0},
              arrow={Arrow.None,Arrow.Filled},
              thickness=0.5),
            Text(
              extent={{-22,32},{14,24}},
              textColor={0,0,0},
              fontSize=20,
              textString="Water side"),
            Text(
              extent={{-30,-24},{26,-32}},
              textColor={0,0,0},
              fontSize=20,
              textString="Flue gas side")}),                         Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MonoPhasicHX_LCM_1;
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

    model MonoPhasicHX_node_InitEquations_test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_nodes_InitEquations
                                        monoPhasicHX_nodes_InitEquations(
                                                           N=10,
        D_out=0.0381,
        e=0.003048,
        L=18.29,
        N_tubes=2*184,
        A_water=676.73035,
        T_wall_0=745.15)                                                          annotation (Placement(transformation(extent={{-10,-6},{10,16}})));

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

      connect(monoPhasicHX_nodes_InitEquations.water_inlet, cold_source.C_out) annotation (Line(points={{10,9.4},{10,8},{48,8},{48,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_nodes_InitEquations.water_outlet, cold_sink.C_in) annotation (Line(points={{-10,9.62},{-10,8},{-48,8},{-48,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_nodes_InitEquations.fg_inlet, hot_source.C_out) annotation (Line(points={{-10,0.6},{-50,0.6},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_nodes_InitEquations.fg_outlet, hot_sink.C_in) annotation (Line(points={{9.8,0.6},{50,0.6},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_node_InitEquations_test;

    model MonoPhasicHX_ConvHTC_WaterCalc_test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_ConvHTC_WaterCalc
                                        monoPhasicHX_ConvHTC_WaterCalc(
        N=10,
        D_out=0.0381,
        e=0.003048,
        L=18.29,
        A_water=676.73035,
        T_wall_0=745.15)                                                          annotation (Placement(transformation(extent={{-10,-6},{10,16}})));

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
      Modelica.Blocks.Sources.Step step(height=10,  startTime=100) annotation (Placement(transformation(extent={{-72,66},{-52,86}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=-20,
        duration=60,
        startTime=300) annotation (Placement(transformation(extent={{-6,66},{14,86}})));
    equation
      hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
      hot_source.P_out = P_hot_source*1e5;
      hot_source.T_out = T_hot_source + 273.15 + ramp.y;
      hot_source.Q_out = - Q_hot_source;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source + step.y;

      connect(monoPhasicHX_ConvHTC_WaterCalc.water_inlet, cold_source.C_out) annotation (Line(points={{10,9.4},{10,8},{48,8},{48,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_ConvHTC_WaterCalc.water_outlet, cold_sink.C_in) annotation (Line(points={{-10,9.62},{-10,8},{-48,8},{-48,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_ConvHTC_WaterCalc.fg_inlet, hot_source.C_out) annotation (Line(points={{-10,0.6},{-50,0.6},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_ConvHTC_WaterCalc.fg_outlet, hot_sink.C_in) annotation (Line(points={{9.8,0.6},{50,0.6},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_ConvHTC_WaterCalc_test;

    model MonoPhasicHX_ConvHTC_FGCalc_test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_ConvHTC_FGCalc
                                        monoPhasicHX_ConvHTC_FGCalc(
        N_tubes_row=184,
        Rows=2,
        Tubes_Config=2,
        fg_path_width=14.07,                               N=10,
        D_out=0.0381,
        e=0.003048,
        L=18.29,
        A_water=676.73035,
        T_wall_0=745.15)                                                          annotation (Placement(transformation(extent={{-10,-6},{10,16}})));

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
      //hot_source.Xi_out = {0.78,0.22,0,0,0};
      hot_source.P_out = P_hot_source*1e5;
      hot_source.T_out = T_hot_source + 273.15 + ramp.y;
      hot_source.Q_out = - Q_hot_source + step.y;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source;

      connect(monoPhasicHX_ConvHTC_FGCalc.water_inlet, cold_source.C_out) annotation (Line(points={{10,9.4},{10,8},{48,8},{48,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_ConvHTC_FGCalc.water_outlet, cold_sink.C_in) annotation (Line(points={{-10,9.62},{-10,8},{-48,8},{-48,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_ConvHTC_FGCalc.fg_inlet, hot_source.C_out) annotation (Line(points={{-10,0.6},{-50,0.6},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_ConvHTC_FGCalc.fg_outlet, hot_sink.C_in) annotation (Line(points={{9.8,0.6},{50,0.6},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_ConvHTC_FGCalc_test;

    model MonoPhasicHX_SH_WaterHTC_Bi_test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_WaterHTC_Bi
                                        monoPhasicHX_WaterHTC_Bi(
                                                           N=10,
        D_out=0.0381,
        e=0.003048,
        L=18.29,
        A_water=676.73035,
        T_wall_0=745.15)                                                          annotation (Placement(transformation(extent={{-10,-6},{10,16}})));

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
      Modelica.Blocks.Sources.Step step(height=10,  startTime=100) annotation (Placement(transformation(extent={{-72,66},{-52,86}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=-20,
        duration=60,
        startTime=300) annotation (Placement(transformation(extent={{-6,66},{14,86}})));
    equation
      hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
      hot_source.P_out = P_hot_source*1e5;
      hot_source.T_out = T_hot_source + 273.15 + ramp.y;
      hot_source.Q_out = - Q_hot_source;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source + step.y;

      connect(monoPhasicHX_WaterHTC_Bi.water_inlet, cold_source.C_out) annotation (Line(points={{10,9.4},{10,8},{48,8},{48,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_WaterHTC_Bi.water_outlet, cold_sink.C_in) annotation (Line(points={{-10,9.62},{-10,8},{-48,8},{-48,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_WaterHTC_Bi.fg_inlet, hot_source.C_out) annotation (Line(points={{-10,0.6},{-50,0.6},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_WaterHTC_Bi.fg_outlet, hot_sink.C_in) annotation (Line(points={{9.8,0.6},{50,0.6},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_SH_WaterHTC_Bi_test;

    model MonoPhasicHX_ECO_WaterHTC_Bi_test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_WaterHTC_Bi
                                        monoPhasicHX_WaterHTC_Bi(
                                                           N=10,
        D_out=0.0381,
        e=0.003048,
        L=18.29,
        A_water=676.73035,
        T_wall_0=745.15)                                                          annotation (Placement(transformation(extent={{-10,-6},{10,16}})));

        // Boundary conditions
      input Real P_hot_source(start = 1.1, min = 0, nominal = 1) "barA";
      input Real Q_hot_source(start = 658.695) "kg/s";
      input Utilities.Units.Temperature T_hot_source(start = 337.1) "degC";

      input Real P_cold_source(start = 124.8, min = 1.5, nominal = 100) "barA";
      input Utilities.Units.MassFlowRate Q_cold_source(start = 83.481) "kg/s";
      input Real T_cold_source(start = 498.8, min = 130, nominal = 290) "degC";

      WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={64,20})));
      WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{-54,10},{-74,30}})));
      FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-76,-50},{-56,-30}})));
      FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{54,-50},{74,-30}})));
      Modelica.Blocks.Sources.Step step(height=10,  startTime=100) annotation (Placement(transformation(extent={{-72,66},{-52,86}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=-20,
        duration=60,
        startTime=300) annotation (Placement(transformation(extent={{-6,66},{14,86}})));
    equation
      hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
      hot_source.P_out = P_hot_source*1e5;
      hot_source.T_out = T_hot_source + 273.15 + ramp.y;
      hot_source.Q_out = - Q_hot_source;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source + step.y;

      connect(monoPhasicHX_WaterHTC_Bi.water_inlet, cold_source.C_out) annotation (Line(points={{10,9.4},{10,8},{48,8},{48,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_WaterHTC_Bi.water_outlet, cold_sink.C_in) annotation (Line(points={{-10,9.62},{-10,8},{-48,8},{-48,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_WaterHTC_Bi.fg_inlet, hot_source.C_out) annotation (Line(points={{-10,0.6},{-50,0.6},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_WaterHTC_Bi.fg_outlet, hot_sink.C_in) annotation (Line(points={{9.8,0.6},{50,0.6},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_ECO_WaterHTC_Bi_test;

    model MonoPhasicHX_ConvHTC_Conduction_test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_WaterHTC_Conduction
                                        monoPhasicHX_WaterHTC_Conduction(
                                                           N=10,
        D_out=0.0381,
        e=0.003048,
        L=18.29,
        A_water=676.73035,
        T_wall_0=745.15)                                                          annotation (Placement(transformation(extent={{-10,-6},{10,16}})));

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
    equation
      hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
      hot_source.P_out = P_hot_source*1e5;
      hot_source.T_out = T_hot_source + 273.15;
      hot_source.Q_out = - Q_hot_source;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source;

      connect(monoPhasicHX_WaterHTC_Conduction.water_inlet, cold_source.C_out) annotation (Line(points={{10,9.4},{10,8},{48,8},{48,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_WaterHTC_Conduction.water_outlet, cold_sink.C_in) annotation (Line(points={{-10,9.62},{-10,8},{-48,8},{-48,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_WaterHTC_Conduction.fg_inlet, hot_source.C_out) annotation (Line(points={{-10,0.6},{-50,0.6},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_WaterHTC_Conduction.fg_outlet, hot_sink.C_in) annotation (Line(points={{9.8,0.6},{50,0.6},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_ConvHTC_Conduction_test;

    model MonoPhasicHX_v1_test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_v1    monoPhasicHX_v1_1(
        N_tubes_row=184,
        Rows=2,
        Tubes_Config=2,
        fg_path_width=14.07,                               N=10,
        D_out=0.0381,
        e=0.003048,
        L=18.29,
        A_water=676.73035,
        T_wall_0=745.15)                                                          annotation (Placement(transformation(extent={{-10,-6},{10,16}})));

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
      //hot_source.Xi_out = {0.78,0.22,0,0,0};
      hot_source.P_out = P_hot_source*1e5;
      hot_source.T_out = T_hot_source + 273.15 + ramp.y;
      hot_source.Q_out = - Q_hot_source + step.y;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source;

      connect(monoPhasicHX_v1_1.water_inlet, cold_source.C_out) annotation (Line(points={{10,9.4},{10,8},{48,8},{48,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_v1_1.water_outlet, cold_sink.C_in) annotation (Line(points={{-10,9.62},{-10,8},{-48,8},{-48,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_v1_1.fg_inlet, hot_source.C_out) annotation (Line(points={{-10,0.6},{-50,0.6},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_v1_1.fg_outlet, hot_sink.C_in) annotation (Line(points={{9.8,0.6},{50,0.6},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_v1_test;

    model MonoPhasicHX_v2_test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_v2    monoPhasicHX_v2_1(
        N_tubes_row=184,
        Rows=2,
        Tubes_Config=2,
        fg_path_width=14.07,
        A_fg=2866.2,                                       N=10,
        D_out=0.0381,
        e=0.003048,
        L=18.29,
        A_water=676.73035,
        T_wall_0=745.15)                                                          annotation (Placement(transformation(extent={{-10,-6},{10,16}})));

        // Boundary conditions
      input Real P_hot_source(start = 1, min = 0, nominal = 1) "barA";
      input Real Q_hot_source(start = 658.695) "kg/s";
      input Utilities.Units.Temperature T_hot_source(start = 633.7) "degC";

      input Real P_cold_source(start = 118.9, min = 1.5, nominal = 100) "barA";
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
      //hot_source.Xi_out = {0.78,0.22,0,0,0};
      hot_source.P_out = P_hot_source*1e5;
      hot_source.T_out = T_hot_source + 273.15 + ramp.y;
      hot_source.Q_out = - Q_hot_source + step.y;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source;

      connect(monoPhasicHX_v2_1.water_inlet, cold_source.C_out) annotation (Line(points={{10,9.4},{10,8},{48,8},{48,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_v2_1.water_outlet, cold_sink.C_in) annotation (Line(points={{-10,9.62},{-10,8},{-48,8},{-48,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_v2_1.fg_inlet, hot_source.C_out) annotation (Line(points={{-10,0.6},{-50,0.6},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_v2_1.fg_outlet, hot_sink.C_in) annotation (Line(points={{9.8,0.6},{50,0.6},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_v2_test;

    model ESCOA_test

      import MetroscopeModelingLibrary.Utilities.Units;

      // parameter arguments
        parameter Real Re_fg = 5466 "Flue gas Reynold's number";
        parameter Real Pr_fg = 0.7139 "Flue gas Prandtl number";
        parameter Integer Rows = 2 "Number of tubes in flow direction";
        parameter Units.Temperature T_fg = (633.7 + 614.7)/2 "Flue gas temperature";
        parameter Units.Temperature T_fin = 590 "Fin temperature";
        parameter Units.Length D_out = 38.1/1000 "Tube outer diameter";
        parameter Units.Length H_fin =  9.525/1000 "Fin height";
        parameter Units.Length e_fin = 0.9906/1000 "Fin thickness";
        parameter Units.Length S_fin = 1/105.2 "Fin pitch";
        parameter Units.Length S_T = 76.25/1000 "Tube bundle transverse pitch";
        parameter Units.Length S_L = 95.25/1000 "Tube bundle longitudanal pitch";

      // Function output
        // Nusselt number
        output Real Nu_fg;

    equation

      Nu_fg = Correlations.ESCOA(Re_fg, Pr_fg, Rows, T_fg, T_fin, D_out, H_fin, e_fin, S_fin, S_T, S_L);


      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end ESCOA_test;

    model ESCOA_func_test

        import MetroscopeModelingLibrary.Utilities.Units;

      // parameter arguments
        parameter Real Re_fg = 5466 "Flue gas Reynold's number";
        parameter Real Pr_fg = 0.7139 "Flue gas Prandtl number";
        parameter Integer Rows = 2 "Number of tubes in flow direction";
        parameter Units.Temperature T_fg = (633.7 + 614.7)/2 "Flue gas temperature";
        parameter Units.Temperature T_fin = 590 "Fin temperature";
        parameter Units.Length D_out = 38.1/1000 "Tube outer diameter";
        parameter Units.Length H_fin =  9.525/1000 "Fin height";
        parameter Units.Length e_fin = 0.9906/1000 "Fin thickness";
        parameter Units.Length S_fin = 1/105.2 "Fin pitch";
        parameter Units.Length S_T = 76.25/1000 "Tube bundle transverse pitch";
        parameter Units.Length S_L = 95.25/1000 "Tube bundle longitudanal pitch";


      // Function output
        // Nusselt number
        output Real Nu_fg;

      Real Constant_C1 "Constant used in ESCOA empirical relation";
      Real Constant_C3 "Constant used in ESCOA empirical relation";
      Real Constant_C5 "Constant used in ESCOA empirical relation";
      parameter Units.Length D_fin = D_out + 2*H_fin "Fin outer diameter";

    equation

      Nu_fg =  Constant_C1*Constant_C3*Constant_C5*Re_fg*Pr_fg^(1/3)*((T_fg + 273.15)/(T_fin + 273.15))^0.25*(D_fin/D_out)^0.5;

      // Traditional ESCOA Correlation
      Constant_C1 =  0.25*Re_fg^(-0.35);
      Constant_C3 =  0.55 + 0.45*exp(-0.35*H_fin/(S_fin - e_fin));
      Constant_C5 =  0.7 + (0.7 - 0.8*exp(-0.15*Rows^2))*exp(-S_L/S_T);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end ESCOA_func_test;

    model MonoPhasicHX_LCM
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_LCM   monoPhasicHX_LCM(
        N_tubes_row=184,
        Rows=2,
        Tubes_Config=2,
        fg_path_width=14.07,
        M_wall=17800,                                      N=10,
        D_out=0.0381,
        e=0.003048,
        L=18.29,
        A_water=676.73035,
        T_wall_0=745.15)                                                          annotation (Placement(transformation(extent={{-10,-6},{10,16}})));

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
      //hot_source.Xi_out = {0.78,0.22,0,0,0};
      hot_source.P_out = P_hot_source*1e5;
      hot_source.T_out = T_hot_source + 273.15 + ramp.y;
      hot_source.Q_out = - Q_hot_source + step.y;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source;

      connect(monoPhasicHX_LCM.water_inlet, cold_source.C_out) annotation (Line(points={{10,9.4},{10,8},{48,8},{48,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_LCM.water_outlet, cold_sink.C_in) annotation (Line(points={{-10,9.62},{-10,8},{-48,8},{-48,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_LCM.fg_inlet, hot_source.C_out) annotation (Line(points={{-10,0.6},{-50,0.6},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_LCM.fg_outlet, hot_sink.C_in) annotation (Line(points={{9.8,0.6},{50,0.6},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_LCM;

    model MonoPhasicHX_LCM_1_Test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.MonoPhasicHX_LCM_1 monoPhasicHX_LCM_1_1(
        N_tubes_row=184,
        Rows=2,
        Tubes_Config=2,
        fg_path_width=14.07,
        N=10,
        D_out=0.0381,
        e=0.003048,
        L=18.29,
        A_water=676.73035,
        T_wall_0=745.15)                                                          annotation (Placement(transformation(extent={{-10,-6},{10,16}})));

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
      //hot_source.Xi_out = {0.78,0.22,0,0,0};
      hot_source.P_out = P_hot_source*1e5;
      hot_source.T_out = T_hot_source + 273.15 + ramp.y;
      hot_source.Q_out = - Q_hot_source + step.y;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source;

      connect(monoPhasicHX_LCM_1_1.water_inlet, cold_source.C_out) annotation (Line(points={{10,9.4},{10,8},{48,8},{48,20},{59,20}}, color={28,108,200}));
      connect(monoPhasicHX_LCM_1_1.water_outlet, cold_sink.C_in) annotation (Line(points={{-10,9.62},{-10,8},{-48,8},{-48,20},{-59,20}}, color={28,108,200}));
      connect(monoPhasicHX_LCM_1_1.fg_inlet, hot_source.C_out) annotation (Line(points={{-10,0.6},{-50,0.6},{-50,-40},{-61,-40}}, color={95,95,95}));
      connect(monoPhasicHX_LCM_1_1.fg_outlet, hot_sink.C_in) annotation (Line(points={{9.8,0.6},{50,0.6},{50,-40},{59,-40}}, color={95,95,95}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end MonoPhasicHX_LCM_1_Test;
  end Tests;

  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),      Line(points={{-56,72}}, color={28,108,200}), Line(
          points={{-100,0},{-50,100},{50,-100},{100,0}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier)}));
end DynamicComponents;
