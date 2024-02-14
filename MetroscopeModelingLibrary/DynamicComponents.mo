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
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Text(
            extent={{100,100},{-100,-100}},
            textColor={0,0,0},
            fontName="Centaur",
            textString="f")}));
  end Correlations;

  package HeatExchangers

    package One_pass_HX
      model CounterCurrent_MonoPhasicHX_LCM "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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
          Units.Temperature T_wall[N] "Node wall average temperature";

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
      end CounterCurrent_MonoPhasicHX_LCM;

      model CounterCurrent_MonoPhasicHX_2NodesConduction "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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

            // Conduction heat transfer
              dW_water[i] = 2*pi*K_cond_wall*dx*N_tubes*(T_wall[i] - T_wall_water[i])/(Modelica.Math.log(1 + e/D_in));
              dW_fg[i] = 2*pi*K_cond_wall*dx*N_tubes*(T_wall[i] - T_wall_fg[i])/(Modelica.Math.log(1 + e/(e + D_in)));

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
              dW_fg[i] = K_conv_fg*(T_wall_fg[i] - T_fg_node[i])*(dA_fg_tubes + eff_fins*dA_fg_fin);

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
      end CounterCurrent_MonoPhasicHX_2NodesConduction;

      model CounterCurrent_MonoPhasicHX_1DConduction_plate "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_tubes = 17712 "Tubes mass";
          parameter Units.Mass M_fins = 7667 "Fins mass";
          parameter Units.Mass M_wall = M_tubes + M_fins "Tubes + fins total mass";
          parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";
          parameter Units.Density rho_wall = 7770 "Tube material density";
          parameter Modelica.Units.SI.ThermalDiffusivity a_wall = K_cond_wall/(rho_wall*Cp_wall);

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
          parameter Units.Mass dM_tubes = M_tubes/N;
          parameter Units.Mass dM_fins = M_fins/N;
          parameter Units.Mass dM_wall = M_wall/N "Tube mass of a single node";
          parameter Units.Area dA_water = A_water/N "Water side heat exchange surface of a single node";
          parameter Units.Area dA_fg_tubes = A_fg_tubes/N "Flue gas side heat exchange surface of a single node";
          parameter Units.Area dA_fin_cb = A_fin_cb/N;
          parameter Units.Area dA_fg_fin = A_fg_fins/N;
          parameter Units.Length dx = L/N;
          parameter Integer N_wall = 5;
          parameter Units.Length dy_wall = e/N_wall;

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
          Units.Temperature T_wall_y[N_wall+1, N] "Wall temperature distribution";

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
          T_wall_avg =  sum(T_wall_y[N_wall+1])/N;
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

            // 1-D Conduction
              // Conduction boundary conditions
                //- dW_water[i] + K_cond_wall*(T_wall_y[2,i] - T_wall_y[1,i])/dy_wall*pi*D_in*dx*N_tubes = dM_wall*Cp_wall/N_wall/2*der(T_wall_y[1,i]); // check sign

                - dW_water[i] + K_cond_wall*pi*dx*N_tubes*D_in*(T_wall_y[2,i] - T_wall_y[1,i])/dy_wall = dM_wall*Cp_wall/N_wall/2*der(T_wall_y[1,i]);
                - dW_fg[i] - K_cond_wall*pi*dx*N_tubes*D_out*(T_wall_y[N_wall+1,i] - T_wall_y[N_wall,i])/dy_wall = dM_wall*Cp_wall/N_wall/2*der(T_wall_y[N_wall+1,i]); // check sign

              // Conduction inner nodes
              for j in 2:N_wall loop
                K_cond_wall*pi*dx*N_tubes*((D_in+D_out)/2)*(T_wall_y[j-1,i] - T_wall_y[j,i])/dy_wall
                + K_cond_wall*pi*dx*N_tubes*((D_in+D_out)/2)*(T_wall_y[j+1,i] - T_wall_y[j,i])/dy_wall
                = dM_wall*Cp_wall/N_wall*der(T_wall_y[j,i]);
              end for;

            // Node energy balance
              // Water side
              dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
              // Flue gas side
              dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);
              // Global with wall storage
              //dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

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
              dW_water[i] = K_conv_water[i]*dA_water*(T_wall_y[1, i] - T_water_node[i]);
              // Flue gas side
              dW_fg[i] = K_conv_fg*(T_wall_y[N_wall+1,i] - T_fg_node[i])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
              Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;

      initial equation

        for i in 1:N loop
          //der(T_wall[i]) = 0;

          for j in 1:N_wall+1 loop
            der(T_wall_y[j,i]) = 0;
          end for;
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
      end CounterCurrent_MonoPhasicHX_1DConduction_plate;

      model CounterCurrent_MonoPhasicHX_1DConduction_radial "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
          parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";
          parameter Units.Density rho_wall = 7770 "Tube material density";
          parameter Modelica.Units.SI.ThermalDiffusivity a_wall = K_cond_wall/(rho_wall*Cp_wall);

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
          parameter Integer N_wall = 5;
          parameter Units.Length dy_wall = e/N_wall;

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
          Units.Temperature T_wall_y[N_wall+1, N] "Wall temperature distribution";

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
          T_wall_avg =  sum(T_wall_y[N_wall+1])/N;
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

            // 1-D Conduction
              // Conduction boundary conditions
                //- dW_water[i] + K_cond_wall*(T_wall_y[2,i] - T_wall_y[1,i])/dy_wall*pi*D_in*dx*N_tubes = dM_wall*Cp_wall/N_wall/2*der(T_wall_y[1,i]); // check sign

                - dW_water[i] + K_cond_wall*2*pi*dx*N_tubes*(T_wall_y[2,i] - T_wall_y[1,i])/(Modelica.Math.log((D_in + 2*dy_wall)/D_in)) = dM_wall*Cp_wall/N_wall/2*der(T_wall_y[1,i]);
                - dW_fg[i] - K_cond_wall*2*pi*dx*N_tubes*(T_wall_y[N_wall+1,i] - T_wall_y[N_wall,i])/(Modelica.Math.log(D_out/(D_out - 2*dy_wall))) = dM_wall*Cp_wall/N_wall/2*der(T_wall_y[N_wall+1,i]); // check sign

              // Conduction inner nodes
              for j in 2:N_wall loop
                K_cond_wall*2*pi*dx*N_tubes*(T_wall_y[j-1,i] - T_wall_y[j,i])/(Modelica.Math.log((D_in/2 + (j-1)*dy_wall)/(D_in/2 + (j-2)*dy_wall)))
                + K_cond_wall*2*pi*dx*N_tubes*(T_wall_y[j+1,i] - T_wall_y[j,i])/(Modelica.Math.log((D_in/2 + j*dy_wall)/(D_in/2 + (j-1)*dy_wall)))
                = dM_wall*Cp_wall/N_wall*der(T_wall_y[j,i]);
              end for;

            // Node energy balance
              // Water side
              dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
              // Flue gas side
              dW_fg[i] = Q_fg*(h_fg[i] - h_fg[i+1]);
              // Global with wall storage
              //dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

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
              dW_water[i] = K_conv_water[i]*dA_water*(T_wall_y[1, i] - T_water_node[i]);
              // Flue gas side
              dW_fg[i] = K_conv_fg*(T_wall_y[N_wall+1,i] - T_fg_node[i])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
              Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;

      initial equation

        for i in 1:N loop
          //der(T_wall[i]) = 0;

          for j in 1:N_wall+1 loop
            der(T_wall_y[j,i]) = 0;
          end for;
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
      end CounterCurrent_MonoPhasicHX_1DConduction_radial;

      model CrossCurrent_MonoPhasicHX_LCM_ConstantK "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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
          FlueGasesMedium.ThermodynamicState state_fg[2, N] "Flue gas side node boundary states";
          // Enthalpy
          Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
          Units.SpecificEnthalpy h_fg[2, N] "Flue gas specific enthalpy";
          // Mass flow rate
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Pressure
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Density
          Units.Density rho_water[N+1] "Node boundary water density";
          Units.Density rho_water_node[N] "Node average water density";
          Units.Density rho_fg[2, N] "Node boundary flue gas density";
          // Mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
          // Temperature
          Units.Temperature T_water[N+1] "Node boundary water temperature";
          Units.Temperature T_water_node[N] "Node average water temperature";
          Units.Temperature T_fg[2, N] "Node boundary flue gas temperature";
          // Dynamic viscosities
          Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_fg[2, N] "Node boundary flue gas dynamic viscosity";
          // Heat capacities Cp
          Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
          Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
          Units.HeatCapacity Cp_fg[2, N] "Node boundary flue gas Cp";
          // Thermal conductivity
          Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_fg[2, N] "Node boundary flue gas thermal conductivity";

        // ------ Conduction variables ------
          Units.Temperature T_wall[N] "Node wall average temperature";

        // ------ Tubes configuration parameters ------
      //     Units.Velocity U_fg_face "Flue gas face velocity";
      //     Units.Velocity U_fg_max "Flue gas maximum velocity";
      //     Real Re_fg_max "Flue gas maximum Reynold's number";
      //     Real Pr_fg "Flue gas overall average Prandtl number";
      //     FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
      //     Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
      //     Real Nu_fg_avg "Flue gass overall average Nusselt number";
          parameter Units.HeatExchangeCoefficient K_conv_fg = 76.83 "Flue gase convection heat transfer coefficient"; //  = 82.06

        // ------ Heat transfer parameters ------
      //     // Average velocities
      //     Units.Velocity U_water_node[N] "Node average water velocity";
      //     // Prandtl Number
      //     Real Pr_water[N] "Node water Prandtl's number";
      //     // Reynold's Number
      //     Real Re_water[N] "Node water Reynold's number";
      //     // Nusselt Nymber
      //     Real Nu_water[N] "Node water Nusselt number";
          // Convection heat transfer coefficient
          parameter Units.HeatExchangeCoefficient K_conv_water = 2418 "Water side convection heat transfer coefficient";
          // Discretized heat transfer power
          Units.Power dW_water[N] "Node water heat exchange";
          Units.Power dW_fg[N] "Node flue gas heat exchange";
          // Biot Number
          // Real Bi[N] "Node Biot number";

        // Parameters of interest
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";
          Units.Temperature T_water_avg "Water overall average temperature";
          //Units.Temperature T_fg_avg "Flue gas overall average temperature";
          Units.Temperature T_wall_avg "Wall overall average temperature";
      //     Units.Velocity U_water_avg "Water overall average velocity";
      //     Real Pr_water_avg "Water overall average Prandtl number";
      //     Real Re_water_avg "Water overall average Reynold's number";
      //     Real Nu_water_avg "Water overall average Nusselt number";
      //     Real K_conv_water_avg "Water side overall average convection heat transfer coefficient";
      //     Real Bi_max "Maximum Biot number";

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------
          // Enthalpy
          h_water[1] = water_side.h_in;
          h_water[N+1] = water_side.h_out;
          //sum(h_fg[2, N]) = fg_side.h_out;
          fg_side.W = sum(dW_fg);

          for i in 1:N loop
           h_fg[1, i] = fg_side.h_in;
           T_fg[1, i] = FlueGasesMedium.temperature(state_fg[1, i]);
           state_fg[1, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[1, i], Xi_fg);
           rho_fg[1, i] = FlueGasesMedium.density(state_fg[1, i]);
           Mu_fg[1, i] = FlueGasesMedium.dynamicViscosity(state_fg[1, i]);
           Cp_fg[1, i] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1, i]);
           k_fg[1, i] = FlueGasesMedium.thermalConductivity(state_fg[1, i]);
          end for;
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
      //     U_fg_face = Q_fg/(rho_fg[N+1]*Af_fg);
      //     // Flue gas maximum velocity
      //     if (Tubes_Config == 1) then
      //       U_fg_max = S_T*U_fg_face/(S_T - D_out);
      //     elseif (S_D < (S_T + D_out)/2) then
      //       U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
      //     else
      //       U_fg_max = S_T*U_fg_face/(S_T - D_out);
      //     end if;
      //     // Flue gas maximum Reynold's number
      //     Re_fg_max = rho_fg[N+1]*U_fg_max*D_out/Mu_fg[N+1];
      //     // Flue gas Prandtl number
      //     Pr_fg = Cp_fg[N+1]*Mu_fg[N+1]/k_fg[N+1];
      //     // Flue gas Prandtl number at surface temperature
      //     state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
      //     Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
      //     // Convection coefficient calculation
      //     Nu_fg_avg = K_conv_fg*D_out/k_fg[N+1];
      //     // Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);
      //     Nu_fg_avg = CorrelationConstants.ESCOA(Re_fg_max, Pr_fg, Rows, T_fg_avg, T_wall_avg, D_out, H_fin, e_fin, S_f, S_T, S_L);

        // ------ First node properties ------
          state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);

          T_water[1] = WaterSteamMedium.temperature(state_water[1]);
          rho_water[1] = WaterSteamMedium.density(state_water[1]);
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
          //T_fg_avg = sum(T_fg_node)/N;
          T_wall_avg =  sum(T_wall)/N;
          // Average velocity
      //     U_water_avg = sum(U_water_node)/N;
      //     // Average heat transfer parameters
      //     Pr_water_avg = sum(Pr_water)/N;
      //     Re_water_avg = sum(Re_water)/N;
      //     Nu_water_avg = sum(Nu_water)/N;
      //     K_conv_water_avg = sum(K_conv_water)/N;
      //     // Biot number and validity of LCM verification
      //     Bi_max = max(Bi);
      //     assert(Bi_max < 0.1, "LCM is not valid",  AssertionLevel.warning);

        // ------ Discretization computation loop ------
          for i in 1:N loop

            // Fluids Properties
              // State
              state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
              state_fg[2, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[2, i], Xi_fg);
              // Temperature
              T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
              T_fg[2, i] = FlueGasesMedium.temperature(state_fg[2, i]);
              T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
              // Density
              rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
              rho_fg[2, i] = FlueGasesMedium.density(state_fg[2, i]);
              rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
              // Dynamic viscosity
              Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
              Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
              Mu_fg[2, i] = FlueGasesMedium.dynamicViscosity(state_fg[2, i]);
              // Specific heat capacities Cp
              Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
              Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
              Cp_fg[2, i] = FlueGasesMedium.specificHeatCapacityCp(state_fg[2, i]);
              // Thermal conductivity
              k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
              k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);
              k_fg[2, i] = FlueGasesMedium.thermalConductivity(state_fg[2, i]);

            // Node energy balance
              // Water side
              dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
              // Flue gas side
              dW_fg[i] = Q_fg/N*(h_fg[2, i] - h_fg[1, i]);
              // Global with wall storage
              dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

      //       // Convection heat transfer coefficient calculation
      //         // Average velocities
      //         U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
      //         // Prandtl number
      //         Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
      //         // Reynold's number
      //         Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
      //         // Nusselt number
      //         Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
      //         // Convection correlation: Dittus-Boelter equation
      //         Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

            // Convection heat transfer equations
              // Water side
              dW_water[i] = K_conv_water*dA_water*(T_wall[i] - T_water_node[i]);
              // Flue gas side
              dW_fg[i] = K_conv_fg*(T_wall[i] - T_fg[1, i])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
      //         Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;

      initial equation

        for i in 1:N loop
          der(T_wall[i]) = 0;
        end for;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end CrossCurrent_MonoPhasicHX_LCM_ConstantK;

      model CrossCurrent_MonoPhasicHX_LCM "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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
          FlueGasesMedium.ThermodynamicState state_fg[2, N] "Flue gas side node boundary states";
          // Enthalpy
          Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
          Units.SpecificEnthalpy h_fg[2, N] "Flue gas specific enthalpy";
          // Mass flow rate
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Pressure
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Density
          Units.Density rho_water[N+1] "Node boundary water density";
          Units.Density rho_water_node[N] "Node average water density";
          Units.Density rho_fg[2, N] "Node boundary flue gas density";
          // Mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
          // Temperature
          Units.Temperature T_water[N+1] "Node boundary water temperature";
          Units.Temperature T_water_node[N] "Node average water temperature";
          Units.Temperature T_fg[2, N] "Node boundary flue gas temperature";
          // Dynamic viscosities
          Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_fg[2, N] "Node boundary flue gas dynamic viscosity";
          // Heat capacities Cp
          Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
          Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
          Units.HeatCapacity Cp_fg[2, N] "Node boundary flue gas Cp";
          // Thermal conductivity
          Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_fg[2, N] "Node boundary flue gas thermal conductivity";

        // ------ Conduction variables ------
          Units.Temperature T_wall[N](each start=T_wall_0) "Node wall average temperature";

        // ------ Tubes configuration parameters ------
          Units.Velocity U_fg_face "Flue gas face velocity";
          Units.Velocity U_fg_max "Flue gas maximum velocity";
          Real Re_fg_max "Flue gas maximum Reynold's number";
          Real Pr_fg "Flue gas overall average Prandtl number";
          FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
          Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
          Real Nu_fg_avg "Flue gass overall average Nusselt number";
          Units.HeatExchangeCoefficient K_conv_fg(each start=K_conv_water_0) "Flue gase convection heat transfer coefficient"; //  = 76.83

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
          Units.HeatExchangeCoefficient K_conv_water[N] "Water side convection heat transfer coefficient"; // = 2418
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

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------
          // Enthalpy
          h_water[1] = water_side.h_in;
          h_water[N+1] = water_side.h_out;
          //sum(h_fg[2, N]) = fg_side.h_out;
          fg_side.W = sum(dW_fg);
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
          U_fg_face = Q_fg/(rho_fg[1, 1]*Af_fg);
          // Flue gas maximum velocity
          if (Tubes_Config == 1) then
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          elseif (S_D < (S_T + D_out)/2) then
            U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
          else
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          end if;
          // Flue gas maximum Reynold's number
          Re_fg_max = rho_fg[1, 1]*U_fg_max*D_out/Mu_fg[1, 1];
          // Flue gas Prandtl number
          Pr_fg = Cp_fg[1, 1]*Mu_fg[1, 1]/k_fg[1, 1];
          // Flue gas Prandtl number at surface temperature
          state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
          Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
          // Convection coefficient calculation
          Nu_fg_avg = K_conv_fg*D_out/k_fg[1, 1];
          // Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);
          Nu_fg_avg = CorrelationConstants.ESCOA(Re_fg_max, Pr_fg, Rows, T_fg[1, 1], T_wall_avg, D_out, H_fin, e_fin, S_f, S_T, S_L);

        // ------ First node properties ------
          state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
          T_water[1] = WaterSteamMedium.temperature(state_water[1]);
          rho_water[1] = WaterSteamMedium.density(state_water[1]);
          Mu_water[1] = WaterSteamMedium.dynamicViscosity(state_water[1]);
          Cp_water[1] = WaterSteamMedium.specificHeatCapacityCp(state_water[1]);
          k_water[1] = WaterSteamMedium.thermalConductivity(state_water[1]);
          for i in 1:N loop
           h_fg[1, i] = fg_side.h_in;
           T_fg[1, i] = FlueGasesMedium.temperature(state_fg[1, i]);
           state_fg[1, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[1, i], Xi_fg);
           rho_fg[1, i] = FlueGasesMedium.density(state_fg[1, i]);
           Mu_fg[1, i] = FlueGasesMedium.dynamicViscosity(state_fg[1, i]);
           Cp_fg[1, i] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1, i]);
           k_fg[1, i] = FlueGasesMedium.thermalConductivity(state_fg[1, i]);
          end for;

        // ------ Parameters of interest ------
          // IN/OUT temperatures
          T_water_in = water_side.T_in;
          T_water_out = water_side.T_out;
          T_fg_in = fg_side.T_in;
          T_fg_out = fg_side.T_out;
          // Average Temperatures
          T_water_avg = sum(T_water_node)/N;
          T_fg_avg = sum(T_fg[2,:])/N;
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
              state_fg[2, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[2, i], Xi_fg);
              // Temperature
              T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
              T_fg[2, i] = FlueGasesMedium.temperature(state_fg[2, i]);
              T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
              // Density
              rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
              rho_fg[2, i] = FlueGasesMedium.density(state_fg[2, i]);
              rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
              // Dynamic viscosity
              Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
              Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
              Mu_fg[2, i] = FlueGasesMedium.dynamicViscosity(state_fg[2, i]);
              // Specific heat capacities Cp
              Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
              Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
              Cp_fg[2, i] = FlueGasesMedium.specificHeatCapacityCp(state_fg[2, i]);
              // Thermal conductivity
              k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
              k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);
              k_fg[2, i] = FlueGasesMedium.thermalConductivity(state_fg[2, i]);

            // Node energy balance
              // Water side
              dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
              // Flue gas side
              dW_fg[i] = Q_fg/N*(h_fg[2, i] - h_fg[1, i]);
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
              dW_fg[i] = K_conv_fg*(T_wall[i] - T_fg[1, i])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
              Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;

      initial equation

        for i in 1:N loop
          der(T_wall[i]) = 0;
        end for;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end CrossCurrent_MonoPhasicHX_LCM;

      model CrossCurrent_MonoPhasicHX_2NodesConduction "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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
          FlueGasesMedium.ThermodynamicState state_fg[2, N] "Flue gas side node boundary states";
          // Enthalpy
          Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
          Units.SpecificEnthalpy h_fg[2, N] "Flue gas specific enthalpy";
          // Mass flow rate
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Pressure
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Density
          Units.Density rho_water[N+1] "Node boundary water density";
          Units.Density rho_water_node[N] "Node average water density";
          Units.Density rho_fg[2, N] "Node boundary flue gas density";
          // Mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
          // Temperature
          Units.Temperature T_water[N+1] "Node boundary water temperature";
          Units.Temperature T_water_node[N] "Node average water temperature";
          Units.Temperature T_fg[2, N] "Node boundary flue gas temperature";
          // Dynamic viscosities
          Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_fg[2, N] "Node boundary flue gas dynamic viscosity";
          // Heat capacities Cp
          Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
          Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
          Units.HeatCapacity Cp_fg[2, N] "Node boundary flue gas Cp";
          // Thermal conductivity
          Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_fg[2, N] "Node boundary flue gas thermal conductivity";

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
          Units.HeatExchangeCoefficient K_conv_fg(each start=K_conv_water_0) "Flue gase convection heat transfer coefficient"; //  = 76.83

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
          Units.HeatExchangeCoefficient K_conv_water[N] "Water side convection heat transfer coefficient"; // = 2418
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

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------
          // Enthalpy
          h_water[1] = water_side.h_in;
          h_water[N+1] = water_side.h_out;
          //sum(h_fg[2, N]) = fg_side.h_out;
          fg_side.W = sum(dW_fg);
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
          U_fg_face = Q_fg/(rho_fg[1, 1]*Af_fg);
          // Flue gas maximum velocity
          if (Tubes_Config == 1) then
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          elseif (S_D < (S_T + D_out)/2) then
            U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
          else
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          end if;
          // Flue gas maximum Reynold's number
          Re_fg_max = rho_fg[1, 1]*U_fg_max*D_out/Mu_fg[1, 1];
          // Flue gas Prandtl number
          Pr_fg = Cp_fg[1, 1]*Mu_fg[1, 1]/k_fg[1, 1];
          // Flue gas Prandtl number at surface temperature
          state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
          Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
          // Convection coefficient calculation
          Nu_fg_avg = K_conv_fg*D_out/k_fg[1, 1];
          // Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);
          Nu_fg_avg = CorrelationConstants.ESCOA(Re_fg_max, Pr_fg, Rows, T_fg[1, 1], T_wall_avg, D_out, H_fin, e_fin, S_f, S_T, S_L);

        // ------ First node properties ------
          state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
          T_water[1] = WaterSteamMedium.temperature(state_water[1]);
          rho_water[1] = WaterSteamMedium.density(state_water[1]);
          Mu_water[1] = WaterSteamMedium.dynamicViscosity(state_water[1]);
          Cp_water[1] = WaterSteamMedium.specificHeatCapacityCp(state_water[1]);
          k_water[1] = WaterSteamMedium.thermalConductivity(state_water[1]);
          for i in 1:N loop
           h_fg[1, i] = fg_side.h_in;
           T_fg[1, i] = FlueGasesMedium.temperature(state_fg[1, i]);
           state_fg[1, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[1, i], Xi_fg);
           rho_fg[1, i] = FlueGasesMedium.density(state_fg[1, i]);
           Mu_fg[1, i] = FlueGasesMedium.dynamicViscosity(state_fg[1, i]);
           Cp_fg[1, i] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1, i]);
           k_fg[1, i] = FlueGasesMedium.thermalConductivity(state_fg[1, i]);
          end for;

        // ------ Parameters of interest ------
          // IN/OUT temperatures
          T_water_in = water_side.T_in;
          T_water_out = water_side.T_out;
          T_fg_in = fg_side.T_in;
          T_fg_out = fg_side.T_out;
          // Average Temperatures
          T_water_avg = sum(T_water_node)/N;
          T_fg_avg = sum(T_fg[2,:])/N;
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
              state_fg[2, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[2, i], Xi_fg);
              // Temperature
              T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
              T_fg[2, i] = FlueGasesMedium.temperature(state_fg[2, i]);
              T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
              // Density
              rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
              rho_fg[2, i] = FlueGasesMedium.density(state_fg[2, i]);
              rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
              // Dynamic viscosity
              Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
              Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
              Mu_fg[2, i] = FlueGasesMedium.dynamicViscosity(state_fg[2, i]);
              // Specific heat capacities Cp
              Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
              Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
              Cp_fg[2, i] = FlueGasesMedium.specificHeatCapacityCp(state_fg[2, i]);
              // Thermal conductivity
              k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
              k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);
              k_fg[2, i] = FlueGasesMedium.thermalConductivity(state_fg[2, i]);

            // Conduction heat transfer
              dW_water[i] = 2*pi*K_cond_wall*dx*N_tubes*(T_wall[i] - T_wall_water[i])/(Modelica.Math.log(1 + e/D_in));
              dW_fg[i] = 2*pi*K_cond_wall*dx*N_tubes*(T_wall[i] - T_wall_fg[i])/(Modelica.Math.log(1 + e/(e + D_in)));

            // Node energy balance
              // Water side
              dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
              // Flue gas side
              dW_fg[i] = Q_fg/N*(h_fg[2, i] - h_fg[1, i]);
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
              dW_fg[i] = K_conv_fg*(T_wall_fg[i] - T_fg[1, i])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
              Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;

      initial equation

        for i in 1:N loop
          der(T_wall[i]) = 0;
        end for;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end CrossCurrent_MonoPhasicHX_2NodesConduction;

      model CrossCurrent_MonoPhasicHX_1DConduction_radial "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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
          parameter Integer N_wall = 5;
          parameter Units.Length dy_wall = e/N_wall;

        // ------ Fluids properties ------
          // State
          WaterSteamMedium.ThermodynamicState state_water[N+1] "Water side node boundary state";
          FlueGasesMedium.ThermodynamicState state_fg[2, N] "Flue gas side node boundary states";
          // Enthalpy
          Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
          Units.SpecificEnthalpy h_fg[2, N] "Flue gas specific enthalpy";
          // Mass flow rate
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Pressure
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Density
          Units.Density rho_water[N+1] "Node boundary water density";
          Units.Density rho_water_node[N] "Node average water density";
          Units.Density rho_fg[2, N] "Node boundary flue gas density";
          // Mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
          // Temperature
          Units.Temperature T_water[N+1] "Node boundary water temperature";
          Units.Temperature T_water_node[N] "Node average water temperature";
          Units.Temperature T_fg[2, N] "Node boundary flue gas temperature";
          // Dynamic viscosities
          Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_fg[2, N] "Node boundary flue gas dynamic viscosity";
          // Heat capacities Cp
          Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
          Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
          Units.HeatCapacity Cp_fg[2, N] "Node boundary flue gas Cp";
          // Thermal conductivity
          Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_fg[2, N] "Node boundary flue gas thermal conductivity";

        // ------ Conduction variables ------
          Units.Temperature T_wall_y[N_wall+1, N] "Wall temperature distribution";

        // ------ Tubes configuration parameters ------
          Units.Velocity U_fg_face "Flue gas face velocity";
          Units.Velocity U_fg_max "Flue gas maximum velocity";
          Real Re_fg_max "Flue gas maximum Reynold's number";
          Real Pr_fg "Flue gas overall average Prandtl number";
          FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
          Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
          Real Nu_fg_avg "Flue gass overall average Nusselt number";
          Units.HeatExchangeCoefficient K_conv_fg(each start=K_conv_water_0) "Flue gase convection heat transfer coefficient"; //  = 76.83

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
          Units.HeatExchangeCoefficient K_conv_water[N] "Water side convection heat transfer coefficient"; // = 2418
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

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------
          // Enthalpy
          h_water[1] = water_side.h_in;
          h_water[N+1] = water_side.h_out;
          //sum(h_fg[2, N]) = fg_side.h_out;
          fg_side.W = sum(dW_fg);
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
          U_fg_face = Q_fg/(rho_fg[1, 1]*Af_fg);
          // Flue gas maximum velocity
          if (Tubes_Config == 1) then
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          elseif (S_D < (S_T + D_out)/2) then
            U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
          else
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          end if;
          // Flue gas maximum Reynold's number
          Re_fg_max = rho_fg[1, 1]*U_fg_max*D_out/Mu_fg[1, 1];
          // Flue gas Prandtl number
          Pr_fg = Cp_fg[1, 1]*Mu_fg[1, 1]/k_fg[1, 1];
          // Flue gas Prandtl number at surface temperature
          state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
          Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
          // Convection coefficient calculation
          Nu_fg_avg = K_conv_fg*D_out/k_fg[1, 1];
          // Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);
          Nu_fg_avg = CorrelationConstants.ESCOA(Re_fg_max, Pr_fg, Rows, T_fg[1, 1], T_wall_avg, D_out, H_fin, e_fin, S_f, S_T, S_L);

        // ------ First node properties ------
          state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
          T_water[1] = WaterSteamMedium.temperature(state_water[1]);
          rho_water[1] = WaterSteamMedium.density(state_water[1]);
          Mu_water[1] = WaterSteamMedium.dynamicViscosity(state_water[1]);
          Cp_water[1] = WaterSteamMedium.specificHeatCapacityCp(state_water[1]);
          k_water[1] = WaterSteamMedium.thermalConductivity(state_water[1]);
          for i in 1:N loop
           h_fg[1, i] = fg_side.h_in;
           T_fg[1, i] = FlueGasesMedium.temperature(state_fg[1, i]);
           state_fg[1, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[1, i], Xi_fg);
           rho_fg[1, i] = FlueGasesMedium.density(state_fg[1, i]);
           Mu_fg[1, i] = FlueGasesMedium.dynamicViscosity(state_fg[1, i]);
           Cp_fg[1, i] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1, i]);
           k_fg[1, i] = FlueGasesMedium.thermalConductivity(state_fg[1, i]);
          end for;

        // ------ Parameters of interest ------
          // IN/OUT temperatures
          T_water_in = water_side.T_in;
          T_water_out = water_side.T_out;
          T_fg_in = fg_side.T_in;
          T_fg_out = fg_side.T_out;
          // Average Temperatures
          T_water_avg = sum(T_water_node)/N;
          T_fg_avg = sum(T_fg[2,:])/N;
          T_wall_avg =  sum(T_wall_y[N_wall+1])/N;
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
              state_fg[2, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[2, i], Xi_fg);
              // Temperature
              T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
              T_fg[2, i] = FlueGasesMedium.temperature(state_fg[2, i]);
              T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
              // Density
              rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
              rho_fg[2, i] = FlueGasesMedium.density(state_fg[2, i]);
              rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
              // Dynamic viscosity
              Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
              Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
              Mu_fg[2, i] = FlueGasesMedium.dynamicViscosity(state_fg[2, i]);
              // Specific heat capacities Cp
              Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
              Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
              Cp_fg[2, i] = FlueGasesMedium.specificHeatCapacityCp(state_fg[2, i]);
              // Thermal conductivity
              k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
              k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);
              k_fg[2, i] = FlueGasesMedium.thermalConductivity(state_fg[2, i]);

            // Conduction heat transfer
                - dW_water[i] + K_cond_wall*2*pi*dx*N_tubes*(T_wall_y[2,i] - T_wall_y[1,i])/(Modelica.Math.log((D_in + 2*dy_wall)/D_in)) = dM_wall*Cp_wall/N_wall/2*der(T_wall_y[1,i]);
                - dW_fg[i] - K_cond_wall*2*pi*dx*N_tubes*(T_wall_y[N_wall+1,i] - T_wall_y[N_wall,i])/(Modelica.Math.log(D_out/(D_out - 2*dy_wall))) = dM_wall*Cp_wall/N_wall/2*der(T_wall_y[N_wall+1,i]); // check sign

              // Conduction inner nodes
              for j in 2:N_wall loop
                K_cond_wall*2*pi*dx*N_tubes*(T_wall_y[j-1,i] - T_wall_y[j,i])/(Modelica.Math.log((D_in/2 + (j-1)*dy_wall)/(D_in/2 + (j-2)*dy_wall)))
                + K_cond_wall*2*pi*dx*N_tubes*(T_wall_y[j+1,i] - T_wall_y[j,i])/(Modelica.Math.log((D_in/2 + j*dy_wall)/(D_in/2 + (j-1)*dy_wall)))
                = dM_wall*Cp_wall/N_wall*der(T_wall_y[j,i]);
              end for;

            // Node energy balance
              // Water side
              dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
              // Flue gas side
              dW_fg[i] = Q_fg/N*(h_fg[2, i] - h_fg[1, i]);
              // Global with wall storage
              //dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

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
              dW_water[i] = K_conv_water[i]*dA_water*(T_wall_y[1, i] - T_water_node[i]);
              // Flue gas side
              dW_fg[i] = K_conv_fg*(T_wall_y[N_wall+1, i] - T_fg[1, i])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
              Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;

      initial equation

        for i in 1:N loop
          for j in 1:N_wall+1 loop
            der(T_wall_y[j,i]) = 0;
          end for;
        end for;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end CrossCurrent_MonoPhasicHX_1DConduction_radial;

      model CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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

        // ------ Discretization z axis ------
          parameter Integer N = 1;
          parameter Units.Mass dM_wall = M_wall/N/Rows "Tube mass of a single node";
          parameter Units.Area dA_water = A_water/N/Rows "Water side heat exchange surface of a single node";
          parameter Units.Area dA_fg_tubes = A_fg_tubes/N/Rows "Flue gas side heat exchange surface of a single node";
          parameter Units.Area dA_fin_cb = A_fin_cb/N/Rows;
          parameter Units.Area dA_fg_fin = A_fg_fins/N/Rows;
          parameter Units.Length dz = L/N;

        // ------ Fluids properties ------
          // State
          WaterSteamMedium.ThermodynamicState state_water[Rows, N+1] "Water side node boundary state";
          FlueGasesMedium.ThermodynamicState state_fg[Rows + 1, N] "Flue gas side node boundary states";
          // Enthalpy
          Units.SpecificEnthalpy h_water[Rows, N+1](each start=h_water_out_0) "Water specific enthalpy";
          Units.SpecificEnthalpy h_fg[Rows + 1, N] "Flue gas specific enthalpy";
          // Mass flow rate
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Pressure
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Density
          Units.Density rho_water[Rows, N+1] "Node boundary water density";
          Units.Density rho_water_node[Rows, N] "Node average water density";
          Units.Density rho_fg[Rows + 1, N] "Node boundary flue gas density";
          // Mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
          // Temperature
          Units.Temperature T_water[Rows, N+1] "Node boundary water temperature";
          Units.Temperature T_water_node[Rows, N] "Node average water temperature";
          Units.Temperature T_fg[Rows + 1, N] "Node boundary flue gas temperature";
          // Dynamic viscosities
          Modelica.Units.SI.DynamicViscosity Mu_water[Rows, N+1] "Node boundary water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_water_node[Rows, N] "Node average water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_fg[Rows + 1, N] "Node boundary flue gas dynamic viscosity";
          // Heat capacities Cp
          Units.HeatCapacity Cp_water[Rows, N+1] "Node boundary water Cp";
          Units.HeatCapacity Cp_water_node[Rows, N] "Node average water Cp";
          Units.HeatCapacity Cp_fg[Rows + 1, N] "Node boundary flue gas Cp";
          // Thermal conductivity
          Modelica.Units.SI.ThermalConductivity k_water[Rows, N+1] "Node boundary water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_water_node[Rows, N] "Node average water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_fg[Rows + 1, N] "Node boundary flue gas thermal conductivity";

        // ------ Conduction variables ------
          Units.Temperature T_wall[Rows, N] "Node wall average temperature";

          // Convection heat transfer coefficient
          parameter Units.HeatExchangeCoefficient K_conv_water = 2418 "Water side convection heat transfer coefficient";
          parameter Units.HeatExchangeCoefficient K_conv_fg = 76.83 "Flue gase convection heat transfer coefficient"; //  = 82.06
          // Discretized heat transfer power
          Units.Power dW_water[Rows, N] "Node water heat exchange";
          Units.Power dW_fg[Rows, N] "Node flue gas heat exchange";
          // Biot Number
          // Real Bi[N] "Node Biot number";

        // Parameters of interest
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";
          Units.Temperature T_water_avg "Water overall average temperature";
          //Units.Temperature T_fg_avg "Flue gas overall average temperature";
          Units.Temperature T_wall_avg "Wall overall average temperature";

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------

          // Outlet
          water_side.W = sum(dW_water);
          fg_side.W = sum(dW_fg);

          // Inlet
          for i in 1:Rows loop
            h_water[i, 1] = water_side.h_in;
            state_water[i, 1] = WaterSteamMedium.setState_phX(P_water, h_water[i, 1], Xi_water);
            T_water[i, 1] = WaterSteamMedium.temperature(state_water[i, 1]);
            rho_water[i, 1] = WaterSteamMedium.density(state_water[i, 1]);
            Mu_water[i, 1] = WaterSteamMedium.dynamicViscosity(state_water[i, 1]);
            Cp_water[i, 1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, 1]);
            k_water[i, 1] = WaterSteamMedium.thermalConductivity(state_water[i, 1]);
          end for;

          for j in 1:N loop
           h_fg[1,  j] = fg_side.h_in;
           T_fg[1,  j] = FlueGasesMedium.temperature(state_fg[1,  j]);
           state_fg[1,  j] = FlueGasesMedium.setState_phX(P_fg, h_fg[1,  j], Xi_fg);
           rho_fg[1,  j] = FlueGasesMedium.density(state_fg[1,  j]);
           Mu_fg[1,  j] = FlueGasesMedium.dynamicViscosity(state_fg[1,  j]);
           Cp_fg[1,  j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1,  j]);
           k_fg[1,  j] = FlueGasesMedium.thermalConductivity(state_fg[1,  j]);
          end for;

          // Pressure
          P_water = water_side.P_in;
          P_fg = fg_side.P_in;
          // Mass flow rate
          Q_water = water_side.Q;
          Q_fg = fg_side.Q;
          // Mass Fraction
          Xi_water = water_side.Xi;
          Xi_fg = fg_side.Xi;

        // ------ Parameters of interest ------
          // IN/OUT temperatures
          T_water_in = water_side.T_in;
          T_water_out = water_side.T_out;
          T_fg_in = fg_side.T_in;
          T_fg_out = fg_side.T_out;
          // Average Temperatures
          T_water_avg = sum(T_water_node)/N;
          //T_fg_avg = sum(T_fg_node)/N;
          T_wall_avg =  sum(T_wall)/N;

        // ------ Discretization computation loop ------
          for i in 1:Rows loop
              for j in 1:N loop
            // Fluids Properties
              // State
              state_water[i, j+1] = WaterSteamMedium.setState_phX(P_water, h_water[i, j+1], Xi_water);
              state_fg[i+1, j] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1, j], Xi_fg);
              // Temperature
              T_water[i, j+1] = WaterSteamMedium.temperature(state_water[i, j+1]);
              T_fg[i+1, j] = FlueGasesMedium.temperature(state_fg[i+1, j]);
              T_water_node[i, j] = 0.5*(T_water[i, j] + T_water[i, j+1]);
              // Density
              rho_water[i, j+1] = WaterSteamMedium.density(state_water[i, j+1]);
              rho_fg[i+1, j] = FlueGasesMedium.density(state_fg[i+1, j]);
              rho_water_node[i, j] = 0.5*(rho_water[i, j] + rho_water[i, j+1]);
              // Dynamic viscosity
              Mu_water[i, j+1] = WaterSteamMedium.dynamicViscosity(state_water[i, j+1]);
              Mu_water_node[i, j] = 0.5*(Mu_water[i, j] + Mu_water[i, j+1]);
              Mu_fg[i+1, j] = FlueGasesMedium.dynamicViscosity(state_fg[i+1, j]);
              // Specific heat capacities Cp
              Cp_water[i, j+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, j+1]);
              Cp_water_node[i, j] = 0.5*(Cp_water[i, j] + Cp_water[i, j+1]);
              Cp_fg[i+1, j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1, j]);
              // Thermal conductivity
              k_water[i, j+1] = WaterSteamMedium.thermalConductivity(state_water[i, j+1]);
              k_water_node[i, j] = 0.5*(k_water[i, j] + k_water[i, j+1]);
              k_fg[i+1, j] = FlueGasesMedium.thermalConductivity(state_fg[i+1, j]);

            // Node energy balance
              // Water side
              dW_water[i, j] = Q_water/Rows*(h_water[i, j+1] - h_water[i, j]);
              // Flue gas side
              dW_fg[i, j] = Q_fg/N*(h_fg[i+1, j] - h_fg[i, j]);
              // Global with wall storage
              dW_water[i, j] + dW_fg[i, j] + dM_wall*Cp_wall*der(T_wall[i, j]) = 0;

      //       // Convection heat transfer coefficient calculation
      //         // Average velocities
      //         U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
      //         // Prandtl number
      //         Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
      //         // Reynold's number
      //         Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
      //         // Nusselt number
      //         Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
      //         // Convection correlation: Dittus-Boelter equation
      //         Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

            // Convection heat transfer equations
              // Water side
              dW_water[i, j] = K_conv_water*dA_water*(T_wall[i, j] - T_water_node[i, j]);
              // Flue gas side
              dW_fg[i, j] = K_conv_fg*(T_wall[i, j] - T_fg[i, j])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
      //         Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;
          end for;

      initial equation

        for i in 1:Rows loop
          for j in 1:N loop
          der(T_wall[i, j]) = 0;
          end for;
        end for;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK;

      model CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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

        // ------ Discretization z axis ------
          parameter Integer N = 1;
          parameter Units.Mass dM_wall = M_wall/N/Rows "Tube mass of a single node";
          parameter Units.Area dA_water = A_water/N/Rows "Water side heat exchange surface of a single node";
          parameter Units.Area dA_fg_tubes = A_fg_tubes/N/Rows "Flue gas side heat exchange surface of a single node";
          parameter Units.Area dA_fin_cb = A_fin_cb/N/Rows;
          parameter Units.Area dA_fg_fin = A_fg_fins/N/Rows;
          parameter Units.Length dz = L/N;

        // ------ Fluids properties ------
            // State
            WaterSteamMedium.ThermodynamicState state_water[Rows, N+1] "Water side node boundary state";
            FlueGasesMedium.ThermodynamicState state_fg[Rows + 1, N] "Flue gas side node boundary states";
            // Enthalpy
            Units.SpecificEnthalpy h_water[Rows, N+1](each start=h_water_out_0) "Water specific enthalpy";
            Units.SpecificEnthalpy h_fg[Rows + 1, N] "Flue gas specific enthalpy";
            // Mass flow rate
            Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
            Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
            // Pressure
            Units.Pressure P_water(start=P_water_0) "Water Pressure";
            Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
            // Density
            Units.Density rho_water[Rows, N+1] "Node boundary water density";
            Units.Density rho_water_node[Rows, N] "Node average water density";
            Units.Density rho_fg[Rows + 1, N] "Node boundary flue gas density";
            Units.Density rho_fg_node[Rows, N] "Node average flue gas density";
            // Mass fraction
            Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
            Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
            // Temperature
            Units.Temperature T_water[Rows, N+1] "Node boundary water temperature";
            Units.Temperature T_water_node[Rows, N] "Node average water temperature";
            Units.Temperature T_fg[Rows + 1, N] "Node boundary flue gas temperature";
            Units.Temperature T_fg_node[Rows, N] "Node average flue gas temperature";
            // Dynamic viscosities
            Modelica.Units.SI.DynamicViscosity Mu_water[Rows, N+1] "Node boundary water dynamic viscosity";
            Modelica.Units.SI.DynamicViscosity Mu_water_node[Rows, N] "Node average water dynamic viscosity";
            Modelica.Units.SI.DynamicViscosity Mu_fg[Rows + 1, N] "Node boundary flue gas dynamic viscosity";
            Modelica.Units.SI.DynamicViscosity Mu_fg_node[Rows, N] "Node average flue gas dynamic viscosity";
            // Heat capacities Cp
            Units.HeatCapacity Cp_water[Rows, N+1] "Node boundary water Cp";
            Units.HeatCapacity Cp_water_node[Rows, N] "Node average water Cp";
            Units.HeatCapacity Cp_fg[Rows + 1, N] "Node boundary flue gas Cp";
            Units.HeatCapacity Cp_fg_node[Rows, N] "Node average flue gas Cp";
            // Thermal conductivity
            Modelica.Units.SI.ThermalConductivity k_water[Rows, N+1] "Node boundary water thermal conductivity";
            Modelica.Units.SI.ThermalConductivity k_water_node[Rows, N] "Node average water thermal conductivity";
            Modelica.Units.SI.ThermalConductivity k_fg[Rows + 1, N] "Node boundary flue gas thermal conductivity";
            Modelica.Units.SI.ThermalConductivity k_fg_node[Rows, N] "Node average flue gas thermal conductivity";

        // ------ Conduction variables ------
          Units.Temperature T_wall[Rows, N] "Node wall average temperature";

          // Convection heat transfer coefficient
          parameter Units.HeatExchangeCoefficient K_conv_water = 2418 "Water side convection heat transfer coefficient";
          parameter Units.HeatExchangeCoefficient K_conv_fg = 76.83 "Flue gase convection heat transfer coefficient"; //  = 82.06
          // Discretized heat transfer power
          Units.Power dW_water[Rows, N] "Node water heat exchange";
          Units.Power dW_fg[Rows, N] "Node flue gas heat exchange";
          // Biot Number
          // Real Bi[N] "Node Biot number";

        // Parameters of interest
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";
          Units.Temperature T_water_avg "Water overall average temperature";
          //Units.Temperature T_fg_avg "Flue gas overall average temperature";
          Units.Temperature T_wall_avg "Wall overall average temperature";

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------

          // Outlet
          water_side.W = sum(dW_water);
          fg_side.W = sum(dW_fg);

          // Inlet
          for i in 1:Rows loop
            h_water[i, 1] = water_side.h_in;
            state_water[i, 1] = WaterSteamMedium.setState_phX(P_water, h_water[i, 1], Xi_water);
            T_water[i, 1] = WaterSteamMedium.temperature(state_water[i, 1]);
            rho_water[i, 1] = WaterSteamMedium.density(state_water[i, 1]);
            Mu_water[i, 1] = WaterSteamMedium.dynamicViscosity(state_water[i, 1]);
            Cp_water[i, 1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, 1]);
            k_water[i, 1] = WaterSteamMedium.thermalConductivity(state_water[i, 1]);
          end for;

          for j in 1:N loop
           h_fg[1,  j] = fg_side.h_in;
           T_fg[1,  j] = FlueGasesMedium.temperature(state_fg[1,  j]);
           state_fg[1,  j] = FlueGasesMedium.setState_phX(P_fg, h_fg[1,  j], Xi_fg);
           rho_fg[1,  j] = FlueGasesMedium.density(state_fg[1,  j]);
           Mu_fg[1,  j] = FlueGasesMedium.dynamicViscosity(state_fg[1,  j]);
           Cp_fg[1,  j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1,  j]);
           k_fg[1,  j] = FlueGasesMedium.thermalConductivity(state_fg[1,  j]);
          end for;

          // Pressure
          P_water = water_side.P_in;
          P_fg = fg_side.P_in;
          // Mass flow rate
          Q_water = water_side.Q;
          Q_fg = fg_side.Q;
          // Mass Fraction
          Xi_water = water_side.Xi;
          Xi_fg = fg_side.Xi;

        // ------ Parameters of interest ------
          // IN/OUT temperatures
          T_water_in = water_side.T_in;
          T_water_out = water_side.T_out;
          T_fg_in = fg_side.T_in;
          T_fg_out = fg_side.T_out;
          // Average Temperatures
          T_water_avg = sum(T_water_node)/N;
          //T_fg_avg = sum(T_fg_node)/N;
          T_wall_avg =  sum(T_wall)/N;

        // ------ Discretization computation loop ------
          for i in 1:Rows loop
              for j in 1:N loop
            // Fluids Properties
                // State equations
                state_water[i, j+1] = WaterSteamMedium.setState_phX(P_water, h_water[i, j+1], Xi_water);
                state_fg[i+1, j] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1, j], Xi_fg);

                // Temperature
                T_water[i, j+1] = WaterSteamMedium.temperature(state_water[i, j+1]);
                T_fg[i+1, j] = FlueGasesMedium.temperature(state_fg[i+1, j]);
                T_water_node[i, j] = 0.5*(T_water[i, j] + T_water[i, j+1]);
                T_fg_node[i, j] = 0.5*(T_fg[i, j] + T_fg[i+1, j]);

                // Density
                rho_water[i, j+1] = WaterSteamMedium.density(state_water[i, j+1]);
                rho_fg[i+1, j] = FlueGasesMedium.density(state_fg[i+1, j]);
                rho_water_node[i, j] = 0.5*(rho_water[i, j] + rho_water[i, j+1]);
                rho_fg_node[i, j] = 0.5*(rho_fg[i+1, j] + rho_fg[i+1, j]);

                // Dynamic viscosity
                Mu_water[i, j+1] = WaterSteamMedium.dynamicViscosity(state_water[i, j+1]);
                Mu_water_node[i, j] = 0.5*(Mu_water[i, j] + Mu_water[i, j+1]);
                Mu_fg[i+1, j] = FlueGasesMedium.dynamicViscosity(state_fg[i+1, j]);
                Mu_fg_node[i, j] = 0.5*(Mu_fg[i+1, j] + Mu_fg[i+1, j]);

                // Specific heat capacities Cp
                Cp_water[i, j+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, j+1]);
                Cp_water_node[i, j] = 0.5*(Cp_water[i, j] + Cp_water[i, j+1]);
                Cp_fg[i+1, j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1, j]);
                Cp_fg_node[i, j] = 0.5*(Cp_fg[i+1, j] + Cp_fg[i+1, j]);

                // Thermal conductivity
                k_water[i, j+1] = WaterSteamMedium.thermalConductivity(state_water[i, j+1]);
                k_water_node[i, j] = 0.5*(k_water[i, j] + k_water[i, j+1]);
                k_fg[i+1, j] = FlueGasesMedium.thermalConductivity(state_fg[i+1, j]);
                k_fg_node[i, j] = 0.5*(k_fg[i+1, j] + k_fg[i+1, j]);

            // Node energy balance
              // Water side
              dW_water[i, j] = Q_water/Rows*(h_water[i, j+1] - h_water[i, j]);
              // Flue gas side
              dW_fg[i, j] = Q_fg/N*(h_fg[i+1, j] - h_fg[i, j]);
              // Global with wall storage
              dW_water[i, j] + dW_fg[i, j] + dM_wall*Cp_wall*der(T_wall[i, j]) = 0;

      //       // Convection heat transfer coefficient calculation
      //         // Average velocities
      //         U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
      //         // Prandtl number
      //         Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
      //         // Reynold's number
      //         Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
      //         // Nusselt number
      //         Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
      //         // Convection correlation: Dittus-Boelter equation
      //         Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

            // Convection heat transfer equations
              // Water side
              dW_water[i, j] = K_conv_water*dA_water*(T_wall[i, j] - T_water_node[i, j])
              - N_tubes_row*0.25*pi*D_in^2*dz*rho_water_node[i, j]*Cp_water_node[i, j]*der(T_water_node[i, j]);
              // Flue gas side
              dW_fg[i, j] = K_conv_fg*(T_wall[i, j] - T_fg[i, j])*(dA_fg_tubes + eff_fins*dA_fg_fin)
              - N_tubes_row*dz*(S_T*S_L - 0.25*pi*D_out^2)*rho_fg_node[i, j]*Cp_fg_node[i, j]*der(T_fg_node[i, j]);

            // Biot number
      //         Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;
          end for;

      initial equation

        for i in 1:Rows loop
          for j in 1:N loop
          der(T_wall[i, j]) = 0;
          der(T_water_node[i, j]) =0;
          der(T_fg_node[i, j]) = 0;
          end for;
        end for;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage;

      model CrossCurrent_1NodePerRow_MonoPhasicHX_LCM "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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

        // ------ Discretization z axis ------
          parameter Integer N = 1;
          parameter Units.Mass dM_wall = M_wall/N/Rows "Tube mass of a single node";
          parameter Units.Area dA_water = A_water/N/Rows "Water side heat exchange surface of a single node";
          parameter Units.Area dA_fg_tubes = A_fg_tubes/N/Rows "Flue gas side heat exchange surface of a single node";
          parameter Units.Area dA_fin_cb = A_fin_cb/N/Rows;
          parameter Units.Area dA_fg_fin = A_fg_fins/N/Rows;
          parameter Units.Length dz = L/N;

        // ------ Fluids properties ------
          // State
          WaterSteamMedium.ThermodynamicState state_water[Rows, N+1] "Water side node boundary state";
          FlueGasesMedium.ThermodynamicState state_fg[Rows + 1, N] "Flue gas side node boundary states";
          // Enthalpy
          Units.SpecificEnthalpy h_water[Rows, N+1](each start=h_water_out_0) "Water specific enthalpy";
          Units.SpecificEnthalpy h_fg[Rows + 1, N] "Flue gas specific enthalpy";
          // Mass flow rate
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Pressure
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Density
          Units.Density rho_water[Rows, N+1] "Node boundary water density";
          Units.Density rho_water_node[Rows, N] "Node average water density";
          Units.Density rho_fg[Rows + 1, N] "Node boundary flue gas density";
          // Mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
          // Temperature
          Units.Temperature T_water[Rows, N+1] "Node boundary water temperature";
          Units.Temperature T_water_node[Rows, N] "Node average water temperature";
          Units.Temperature T_fg[Rows + 1, N] "Node boundary flue gas temperature";
          // Dynamic viscosities
          Modelica.Units.SI.DynamicViscosity Mu_water[Rows, N+1] "Node boundary water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_water_node[Rows, N] "Node average water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_fg[Rows + 1, N] "Node boundary flue gas dynamic viscosity";
          // Heat capacities Cp
          Units.HeatCapacity Cp_water[Rows, N+1] "Node boundary water Cp";
          Units.HeatCapacity Cp_water_node[Rows, N] "Node average water Cp";
          Units.HeatCapacity Cp_fg[Rows + 1, N] "Node boundary flue gas Cp";
          // Thermal conductivity
          Modelica.Units.SI.ThermalConductivity k_water[Rows, N+1] "Node boundary water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_water_node[Rows, N] "Node average water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_fg[Rows + 1, N] "Node boundary flue gas thermal conductivity";

        // ------ Conduction variables ------
          Units.Temperature T_wall[Rows, N] "Node wall average temperature";

        // ------ Tubes configuration parameters ------
          Units.Velocity U_fg_face "Flue gas face velocity";
          Units.Velocity U_fg_max "Flue gas maximum velocity";
          Real Re_fg_max "Flue gas maximum Reynold's number";
          Real Pr_fg "Flue gas overall average Prandtl number";
          FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
          Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
          Real Nu_fg_avg "Flue gass overall average Nusselt number";
          Units.HeatExchangeCoefficient K_conv_fg(each start=K_conv_water_0) "Flue gase convection heat transfer coefficient"; //  = 76.83

        // ------ Heat transfer parameters ------
          // Average velocities
          Units.Velocity U_water_node[Rows, N] "Node average water velocity";
          // Prandtl Number
          Real Pr_water[Rows, N] "Node water Prandtl's number";
          // Reynold's Number
          Real Re_water[Rows, N] "Node water Reynold's number";
          // Nusselt Nymber
          Real Nu_water[Rows, N] "Node water Nusselt number";
          // Convection heat transfer coefficient
          Units.HeatExchangeCoefficient K_conv_water[Rows, N](each start=K_conv_water_0) "Water side convection heat transfer coefficient"; // = 2418
          // Discretized heat transfer power
          Units.Power dW_water[Rows, N] "Node water heat exchange";
          Units.Power dW_fg[Rows, N] "Node flue gas heat exchange";
          // Biot Number
          // Real Bi[N] "Node Biot number";

        // Parameters of interest
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";
          Units.Temperature T_water_avg "Water overall average temperature";
          //Units.Temperature T_fg_avg "Flue gas overall average temperature";
          Units.Temperature T_wall_avg "Wall overall average temperature";

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------

          // Outlet
          water_side.W = sum(dW_water);
          fg_side.W = sum(dW_fg);

          // Inlet
          for i in 1:Rows loop
            h_water[i, 1] = water_side.h_in;
            state_water[i, 1] = WaterSteamMedium.setState_phX(P_water, h_water[i, 1], Xi_water);
            T_water[i, 1] = WaterSteamMedium.temperature(state_water[i, 1]);
            rho_water[i, 1] = WaterSteamMedium.density(state_water[i, 1]);
            Mu_water[i, 1] = WaterSteamMedium.dynamicViscosity(state_water[i, 1]);
            Cp_water[i, 1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, 1]);
            k_water[i, 1] = WaterSteamMedium.thermalConductivity(state_water[i, 1]);
          end for;

          for j in 1:N loop
           h_fg[1,  j] = fg_side.h_in;
           T_fg[1,  j] = FlueGasesMedium.temperature(state_fg[1,  j]);
           state_fg[1,  j] = FlueGasesMedium.setState_phX(P_fg, h_fg[1,  j], Xi_fg);
           rho_fg[1,  j] = FlueGasesMedium.density(state_fg[1,  j]);
           Mu_fg[1,  j] = FlueGasesMedium.dynamicViscosity(state_fg[1,  j]);
           Cp_fg[1,  j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1,  j]);
           k_fg[1,  j] = FlueGasesMedium.thermalConductivity(state_fg[1,  j]);
          end for;

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
          U_fg_face = Q_fg/(rho_fg[1, 1]*Af_fg);
          // Flue gas maximum velocity
          if (Tubes_Config == 1) then
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          elseif (S_D < (S_T + D_out)/2) then
            U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
          else
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          end if;
          // Flue gas maximum Reynold's number
          Re_fg_max = rho_fg[1, 1]*U_fg_max*D_out/Mu_fg[1, 1];
          // Flue gas Prandtl number
          Pr_fg = Cp_fg[1, 1]*Mu_fg[1, 1]/k_fg[1, 1];
          // Flue gas Prandtl number at surface temperature
          state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
          Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
          // Convection coefficient calculation
          Nu_fg_avg = K_conv_fg*D_out/k_fg[1, 1];
          // Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);
          Nu_fg_avg = CorrelationConstants.ESCOA(Re_fg_max, Pr_fg, Rows, T_fg[1, 1], T_wall_avg, D_out, H_fin, e_fin, S_f, S_T, S_L);

        // ------ Parameters of interest ------
          // IN/OUT temperatures
          T_water_in = water_side.T_in;
          T_water_out = water_side.T_out;
          T_fg_in = fg_side.T_in;
          T_fg_out = fg_side.T_out;
          // Average Temperatures
          T_water_avg = sum(T_water_node)/(Rows*N);
          //T_fg_avg = sum(T_fg_node)/N;
          T_wall_avg =  sum(T_wall)/(Rows*N);

        // ------ Discretization computation loop ------
          for i in 1:Rows loop
              for j in 1:N loop
            // Fluids Properties
              // State
              state_water[i, j+1] = WaterSteamMedium.setState_phX(P_water, h_water[i, j+1], Xi_water);
              state_fg[i+1, j] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1, j], Xi_fg);
              // Temperature
              T_water[i, j+1] = WaterSteamMedium.temperature(state_water[i, j+1]);
              T_fg[i+1, j] = FlueGasesMedium.temperature(state_fg[i+1, j]);
              T_water_node[i, j] = 0.5*(T_water[i, j] + T_water[i, j+1]);
              // Density
              rho_water[i, j+1] = WaterSteamMedium.density(state_water[i, j+1]);
              rho_fg[i+1, j] = FlueGasesMedium.density(state_fg[i+1, j]);
              rho_water_node[i, j] = 0.5*(rho_water[i, j] + rho_water[i, j+1]);
              // Dynamic viscosity
              Mu_water[i, j+1] = WaterSteamMedium.dynamicViscosity(state_water[i, j+1]);
              Mu_water_node[i, j] = 0.5*(Mu_water[i, j] + Mu_water[i, j+1]);
              Mu_fg[i+1, j] = FlueGasesMedium.dynamicViscosity(state_fg[i+1, j]);
              // Specific heat capacities Cp
              Cp_water[i, j+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, j+1]);
              Cp_water_node[i, j] = 0.5*(Cp_water[i, j] + Cp_water[i, j+1]);
              Cp_fg[i+1, j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1, j]);
              // Thermal conductivity
              k_water[i, j+1] = WaterSteamMedium.thermalConductivity(state_water[i, j+1]);
              k_water_node[i, j] = 0.5*(k_water[i, j] + k_water[i, j+1]);
              k_fg[i+1, j] = FlueGasesMedium.thermalConductivity(state_fg[i+1, j]);

            // Node energy balance
              // Water side
              dW_water[i, j] = Q_water/Rows*(h_water[i, j+1] - h_water[i, j]);
              // Flue gas side
              dW_fg[i, j] = Q_fg/N*(h_fg[i+1, j] - h_fg[i, j]);
              // Global with wall storage
              dW_water[i, j] + dW_fg[i, j] + dM_wall*Cp_wall*der(T_wall[i, j]) = 0;

            // Convection heat transfer coefficient calculation
              // Average velocities
              U_water_node[i, j] = Q_water/(rho_water_node[i, j]*Ac_water);
              // Prandtl number
              Pr_water[i, j] = Cp_water_node[i, j]*Mu_water_node[i, j]/k_water_node[i, j];
              // Reynold's number
              Re_water[i, j] = rho_water_node[i, j]*U_water_node[i, j]*D_in/Mu_water_node[i, j];
              // Nusselt number
              Nu_water[i, j] = K_conv_water[i, j]*D_in/k_water_node[i, j];
              // Convection correlation: Dittus-Boelter equation
              Nu_water[i, j] = 0.023*Re_water[i, j]^0.8*Pr_water[i, j]^0.4;

            // Convection heat transfer equations
              // Water side
              dW_water[i, j] = K_conv_water[i, j]*dA_water*(T_wall[i, j] - T_water_node[i, j]);
              // Flue gas side
              dW_fg[i, j] = K_conv_fg*(T_wall[i, j] - T_fg[i, j])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
      //         Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;
          end for;

      initial equation

        for i in 1:Rows loop
          for j in 1:N loop
          der(T_wall[i, j]) = 0;
          end for;
        end for;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end CrossCurrent_1NodePerRow_MonoPhasicHX_LCM;

      model CrossCurrent_1NodePerRow_MonoPhasicHX_2NodesConduction "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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

        // ------ Discretization z axis ------
          parameter Integer N = 1;
          parameter Units.Mass dM_wall = M_wall/N/Rows "Tube mass of a single node";
          parameter Units.Area dA_water = A_water/N/Rows "Water side heat exchange surface of a single node";
          parameter Units.Area dA_fg_tubes = A_fg_tubes/N/Rows "Flue gas side heat exchange surface of a single node";
          parameter Units.Area dA_fin_cb = A_fin_cb/N/Rows;
          parameter Units.Area dA_fg_fin = A_fg_fins/N/Rows;
          parameter Units.Length dz = L/N;

        // ------ Fluids properties ------
          // State
          WaterSteamMedium.ThermodynamicState state_water[Rows, N+1] "Water side node boundary state";
          FlueGasesMedium.ThermodynamicState state_fg[Rows + 1, N] "Flue gas side node boundary states";
          // Enthalpy
          Units.SpecificEnthalpy h_water[Rows, N+1](each start=h_water_out_0) "Water specific enthalpy";
          Units.SpecificEnthalpy h_fg[Rows + 1, N] "Flue gas specific enthalpy";
          // Mass flow rate
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Pressure
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Density
          Units.Density rho_water[Rows, N+1] "Node boundary water density";
          Units.Density rho_water_node[Rows, N] "Node average water density";
          Units.Density rho_fg[Rows + 1, N] "Node boundary flue gas density";
          // Mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
          // Temperature
          Units.Temperature T_water[Rows, N+1] "Node boundary water temperature";
          Units.Temperature T_water_node[Rows, N] "Node average water temperature";
          Units.Temperature T_fg[Rows + 1, N] "Node boundary flue gas temperature";
          // Dynamic viscosities
          Modelica.Units.SI.DynamicViscosity Mu_water[Rows, N+1] "Node boundary water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_water_node[Rows, N] "Node average water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_fg[Rows + 1, N] "Node boundary flue gas dynamic viscosity";
          // Heat capacities Cp
          Units.HeatCapacity Cp_water[Rows, N+1] "Node boundary water Cp";
          Units.HeatCapacity Cp_water_node[Rows, N] "Node average water Cp";
          Units.HeatCapacity Cp_fg[Rows + 1, N] "Node boundary flue gas Cp";
          // Thermal conductivity
          Modelica.Units.SI.ThermalConductivity k_water[Rows, N+1] "Node boundary water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_water_node[Rows, N] "Node average water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_fg[Rows + 1, N] "Node boundary flue gas thermal conductivity";

        // ------ Conduction variables ------
          Units.Temperature T_wall_water[Rows, N] "Wall temperature from the water side";
          Units.Temperature T_wall[Rows, N] "Node wall average temperature";
          Units.Temperature T_wall_fg[Rows, N] "Wall temperature from the flue gas side";

        // ------ Tubes configuration parameters ------
          Units.Velocity U_fg_face "Flue gas face velocity";
          Units.Velocity U_fg_max "Flue gas maximum velocity";
          Real Re_fg_max "Flue gas maximum Reynold's number";
          Real Pr_fg "Flue gas overall average Prandtl number";
          FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
          Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
          Real Nu_fg_avg "Flue gass overall average Nusselt number";
          Units.HeatExchangeCoefficient K_conv_fg(each start=K_conv_water_0) "Flue gase convection heat transfer coefficient"; //  = 76.83

        // ------ Heat transfer parameters ------
          // Average velocities
          Units.Velocity U_water_node[Rows, N] "Node average water velocity";
          // Prandtl Number
          Real Pr_water[Rows, N] "Node water Prandtl's number";
          // Reynold's Number
          Real Re_water[Rows, N] "Node water Reynold's number";
          // Nusselt Nymber
          Real Nu_water[Rows, N] "Node water Nusselt number";
          // Convection heat transfer coefficient
          Units.HeatExchangeCoefficient K_conv_water[Rows, N](each start=K_conv_water_0) "Water side convection heat transfer coefficient"; // = 2418
          // Discretized heat transfer power
          Units.Power dW_water[Rows, N] "Node water heat exchange";
          Units.Power dW_fg[Rows, N] "Node flue gas heat exchange";
          // Biot Number
          // Real Bi[N] "Node Biot number";

        // Parameters of interest
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";
          Units.Temperature T_water_avg "Water overall average temperature";
          //Units.Temperature T_fg_avg "Flue gas overall average temperature";
          Units.Temperature T_wall_avg "Wall overall average temperature";

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------

          // Outlet
          water_side.W = sum(dW_water);
          fg_side.W = sum(dW_fg);

          // Inlet
          for i in 1:Rows loop
            h_water[i, 1] = water_side.h_in;
            state_water[i, 1] = WaterSteamMedium.setState_phX(P_water, h_water[i, 1], Xi_water);
            T_water[i, 1] = WaterSteamMedium.temperature(state_water[i, 1]);
            rho_water[i, 1] = WaterSteamMedium.density(state_water[i, 1]);
            Mu_water[i, 1] = WaterSteamMedium.dynamicViscosity(state_water[i, 1]);
            Cp_water[i, 1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, 1]);
            k_water[i, 1] = WaterSteamMedium.thermalConductivity(state_water[i, 1]);
          end for;

          for j in 1:N loop
           h_fg[1,  j] = fg_side.h_in;
           T_fg[1,  j] = FlueGasesMedium.temperature(state_fg[1,  j]);
           state_fg[1,  j] = FlueGasesMedium.setState_phX(P_fg, h_fg[1,  j], Xi_fg);
           rho_fg[1,  j] = FlueGasesMedium.density(state_fg[1,  j]);
           Mu_fg[1,  j] = FlueGasesMedium.dynamicViscosity(state_fg[1,  j]);
           Cp_fg[1,  j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1,  j]);
           k_fg[1,  j] = FlueGasesMedium.thermalConductivity(state_fg[1,  j]);
          end for;

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
          U_fg_face = Q_fg/(rho_fg[1, 1]*Af_fg);
          // Flue gas maximum velocity
          if (Tubes_Config == 1) then
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          elseif (S_D < (S_T + D_out)/2) then
            U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
          else
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          end if;
          // Flue gas maximum Reynold's number
          Re_fg_max = rho_fg[1, 1]*U_fg_max*D_out/Mu_fg[1, 1];
          // Flue gas Prandtl number
          Pr_fg = Cp_fg[1, 1]*Mu_fg[1, 1]/k_fg[1, 1];
          // Flue gas Prandtl number at surface temperature
          state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
          Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
          // Convection coefficient calculation
          Nu_fg_avg = K_conv_fg*D_out/k_fg[1, 1];
          // Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);
          Nu_fg_avg = CorrelationConstants.ESCOA(Re_fg_max, Pr_fg, Rows, T_fg[1, 1], T_wall_avg, D_out, H_fin, e_fin, S_f, S_T, S_L);

        // ------ Parameters of interest ------
          // IN/OUT temperatures
          T_water_in = water_side.T_in;
          T_water_out = water_side.T_out;
          T_fg_in = fg_side.T_in;
          T_fg_out = fg_side.T_out;
          // Average Temperatures
          T_water_avg = sum(T_water_node)/(Rows*N);
          //T_fg_avg = sum(T_fg_node)/N;
          T_wall_avg =  sum(T_wall)/(Rows*N);

        // ------ Discretization computation loop ------
          for i in 1:Rows loop
              for j in 1:N loop
            // Fluids Properties
              // State
              state_water[i, j+1] = WaterSteamMedium.setState_phX(P_water, h_water[i, j+1], Xi_water);
              state_fg[i+1, j] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1, j], Xi_fg);
              // Temperature
              T_water[i, j+1] = WaterSteamMedium.temperature(state_water[i, j+1]);
              T_fg[i+1, j] = FlueGasesMedium.temperature(state_fg[i+1, j]);
              T_water_node[i, j] = 0.5*(T_water[i, j] + T_water[i, j+1]);
              // Density
              rho_water[i, j+1] = WaterSteamMedium.density(state_water[i, j+1]);
              rho_fg[i+1, j] = FlueGasesMedium.density(state_fg[i+1, j]);
              rho_water_node[i, j] = 0.5*(rho_water[i, j] + rho_water[i, j+1]);
              // Dynamic viscosity
              Mu_water[i, j+1] = WaterSteamMedium.dynamicViscosity(state_water[i, j+1]);
              Mu_water_node[i, j] = 0.5*(Mu_water[i, j] + Mu_water[i, j+1]);
              Mu_fg[i+1, j] = FlueGasesMedium.dynamicViscosity(state_fg[i+1, j]);
              // Specific heat capacities Cp
              Cp_water[i, j+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, j+1]);
              Cp_water_node[i, j] = 0.5*(Cp_water[i, j] + Cp_water[i, j+1]);
              Cp_fg[i+1, j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1, j]);
              // Thermal conductivity
              k_water[i, j+1] = WaterSteamMedium.thermalConductivity(state_water[i, j+1]);
              k_water_node[i, j] = 0.5*(k_water[i, j] + k_water[i, j+1]);
              k_fg[i+1, j] = FlueGasesMedium.thermalConductivity(state_fg[i+1, j]);

            // Conduction heat transfer
              dW_water[i, j] = 2*pi*K_cond_wall*dz*N_tubes*(T_wall[i, j] - T_wall_water[i, j])/(Modelica.Math.log(1 + e/D_in));
              dW_fg[i, j] = 2*pi*K_cond_wall*dz*N_tubes*(T_wall[i, j] - T_wall_fg[i, j])/(Modelica.Math.log(1 + e/(e + D_in)));

            // Node energy balance
              // Water side
              dW_water[i, j] = Q_water/Rows*(h_water[i, j+1] - h_water[i, j]);
              // Flue gas side
              dW_fg[i, j] = Q_fg/N*(h_fg[i+1, j] - h_fg[i, j]);
              // Global with wall storage
              dW_water[i, j] + dW_fg[i, j] + dM_wall*Cp_wall*der(T_wall[i, j]) = 0;

            // Convection heat transfer coefficient calculation
              // Average velocities
              U_water_node[i, j] = Q_water/(rho_water_node[i, j]*Ac_water);
              // Prandtl number
              Pr_water[i, j] = Cp_water_node[i, j]*Mu_water_node[i, j]/k_water_node[i, j];
              // Reynold's number
              Re_water[i, j] = rho_water_node[i, j]*U_water_node[i, j]*D_in/Mu_water_node[i, j];
              // Nusselt number
              Nu_water[i, j] = K_conv_water[i, j]*D_in/k_water_node[i, j];
              // Convection correlation: Dittus-Boelter equation
              Nu_water[i, j] = 0.023*Re_water[i, j]^0.8*Pr_water[i, j]^0.4;

            // Convection heat transfer equations
              // Water side
              dW_water[i, j] = K_conv_water[i, j]*dA_water*(T_wall[i, j] - T_water_node[i, j]);
              // Flue gas side
              dW_fg[i, j] = K_conv_fg*(T_wall[i, j] - T_fg[i, j])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
      //         Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;
          end for;

      initial equation

        for i in 1:Rows loop
          for j in 1:N loop
          der(T_wall[i, j]) = 0;
          end for;
        end for;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end CrossCurrent_1NodePerRow_MonoPhasicHX_2NodesConduction;

      model CrossCurrent_1NodePerRow_MonoPhasicHX_1DConduction_radial "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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

        // ------ Discretization z axis ------
          parameter Integer N = 1;
          parameter Units.Mass dM_wall = M_wall/N/Rows "Tube mass of a single node";
          parameter Units.Area dA_water = A_water/N/Rows "Water side heat exchange surface of a single node";
          parameter Units.Area dA_fg_tubes = A_fg_tubes/N/Rows "Flue gas side heat exchange surface of a single node";
          parameter Units.Area dA_fin_cb = A_fin_cb/N/Rows;
          parameter Units.Area dA_fg_fin = A_fg_fins/N/Rows;
          parameter Units.Length dz = L/N;
          parameter Integer N_wall = 5;
          parameter Units.Length de_wall = e/N_wall;

        // ------ Fluids properties ------
          // State
          WaterSteamMedium.ThermodynamicState state_water[Rows, N+1] "Water side node boundary state";
          FlueGasesMedium.ThermodynamicState state_fg[Rows + 1, N] "Flue gas side node boundary states";
          // Enthalpy
          Units.SpecificEnthalpy h_water[Rows, N+1](each start=h_water_out_0) "Water specific enthalpy";
          Units.SpecificEnthalpy h_fg[Rows + 1, N] "Flue gas specific enthalpy";
          // Mass flow rate
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Pressure
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Density
          Units.Density rho_water[Rows, N+1] "Node boundary water density";
          Units.Density rho_water_node[Rows, N] "Node average water density";
          Units.Density rho_fg[Rows + 1, N] "Node boundary flue gas density";
          // Mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
          // Temperature
          Units.Temperature T_water[Rows, N+1] "Node boundary water temperature";
          Units.Temperature T_water_node[Rows, N] "Node average water temperature";
          Units.Temperature T_fg[Rows + 1, N] "Node boundary flue gas temperature";
          // Dynamic viscosities
          Modelica.Units.SI.DynamicViscosity Mu_water[Rows, N+1] "Node boundary water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_water_node[Rows, N] "Node average water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_fg[Rows + 1, N] "Node boundary flue gas dynamic viscosity";
          // Heat capacities Cp
          Units.HeatCapacity Cp_water[Rows, N+1] "Node boundary water Cp";
          Units.HeatCapacity Cp_water_node[Rows, N] "Node average water Cp";
          Units.HeatCapacity Cp_fg[Rows + 1, N] "Node boundary flue gas Cp";
          // Thermal conductivity
          Modelica.Units.SI.ThermalConductivity k_water[Rows, N+1] "Node boundary water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_water_node[Rows, N] "Node average water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_fg[Rows + 1, N] "Node boundary flue gas thermal conductivity";

        // ------ Conduction variables ------
          Units.Temperature T_wall[N_wall+1, Rows, N] "Node wall average temperature";

        // ------ Tubes configuration parameters ------
          Units.Velocity U_fg_face "Flue gas face velocity";
          Units.Velocity U_fg_max "Flue gas maximum velocity";
          Real Re_fg_max "Flue gas maximum Reynold's number";
          Real Pr_fg "Flue gas overall average Prandtl number";
          FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
          Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
          Real Nu_fg_avg "Flue gass overall average Nusselt number";
          Units.HeatExchangeCoefficient K_conv_fg(each start=K_conv_water_0) "Flue gase convection heat transfer coefficient"; //  = 76.83

        // ------ Heat transfer parameters ------
          // Average velocities
          Units.Velocity U_water_node[Rows, N] "Node average water velocity";
          // Prandtl Number
          Real Pr_water[Rows, N] "Node water Prandtl's number";
          // Reynold's Number
          Real Re_water[Rows, N] "Node water Reynold's number";
          // Nusselt Nymber
          Real Nu_water[Rows, N] "Node water Nusselt number";
          // Convection heat transfer coefficient
          Units.HeatExchangeCoefficient K_conv_water[Rows, N](each start=K_conv_water_0) "Water side convection heat transfer coefficient"; // = 2418
          // Discretized heat transfer power
          Units.Power dW_water[Rows, N] "Node water heat exchange";
          Units.Power dW_fg[Rows, N] "Node flue gas heat exchange";
          // Biot Number
          // Real Bi[N] "Node Biot number";

        // Parameters of interest
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";
          Units.Temperature T_water_avg "Water overall average temperature";
          //Units.Temperature T_fg_avg "Flue gas overall average temperature";
          Units.Temperature T_wall_avg "Wall overall average temperature";

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------

          // Outlet
          water_side.W = sum(dW_water);
          fg_side.W = sum(dW_fg);

          // Inlet
          for i in 1:Rows loop
            h_water[i, 1] = water_side.h_in;
            state_water[i, 1] = WaterSteamMedium.setState_phX(P_water, h_water[i, 1], Xi_water);
            T_water[i, 1] = WaterSteamMedium.temperature(state_water[i, 1]);
            rho_water[i, 1] = WaterSteamMedium.density(state_water[i, 1]);
            Mu_water[i, 1] = WaterSteamMedium.dynamicViscosity(state_water[i, 1]);
            Cp_water[i, 1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, 1]);
            k_water[i, 1] = WaterSteamMedium.thermalConductivity(state_water[i, 1]);
          end for;

          for j in 1:N loop
           h_fg[1,  j] = fg_side.h_in;
           T_fg[1,  j] = FlueGasesMedium.temperature(state_fg[1,  j]);
           state_fg[1,  j] = FlueGasesMedium.setState_phX(P_fg, h_fg[1,  j], Xi_fg);
           rho_fg[1,  j] = FlueGasesMedium.density(state_fg[1,  j]);
           Mu_fg[1,  j] = FlueGasesMedium.dynamicViscosity(state_fg[1,  j]);
           Cp_fg[1,  j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1,  j]);
           k_fg[1,  j] = FlueGasesMedium.thermalConductivity(state_fg[1,  j]);
          end for;

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
          U_fg_face = Q_fg/(rho_fg[1, 1]*Af_fg);
          // Flue gas maximum velocity
          if (Tubes_Config == 1) then
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          elseif (S_D < (S_T + D_out)/2) then
            U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
          else
            U_fg_max = S_T*U_fg_face/(S_T - D_out);
          end if;
          // Flue gas maximum Reynold's number
          Re_fg_max = rho_fg[1, 1]*U_fg_max*D_out/Mu_fg[1, 1];
          // Flue gas Prandtl number
          Pr_fg = Cp_fg[1, 1]*Mu_fg[1, 1]/k_fg[1, 1];
          // Flue gas Prandtl number at surface temperature
          state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
          Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
          // Convection coefficient calculation
          Nu_fg_avg = K_conv_fg*D_out/k_fg[1, 1];
          // Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);
          Nu_fg_avg = CorrelationConstants.ESCOA(Re_fg_max, Pr_fg, Rows, T_fg[1, 1], T_wall_avg, D_out, H_fin, e_fin, S_f, S_T, S_L);

        // ------ Parameters of interest ------
          // IN/OUT temperatures
          T_water_in = water_side.T_in;
          T_water_out = water_side.T_out;
          T_fg_in = fg_side.T_in;
          T_fg_out = fg_side.T_out;
          // Average Temperatures
          T_water_avg = sum(T_water_node)/(Rows*N);
          //T_fg_avg = sum(T_fg_node)/N;
          T_wall_avg =  sum(T_wall)/((N_wall+1)*Rows*N);

        // ------ Discretization computation loop ------
          for i in 1:Rows loop
              for j in 1:N loop
            // Fluids Properties
              // State
              state_water[i, j+1] = WaterSteamMedium.setState_phX(P_water, h_water[i, j+1], Xi_water);
              state_fg[i+1, j] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1, j], Xi_fg);
              // Temperature
              T_water[i, j+1] = WaterSteamMedium.temperature(state_water[i, j+1]);
              T_fg[i+1, j] = FlueGasesMedium.temperature(state_fg[i+1, j]);
              T_water_node[i, j] = 0.5*(T_water[i, j] + T_water[i, j+1]);
              // Density
              rho_water[i, j+1] = WaterSteamMedium.density(state_water[i, j+1]);
              rho_fg[i+1, j] = FlueGasesMedium.density(state_fg[i+1, j]);
              rho_water_node[i, j] = 0.5*(rho_water[i, j] + rho_water[i, j+1]);
              // Dynamic viscosity
              Mu_water[i, j+1] = WaterSteamMedium.dynamicViscosity(state_water[i, j+1]);
              Mu_water_node[i, j] = 0.5*(Mu_water[i, j] + Mu_water[i, j+1]);
              Mu_fg[i+1, j] = FlueGasesMedium.dynamicViscosity(state_fg[i+1, j]);
              // Specific heat capacities Cp
              Cp_water[i, j+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, j+1]);
              Cp_water_node[i, j] = 0.5*(Cp_water[i, j] + Cp_water[i, j+1]);
              Cp_fg[i+1, j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1, j]);
              // Thermal conductivity
              k_water[i, j+1] = WaterSteamMedium.thermalConductivity(state_water[i, j+1]);
              k_water_node[i, j] = 0.5*(k_water[i, j] + k_water[i, j+1]);
              k_fg[i+1, j] = FlueGasesMedium.thermalConductivity(state_fg[i+1, j]);

            // Conduction heat transfer
                - dW_water[i, j] + K_cond_wall*2*pi*dz*N_tubes*(T_wall[2, i, j] - T_wall[1, i, j])/(Modelica.Math.log((D_in + 2*de_wall)/D_in)) = dM_wall*Cp_wall/N_wall/2*der(T_wall[1, i, j]);
                - dW_fg[i, j] - K_cond_wall*2*pi*dz*N_tubes*(T_wall[N_wall+1, i, j] - T_wall[N_wall, i, j])/(Modelica.Math.log(D_out/(D_out - 2*de_wall))) = dM_wall*Cp_wall/N_wall/2*der(T_wall[N_wall+1, i, j]); // check sign

              // Conduction inner nodes
              for k in 2:N_wall loop
                K_cond_wall*2*pi*dz*N_tubes*(T_wall[k-1, i, j] - T_wall[k, i, j])/(Modelica.Math.log((D_in/2 + (k-1)*de_wall)/(D_in/2 + (k-2)*de_wall)))
                + K_cond_wall*2*pi*dz*N_tubes*(T_wall[k+1, i, j] - T_wall[k, i, j])/(Modelica.Math.log((D_in/2 + k*de_wall)/(D_in/2 + (k-1)*de_wall)))
                = dM_wall*Cp_wall/N_wall*der(T_wall[k, i, j]);
              end for;

            // Node energy balance
              // Water side
              dW_water[i, j] = Q_water/Rows*(h_water[i, j+1] - h_water[i, j]);
              // Flue gas side
              dW_fg[i, j] = Q_fg/N*(h_fg[i+1, j] - h_fg[i, j]);
              // Global with wall storage
              //dW_water[i, j] + dW_fg[i, j] + dM_wall*Cp_wall*der(T_wall[i, j]) = 0;

            // Convection heat transfer coefficient calculation
              // Average velocities
              U_water_node[i, j] = Q_water/(rho_water_node[i, j]*Ac_water);
              // Prandtl number
              Pr_water[i, j] = Cp_water_node[i, j]*Mu_water_node[i, j]/k_water_node[i, j];
              // Reynold's number
              Re_water[i, j] = rho_water_node[i, j]*U_water_node[i, j]*D_in/Mu_water_node[i, j];
              // Nusselt number
              Nu_water[i, j] = K_conv_water[i, j]*D_in/k_water_node[i, j];
              // Convection correlation: Dittus-Boelter equation
              Nu_water[i, j] = 0.023*Re_water[i, j]^0.8*Pr_water[i, j]^0.4;

            // Convection heat transfer equations
              // Water side
              dW_water[i, j] = K_conv_water[i, j]*dA_water*(T_wall[1, i, j] - T_water_node[i, j]);
              // Flue gas side
              dW_fg[i, j] = K_conv_fg*(T_wall[N_wall+1, i, j] - T_fg[i, j])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
      //         Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;
          end for;

      initial equation

        for i in 1:Rows loop
          for j in 1:N loop
            for k in 1:N_wall+1 loop
          der(T_wall[k, i, j]) = 0;
            end for;
          end for;
        end for;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end CrossCurrent_1NodePerRow_MonoPhasicHX_1DConduction_radial;

      annotation (Icon(graphics={
            Rectangle(
              lineColor={200,200,200},
              fillColor={248,248,248},
              fillPattern=FillPattern.HorizontalCylinder,
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Rectangle(
              lineColor={128,128,128},
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Polygon(
              points={{10,-60},{-10,-60},{-10,40},{-40,20},{0,80},{40,20},{10,40},{10,-60}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={28,108,200})}));
    end One_pass_HX;

    package Multiple_pass_HX
      model HX_2_pass
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;

        // Number of tubes per water flow direction
        parameter Integer N_tubes_row_dir_1 = 93 "Number of tubes of water flowing in in direction 1";
        parameter Integer N_tubes_row_dir_2 = 93 "Number of tubes of water flowing in in direction 2";

        // Indicators
          // Configuration
          Integer N_tubes_row_tot "Total number of tubes per row";
          Units.PositiveMassFlowRate Q_fg_dir_1 "Flue gas mass flow rate heating water in direction 1";
          Units.PositiveMassFlowRate Q_fg_dir_2 "Flue gas mass flow rate heating water in direction 2";
          Units.PositiveMassFlowRate Q_fg_tot "Total flue gas mass flow rate";
          // Temperatures
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";

        One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_2(
          N_tubes_row=N_tubes_row_dir_1,
          Rows=2,
          Tubes_Config=2,
          fg_path_width=14.07,
          N=10,
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          A_water=676.73035,
          T_wall_0=745.15) annotation (Placement(transformation(extent={{-50,-11},{-30,11}})));

        One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_1(
          N_tubes_row=N_tubes_row_dir_2,
          Rows=2,
          Tubes_Config=2,
          fg_path_width=14.07,
          N=10,
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          A_water=676.73035,
          T_wall_0=745.15) annotation (Placement(transformation(extent={{30,11},{50,-11}})));
        FlueGases.Pipes.PressureCut PC_1 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,20})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        WaterSteam.Connectors.Inlet inlet annotation (Placement(transformation(extent={{50,90},{70,110}}),     iconTransformation(extent={{50,90},{70,110}})));
        WaterSteam.Connectors.Outlet outlet annotation (Placement(transformation(extent={{-70,90},{-50,110}})));
        FlueGases.BaseClasses.IsoPHFlowModel fg_inlet_properties annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-70,0})));
        FlueGases.BaseClasses.IsoPHFlowModel fg_outlet_properties
                                                                 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={70,0})));
      equation

        // Indicators
          // Configuration
          N_tubes_row_tot = N_tubes_row_dir_1 + N_tubes_row_dir_2;
          Dir_1.fg_side.Q = Q_fg_dir_1;
          Dir_2.fg_side.Q =  Q_fg_dir_2;
          Q_fg_tot = Q_fg_dir_1 + Q_fg_dir_2;
          // Temperatures
          T_water_in = Dir_1.T_water_in;
          T_water_out = Dir_2.T_water_out;
          T_fg_in = fg_inlet_properties.T_in;
          T_fg_out = fg_outlet_properties.T_out;

        // Flow repartition is proportional to the number of tubes
        Q_fg_dir_1 = Q_fg_tot*N_tubes_row_dir_1/N_tubes_row_tot;

        connect(Dir_2.fg_outlet, PC_1.C_in) annotation (Line(points={{-36,0},{-5.55112e-16,0},{-5.55112e-16,10}}, color={95,95,95}));
        connect(fg_inlet, fg_inlet) annotation (Line(points={{-100,0},{-100,0}}, color={95,95,95}));
        connect(fg_inlet, fg_inlet_properties.C_in) annotation (Line(points={{-100,0},{-80,0}},           color={95,95,95}));
        connect(fg_inlet_properties.C_out, Dir_2.fg_inlet) annotation (Line(points={{-60,0},{-44,0}}, color={95,95,95}));
        connect(Dir_1.fg_inlet, Dir_2.fg_inlet) annotation (Line(points={{36,0},{20,0},{20,-40},{-52,-40},{-52,0},{-44,0}}, color={95,95,95}));
        connect(inlet, Dir_1.water_inlet) annotation (Line(points={{60,100},{60,60},{40,60},{40,11}}, color={28,108,200}));
        connect(Dir_2.water_outlet, outlet) annotation (Line(points={{-40,11},{-40,60},{-60,60},{-60,100}}, color={28,108,200}));
        connect(Dir_2.water_inlet, Dir_1.water_outlet) annotation (Line(points={{-40,-11},{-40,-20},{40,-20},{40,-11}}, color={28,108,200}));
        connect(PC_1.C_out, fg_outlet_properties.C_in) annotation (Line(points={{5.55112e-16,30},{5.55112e-16,36},{0,36},{0,40},{54,40},{54,0},{60,0}}, color={95,95,95}));
        connect(fg_outlet_properties.C_out, fg_outlet) annotation (Line(points={{80,0},{100,0}}, color={95,95,95}));
        connect(Dir_1.fg_outlet, fg_outlet_properties.C_in) annotation (Line(points={{44,0},{60,0}}, color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid),
              Ellipse(extent={{-72,70},{-72,70}}, lineColor={28,108,200}),
              Rectangle(
                extent={{-80,86},{80,80}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-80,-80},{80,-86}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Line(
                points={{0,86},{0,80}},
                color={0,0,0},
                thickness=1),
              Line(
                points={{60,80},{60,-82},{-60,-80},{-60,80}},
                color={28,108,200},
                smooth=Smooth.Bezier,
                thickness=1),
              Polygon(
                points={{-60,10},{-66,-10},{-54,-10},{-60,10}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{60,-10},{54,10},{66,10},{60,-10}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-92,18},{-72,-2}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled}),
              Line(
                points={{-50,18},{-30,-2}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled}),
              Line(
                points={{-10,18},{10,-2}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled}),
              Line(
                points={{30,18},{50,-2}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled}),
              Line(
                points={{70,18},{90,-2}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled})}),                    Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=500,
            __Dymola_NumberOfIntervals=1000,
            __Dymola_Algorithm="Dassl"));
      end HX_2_pass;

      model HX_3_pass
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;

        // Number of tubes per water flow direction
        parameter Integer N_tubes_row_dir_1 = 62 "Number of tubes of water flowing in in direction 1";
        parameter Integer N_tubes_row_dir_2 = 62 "Number of tubes of water flowing in in direction 2";
        parameter Integer N_tubes_row_dir_3 = 62 "Number of tubes of water flowing in in direction 2";

        // Indicators
          // Configuration
          Integer N_tubes_row_tot "Total number of tubes per row";
          Units.PositiveMassFlowRate Q_fg_dir_1 "Flue gas mass flow rate heating water in direction 1";
          Units.PositiveMassFlowRate Q_fg_dir_2 "Flue gas mass flow rate heating water in direction 2";
          Units.PositiveMassFlowRate Q_fg_dir_3 "Flue gas mass flow rate heating water in direction 3";
          Units.PositiveMassFlowRate Q_fg_tot "Total flue gas mass flow rate";
          // Temperatures
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";

        One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_1(
          N_tubes_row=N_tubes_row_dir_1,
          Rows=2,
          Tubes_Config=2,
          fg_path_width=14.07,
          N=10,
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          A_water=676.73035,
          T_wall_0=745.15) annotation (Placement(transformation(extent={{-50,-11},{-30,11}})));

        One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_2(
          N_tubes_row=N_tubes_row_dir_3,
          Rows=2,
          Tubes_Config=2,
          fg_path_width=14.07,
          N=10,
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          A_water=676.73035,
          T_wall_0=745.15) annotation (Placement(transformation(extent={{-10,11},{10,-11}})));
        One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_3(
          N_tubes_row=N_tubes_row_dir_2,
          Rows=2,
          Tubes_Config=2,
          fg_path_width=14.07,
          N=10,
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          A_water=676.73035,
          T_wall_0=745.15) annotation (Placement(transformation(extent={{30,-11},{50,11}})));
        FlueGases.Pipes.PressureCut PC_1 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-28,30})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        WaterSteam.Connectors.Inlet inlet annotation (Placement(transformation(extent={{-70,-110},{-50,-90}}), iconTransformation(extent={{-70,-110},{-50,-90}})));
        WaterSteam.Connectors.Outlet outlet annotation (Placement(transformation(extent={{50,90},{70,110}})));
        FlueGases.BaseClasses.IsoPHFlowModel fg_inlet_properties annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-70,0})));
        FlueGases.BaseClasses.IsoPHFlowModel fg_outlet_properties
                                                                 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={70,0})));
        FlueGases.Pipes.PressureCut PC_2 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={20,30})));
      equation

        // Indicators
          // Configuration
          N_tubes_row_tot = N_tubes_row_dir_1 + N_tubes_row_dir_2 + N_tubes_row_dir_3;
          Dir_1.fg_side.Q = Q_fg_dir_1;
          Dir_2.fg_side.Q = Q_fg_dir_2;
          Dir_3.fg_side.Q = Q_fg_dir_3;
          Q_fg_tot = Q_fg_dir_1 + Q_fg_dir_2 + Q_fg_dir_3;
          // Temperatures
          T_water_in = Dir_1.T_water_in;
          T_water_out = Dir_3.T_water_out;
          T_fg_in = fg_inlet_properties.T_in;
          T_fg_out = fg_outlet_properties.T_out;

        // Flow repartition is proportional to the number of tubes
        Q_fg_dir_1 = Q_fg_tot*N_tubes_row_dir_1/N_tubes_row_tot;
        Q_fg_dir_2 = Q_fg_tot*N_tubes_row_dir_2/N_tubes_row_tot;

        connect(fg_inlet, fg_inlet) annotation (Line(points={{-100,0},{-100,0}}, color={95,95,95}));
        connect(fg_inlet, fg_inlet_properties.C_in) annotation (Line(points={{-100,0},{-80,0}},           color={95,95,95}));
        connect(fg_inlet_properties.C_out, Dir_1.fg_inlet) annotation (Line(points={{-60,0},{-44,0}}, color={95,95,95}));
        connect(fg_outlet_properties.C_out, fg_outlet) annotation (Line(points={{80,0},{100,0}}, color={95,95,95}));
        connect(Dir_3.fg_outlet, fg_outlet_properties.C_in) annotation (Line(points={{44,0},{60,0}}, color={95,95,95}));
        connect(PC_1.C_in, Dir_1.fg_outlet) annotation (Line(points={{-28,20},{-28,0},{-36,0}}, color={95,95,95}));
        connect(PC_1.C_out, fg_outlet_properties.C_in) annotation (Line(points={{-28,40},{-28,60},{52,60},{52,0},{60,0}}, color={95,95,95}));
        connect(PC_2.C_out, fg_outlet_properties.C_in) annotation (Line(points={{20,40},{20,60},{52,60},{52,0},{60,0}}, color={95,95,95}));
        connect(PC_2.C_in,Dir_2. fg_outlet) annotation (Line(points={{20,20},{22,20},{22,0},{4,0}}, color={95,95,95}));
        connect(Dir_2.fg_inlet, Dir_1.fg_inlet) annotation (Line(points={{-4,0},{-20,0},{-20,-40},{-52,-40},{-52,0},{-44,0}}, color={95,95,95}));
        connect(Dir_3.fg_inlet, Dir_1.fg_inlet) annotation (Line(points={{36,0},{28,0},{28,-40},{-52,-40},{-52,0},{-44,0}}, color={95,95,95}));
        connect(Dir_1.water_inlet, inlet) annotation (Line(points={{-40,-11},{-40,-60},{-60,-60},{-60,-100}}, color={28,108,200}));
        connect(Dir_1.water_outlet, Dir_2.water_inlet) annotation (Line(points={{-40,11},{-40,80},{0,80},{0,11}}, color={28,108,200}));
        connect(Dir_2.water_outlet, Dir_3.water_inlet) annotation (Line(points={{0,-11},{0,-60},{40,-60},{40,-11}}, color={28,108,200}));
        connect(Dir_3.water_outlet, outlet) annotation (Line(points={{40,11},{40,80},{60,80},{60,100}}, color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid),
              Ellipse(extent={{-72,70},{-72,70}}, lineColor={28,108,200}),
              Rectangle(
                extent={{-80,86},{80,80}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-80,-80},{80,-86}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Line(
                points={{30,86},{30,80}},
                color={0,0,0},
                thickness=1),
              Line(points={{60,0},{60,80}},   color={28,108,200}),
              Line(
                points={{-30,-80},{-30,-86}},
                color={0,0,0},
                thickness=1),
              Line(
                points={{-60,-80},{-60,-62},{-60,80},{0,80},{0,-80},{60,-80},{60,80}},
                color={28,108,200},
                thickness=1,
                smooth=Smooth.Bezier),
              Polygon(
                points={{-60,8},{-66,-12},{-54,-12},{-60,8}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{0,-14},{-6,6},{6,6},{0,-14}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{60,8},{54,-12},{66,-12},{60,8}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-92,16},{-72,-4}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled}),
              Line(
                points={{-42,16},{-22,-4}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled}),
              Line(
                points={{20,16},{40,-4}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled}),
              Line(
                points={{70,18},{90,-2}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled})}),                    Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=500,
            __Dymola_NumberOfIntervals=1000,
            __Dymola_Algorithm="Dassl"));
      end HX_3_pass;

      model HX_4_pass
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;

        // Number of tubes per water flow direction
        parameter Integer N_tubes_row_dir_1 = 40 "Number of tubes of water flowing in in direction 1";
        parameter Integer N_tubes_row_dir_2 = 40 "Number of tubes of water flowing in in direction 2";
        parameter Integer N_tubes_row_dir_3 = 40 "Number of tubes of water flowing in in direction 3";
        parameter Integer N_tubes_row_dir_4 = 40 "Number of tubes of water flowing in in direction 4";

        // Indicators
          // Configuration
          Integer N_tubes_row_tot "Total number of tubes per row";
          Units.PositiveMassFlowRate Q_fg_dir_1 "Flue gas mass flow rate heating water in direction 1";
          Units.PositiveMassFlowRate Q_fg_dir_2 "Flue gas mass flow rate heating water in direction 2";
          Units.PositiveMassFlowRate Q_fg_dir_3 "Flue gas mass flow rate heating water in direction 3";
          Units.PositiveMassFlowRate Q_fg_dir_4 "Flue gas mass flow rate heating water in direction 4";
          Units.PositiveMassFlowRate Q_fg_tot "Total flue gas mass flow rate";
          // Temperatures
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";

        One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_1(
          N_tubes_row=N_tubes_row_dir_1,
          Rows=2,
          Tubes_Config=2,
          fg_path_width=14.07,
          N=10,
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          A_water=676.73035,
          T_wall_0=745.15) annotation (Placement(transformation(extent={{-70,11},{-50,-11}})));

        One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_2(
          N_tubes_row=N_tubes_row_dir_2,
          Rows=2,
          Tubes_Config=2,
          fg_path_width=14.07,
          N=10,
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          A_water=676.73035,
          T_wall_0=745.15) annotation (Placement(transformation(extent={{-30,-11},{-10,11}})));
        One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_3(
          N_tubes_row=N_tubes_row_dir_3,
          Rows=2,
          Tubes_Config=2,
          fg_path_width=14.07,
          N=10,
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          A_water=676.73035,
          T_wall_0=745.15) annotation (Placement(transformation(extent={{10,11},{30,-11}})));
        One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_4(
          N_tubes_row=N_tubes_row_dir_4,
          Rows=2,
          Tubes_Config=2,
          fg_path_width=14.07,
          N=10,
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          A_water=676.73035,
          T_wall_0=745.15) annotation (Placement(transformation(extent={{50,-11},{70,11}})));
        FlueGases.Pipes.PressureCut PC_1 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-46,20})));
        FlueGases.Pipes.PressureCut PC_2 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,20})));
        FlueGases.Pipes.PressureCut PC_3 annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={40,20})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{90,-10},{110,10}})));
        WaterSteam.Connectors.Inlet inlet annotation (Placement(transformation(extent={{-70,90},{-50,110}}),   iconTransformation(extent={{-70,90},{-50,110}})));
        WaterSteam.Connectors.Outlet outlet annotation (Placement(transformation(extent={{50,90},{70,110}})));
        FlueGases.BaseClasses.IsoPHFlowModel fg_inlet_properties annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={-80,-20})));
        FlueGases.BaseClasses.IsoPHFlowModel fg_outlet_properties
                                                                 annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=270,
              origin={80,-22})));
      equation

        // Indicators
          // Configuration
          N_tubes_row_tot = Dir_1.N_tubes_row + Dir_2.N_tubes_row + Dir_3.N_tubes_row + Dir_4.N_tubes_row;
          Dir_1.fg_side.Q =  Q_fg_dir_1;
          Dir_2.fg_side.Q =  Q_fg_dir_2;
          Dir_3.fg_side.Q =  Q_fg_dir_2;
          Dir_4.fg_side.Q =  Q_fg_dir_3;
          Q_fg_tot = Q_fg_dir_1 + Q_fg_dir_2 + Q_fg_dir_3 + Q_fg_dir_4;
          // Temperatures
          T_water_in = Dir_1.T_water_in;
          T_water_out = Dir_4.T_water_out;
          T_fg_in = fg_inlet_properties.T_in;
          T_fg_out = fg_outlet_properties.T_out;

        // Flow repartition is proportional to the number of tubes
        Q_fg_dir_1 = Q_fg_tot*Dir_1.N_tubes_row/N_tubes_row_tot;
        Q_fg_dir_2 = Q_fg_tot*Dir_2.N_tubes_row/N_tubes_row_tot;
        Q_fg_dir_3 = Q_fg_tot*Dir_3.N_tubes_row/N_tubes_row_tot;

        connect(Dir_1.fg_outlet, PC_1.C_in) annotation (Line(points={{-56,0},{-46,0},{-46,10}}, color={95,95,95}));
        connect(Dir_2.fg_outlet, PC_2.C_in) annotation (Line(points={{-16,0},{0,0},{0,10}}, color={95,95,95}));
        connect(Dir_3.fg_outlet, PC_3.C_in) annotation (Line(points={{24,0},{40,0},{40,10}}, color={95,95,95}));
        connect(fg_inlet, fg_inlet) annotation (Line(points={{-100,0},{-100,0}}, color={95,95,95}));
        connect(fg_inlet, fg_inlet_properties.C_in) annotation (Line(points={{-100,0},{-80,0},{-80,-10}}, color={95,95,95}));
        connect(fg_inlet_properties.C_out, Dir_4.fg_inlet) annotation (Line(points={{-80,-30},{-80,-40},{50,-40},{50,0},{56,0}}, color={95,95,95}));
        connect(Dir_1.fg_inlet, Dir_4.fg_inlet) annotation (Line(points={{-64,0},{-70,0},{-70,-40},{50,-40},{50,0},{56,0}}, color={95,95,95}));
        connect(Dir_2.fg_inlet, Dir_4.fg_inlet) annotation (Line(points={{-24,0},{-30,0},{-30,-40},{50,-40},{50,0},{56,0}}, color={95,95,95}));
        connect(Dir_3.fg_inlet, Dir_4.fg_inlet) annotation (Line(points={{16,0},{10,0},{10,-40},{50,-40},{50,0},{56,0}}, color={95,95,95}));
        connect(Dir_4.water_outlet, outlet) annotation (Line(points={{60,11},{60,100},{60,100}}, color={28,108,200}));
        connect(Dir_1.water_inlet, inlet) annotation (Line(points={{-60,11},{-60,100},{-60,100}}, color={28,108,200}));
        connect(Dir_1.water_outlet, Dir_2.water_inlet) annotation (Line(points={{-60,-11},{-60,-60},{-20,-60},{-20,-11}}, color={28,108,200}));
        connect(Dir_2.water_outlet, Dir_3.water_inlet) annotation (Line(points={{-20,11},{-20,60},{20,60},{20,11}}, color={28,108,200}));
        connect(Dir_3.water_outlet, Dir_4.water_inlet) annotation (Line(points={{20,-11},{20,-60},{60,-60},{60,-11}}, color={28,108,200}));
        connect(Dir_4.fg_outlet, fg_outlet_properties.C_in) annotation (Line(points={{64,0},{72,0},{72,-40},{80,-40},{80,-32}}, color={95,95,95}));
        connect(PC_1.C_out, fg_outlet_properties.C_in) annotation (Line(points={{-46,30},{-46,40},{72,40},{72,-40},{80,-40},{80,-32}}, color={95,95,95}));
        connect(PC_2.C_out, fg_outlet_properties.C_in) annotation (Line(points={{0,30},{0,40},{72,40},{72,-40},{80,-40},{80,-32}}, color={95,95,95}));
        connect(PC_3.C_out, fg_outlet_properties.C_in) annotation (Line(points={{40,30},{40,40},{72,40},{72,-40},{80,-40},{80,-32}}, color={95,95,95}));
        connect(fg_outlet_properties.C_out, fg_outlet) annotation (Line(points={{80,-12},{80,0},{100,0}}, color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid),
              Ellipse(extent={{-72,70},{-72,70}}, lineColor={28,108,200}),
              Rectangle(
                extent={{-80,86},{80,80}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-80,-80},{80,-86}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-40,86},{-40,80}},
                color={0,0,0},
                thickness=1),
              Line(
                points={{40,86},{40,80}},
                color={0,0,0},
                thickness=1),
              Line(
                points={{0,-80},{0,-86}},
                color={0,0,0},
                thickness=1),
              Line(
                points={{-60,80},{-60,-82},{-20,-80},{-20,80},{20,80},{20,-80},{60,-80},{60,80}},
                color={28,108,200},
                thickness=1,
                smooth=Smooth.Bezier),
              Polygon(
                points={{-60,-10},{-66,10},{-54,10},{-60,-10}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-20,12},{-26,-8},{-14,-8},{-20,12}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{20,-10},{14,10},{26,10},{20,-10}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{60,10},{54,-10},{66,-10},{60,10}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Line(
                points={{30,20},{50,0}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled}),
              Line(
                points={{-10,20},{10,0}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled}),
              Line(
                points={{-50,20},{-30,0}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled}),
              Line(
                points={{-92,20},{-72,0}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled}),
              Line(
                points={{70,20},{90,0}},
                color={0,0,0},
                thickness=1,
                arrow={Arrow.None,Arrow.Filled})}),                    Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=500,
            __Dymola_NumberOfIntervals=1000,
            __Dymola_Algorithm="Dassl"));
      end HX_4_pass;

      annotation (Icon(graphics={
            Rectangle(
              lineColor={200,200,200},
              fillColor={248,248,248},
              fillPattern=FillPattern.HorizontalCylinder,
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Rectangle(
              lineColor={128,128,128},
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Polygon(
              points={{-40,-60},{-60,-60},{-60,40},{-90,20},{-50,80},{-10,20},{-40,40},{-40,-60}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={28,108,200}),
            Polygon(
              points={{60,80},{40,80},{40,-20},{10,0},{50,-60},{90,0},{60,-20},{60,80}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={28,108,200})}));
    end Multiple_pass_HX;

    package Simplified
      model Base_model_OHTC "Added more info on fins and used ESCOA correlation"
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
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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
          FlueGasesMedium.ThermodynamicState state_fg[2, N] "Flue gas side node boundary states";
          // Enthalpy
          Units.SpecificEnthalpy h_water[N+1](each start=h_water_out_0) "Water specific enthalpy";
          Units.SpecificEnthalpy h_fg[2, N] "Flue gas specific enthalpy";
          // Mass flow rate
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Pressure
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Density
          Units.Density rho_water[N+1] "Node boundary water density";
          Units.Density rho_water_node[N] "Node average water density";
          Units.Density rho_fg[2, N] "Node boundary flue gas density";
          // Mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
          // Temperature
          Units.Temperature T_water[N+1] "Node boundary water temperature";
          Units.Temperature T_water_node[N] "Node average water temperature";
          Units.Temperature T_fg[2, N] "Node boundary flue gas temperature";
          // Dynamic viscosities
          Modelica.Units.SI.DynamicViscosity Mu_water[N+1] "Node boundary water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_water_node[N] "Node average water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_fg[2, N] "Node boundary flue gas dynamic viscosity";
          // Heat capacities Cp
          Units.HeatCapacity Cp_water[N+1] "Node boundary water Cp";
          Units.HeatCapacity Cp_water_node[N] "Node average water Cp";
          Units.HeatCapacity Cp_fg[2, N] "Node boundary flue gas Cp";
          // Thermal conductivity
          Modelica.Units.SI.ThermalConductivity k_water[N+1] "Node boundary water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_water_node[N] "Node average water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_fg[2, N] "Node boundary flue gas thermal conductivity";

        // ------ Conduction variables ------
          Units.Temperature T_wall[N] "Node wall average temperature";

        // ------ Tubes configuration parameters ------
      //     Units.Velocity U_fg_face "Flue gas face velocity";
      //     Units.Velocity U_fg_max "Flue gas maximum velocity";
      //     Real Re_fg_max "Flue gas maximum Reynold's number";
      //     Real Pr_fg "Flue gas overall average Prandtl number";
      //     FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
      //     Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
      //     Real Nu_fg_avg "Flue gass overall average Nusselt number";
          parameter Units.HeatExchangeCoefficient K_conv_fg = 76.83 "Flue gase convection heat transfer coefficient"; //  = 82.06

        // ------ Heat transfer parameters ------
      //     // Average velocities
      //     Units.Velocity U_water_node[N] "Node average water velocity";
      //     // Prandtl Number
      //     Real Pr_water[N] "Node water Prandtl's number";
      //     // Reynold's Number
      //     Real Re_water[N] "Node water Reynold's number";
      //     // Nusselt Nymber
      //     Real Nu_water[N] "Node water Nusselt number";
          // Convection heat transfer coefficient
          parameter Units.HeatExchangeCoefficient K_conv_water = 2418 "Water side convection heat transfer coefficient";
          // Discretized heat transfer power
          Units.Power dW_water[N] "Node water heat exchange";
          Units.Power dW_fg[N] "Node flue gas heat exchange";
          // Biot Number
          // Real Bi[N] "Node Biot number";

        // Parameters of interest
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";
          Units.Temperature T_water_avg "Water overall average temperature";
          //Units.Temperature T_fg_avg "Flue gas overall average temperature";
          Units.Temperature T_wall_avg "Wall overall average temperature";
      //     Units.Velocity U_water_avg "Water overall average velocity";
      //     Real Pr_water_avg "Water overall average Prandtl number";
      //     Real Re_water_avg "Water overall average Reynold's number";
      //     Real Nu_water_avg "Water overall average Nusselt number";
      //     Real K_conv_water_avg "Water side overall average convection heat transfer coefficient";
      //     Real Bi_max "Maximum Biot number";

          Real UA "Overall heat transfer coefficient and exchange surface product";

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------
          // Enthalpy
          h_water[1] = water_side.h_in;
          h_water[N+1] = water_side.h_out;
          //sum(h_fg[2, N]) = fg_side.h_out;
          fg_side.W = sum(dW_fg);

          for i in 1:N loop
           h_fg[1, i] = fg_side.h_in;
           T_fg[1, i] = FlueGasesMedium.temperature(state_fg[1, i]);
           state_fg[1, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[1, i], Xi_fg);
           rho_fg[1, i] = FlueGasesMedium.density(state_fg[1, i]);
           Mu_fg[1, i] = FlueGasesMedium.dynamicViscosity(state_fg[1, i]);
           Cp_fg[1, i] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1, i]);
           k_fg[1, i] = FlueGasesMedium.thermalConductivity(state_fg[1, i]);
          end for;
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
      //     U_fg_face = Q_fg/(rho_fg[N+1]*Af_fg);
      //     // Flue gas maximum velocity
      //     if (Tubes_Config == 1) then
      //       U_fg_max = S_T*U_fg_face/(S_T - D_out);
      //     elseif (S_D < (S_T + D_out)/2) then
      //       U_fg_max = S_T*U_fg_face/(2*(S_D - D_out));
      //     else
      //       U_fg_max = S_T*U_fg_face/(S_T - D_out);
      //     end if;
      //     // Flue gas maximum Reynold's number
      //     Re_fg_max = rho_fg[N+1]*U_fg_max*D_out/Mu_fg[N+1];
      //     // Flue gas Prandtl number
      //     Pr_fg = Cp_fg[N+1]*Mu_fg[N+1]/k_fg[N+1];
      //     // Flue gas Prandtl number at surface temperature
      //     state_fg_s = FlueGasesMedium.setState_pTX(P_fg, T_wall_avg, Xi_fg);
      //     Pr_fg_s = FlueGasesMedium.specificHeatCapacityCp(state_fg_s)*FlueGasesMedium.dynamicViscosity(state_fg_s)/FlueGasesMedium.thermalConductivity(state_fg_s);
      //     // Convection coefficient calculation
      //     Nu_fg_avg = K_conv_fg*D_out/k_fg[N+1];
      //     // Nu_fg_avg = CorrelationConstants.Zukauskas(Re_fg_max, Pr_fg, Pr_fg_s, Tubes_Config, Rows, S_T, S_L);
      //     Nu_fg_avg = CorrelationConstants.ESCOA(Re_fg_max, Pr_fg, Rows, T_fg_avg, T_wall_avg, D_out, H_fin, e_fin, S_f, S_T, S_L);

        // ------ First node properties ------
          state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);

          T_water[1] = WaterSteamMedium.temperature(state_water[1]);
          rho_water[1] = WaterSteamMedium.density(state_water[1]);
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
          //T_fg_avg = sum(T_fg_node)/N;
          T_wall_avg =  sum(T_wall)/N;
          // Average velocity
      //     U_water_avg = sum(U_water_node)/N;
      //     // Average heat transfer parameters
      //     Pr_water_avg = sum(Pr_water)/N;
      //     Re_water_avg = sum(Re_water)/N;
      //     Nu_water_avg = sum(Nu_water)/N;
      //     K_conv_water_avg = sum(K_conv_water)/N;
      //     // Biot number and validity of LCM verification
      //     Bi_max = max(Bi);
      //     assert(Bi_max < 0.1, "LCM is not valid",  AssertionLevel.warning);

          // Overall heat transfer coefficient and exchange surface product
          1/UA = 1/(A_water*K_conv_water) + 1/(A_fg_tubes*K_conv_fg) + 1/(A_fg_fins*eff_fins*K_conv_fg);

        // ------ Discretization computation loop ------
          for i in 1:N loop

            // Fluids Properties
              // State
              state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
              state_fg[2, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[2, i], Xi_fg);
              // Temperature
              T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
              T_fg[2, i] = FlueGasesMedium.temperature(state_fg[2, i]);
              T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
              // Density
              rho_water[i+1] = WaterSteamMedium.density(state_water[i+1]);
              rho_fg[2, i] = FlueGasesMedium.density(state_fg[2, i]);
              rho_water_node[i] = 0.5*(rho_water[i] + rho_water[i+1]);
              // Dynamic viscosity
              Mu_water[i+1] = WaterSteamMedium.dynamicViscosity(state_water[i+1]);
              Mu_water_node[i] = 0.5*(Mu_water[i] + Mu_water[i+1]);
              Mu_fg[2, i] = FlueGasesMedium.dynamicViscosity(state_fg[2, i]);
              // Specific heat capacities Cp
              Cp_water[i+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i+1]);
              Cp_water_node[i] = 0.5*(Cp_water[i] + Cp_water[i+1]);
              Cp_fg[2, i] = FlueGasesMedium.specificHeatCapacityCp(state_fg[2, i]);
              // Thermal conductivity
              k_water[i+1] = WaterSteamMedium.thermalConductivity(state_water[i+1]);
              k_water_node[i] = 0.5*(k_water[i] + k_water[i+1]);
              k_fg[2, i] = FlueGasesMedium.thermalConductivity(state_fg[2, i]);

            // Node energy balance
              // Water side
              dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
              // Flue gas side
              dW_fg[i] = Q_fg/N*(h_fg[2, i] - h_fg[1, i]);
              // Global with wall storage
              dW_water[i] + dW_fg[i] + dM_wall*Cp_wall*der(T_wall[i]) = 0;

      //       // Convection heat transfer coefficient calculation
      //         // Average velocities
      //         U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
      //         // Prandtl number
      //         Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
      //         // Reynold's number
      //         Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
      //         // Nusselt number
      //         Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
      //         // Convection correlation: Dittus-Boelter equation
      //         Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

            // Convection heat transfer equations
              // Water side
              dW_water[i] = K_conv_water*dA_water*(T_wall[i] - T_water_node[i]);
              // Flue gas side
              dW_fg[i] = K_conv_fg*(T_wall[i] - T_fg[1, i])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
      //         Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;

      initial equation

        for i in 1:N loop
          der(T_wall[i]) = 0;
        end for;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end Base_model_OHTC;

      model Reference_Model "Added more info on fins and used ESCOA correlation"
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
          parameter Real eff_fins = 0.77 "Fins efficiency";
          // Water side
          parameter Units.Area A_water = N_tubes*L*pi*D_in "Water side heat exchange surface";
          parameter Units.Area Ac_water = 0.25*pi*D_in^2*N_tubes_row "Water side cross sectional area";
          // Flue gas side
          parameter Units.Area A_fin_cb = pi*D_out*e_fin*N_fins "Fins cross-sectional area at the base";
          parameter Units.Area A_fg_tubes = N_tubes*L*pi*D_out - A_fin_cb "Outer tubes surface";
          parameter Units.Area A_fg_fins = 0.25*pi*(D_fin^2 - D_out^2)*2*N_fins "Fins outer surface";
          parameter Units.Area Af_fg = L*fg_path_width "Flue gas frontal area";

          // Wall
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
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

        // ------ Discretization z axis ------
          parameter Integer N = 1;
          parameter Units.Mass dM_wall = M_wall/N/Rows "Tube mass of a single node";
          parameter Units.Area dA_water = A_water/N/Rows "Water side heat exchange surface of a single node";
          parameter Units.Area dA_fg_tubes = A_fg_tubes/N/Rows "Flue gas side heat exchange surface of a single node";
          parameter Units.Area dA_fin_cb = A_fin_cb/N/Rows;
          parameter Units.Area dA_fg_fin = A_fg_fins/N/Rows;
          parameter Units.Length dz = L/N;

        // ------ Fluids properties ------
          // State
          WaterSteamMedium.ThermodynamicState state_water[Rows, N+1] "Water side node boundary state";
          FlueGasesMedium.ThermodynamicState state_fg[Rows + 1, N] "Flue gas side node boundary states";
          // Enthalpy
          Units.SpecificEnthalpy h_water[Rows, N+1](each start=h_water_out_0) "Water specific enthalpy";
          Units.SpecificEnthalpy h_fg[Rows + 1, N] "Flue gas specific enthalpy";
          // Mass flow rate
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Pressure
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Density
          Units.Density rho_water[Rows, N+1] "Node boundary water density";
          Units.Density rho_water_node[Rows, N] "Node average water density";
          Units.Density rho_fg[Rows + 1, N] "Node boundary flue gas density";
          // Mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
          // Temperature
          Units.Temperature T_water[Rows, N+1] "Node boundary water temperature";
          Units.Temperature T_water_node[Rows, N] "Node average water temperature";
          Units.Temperature T_fg[Rows + 1, N] "Node boundary flue gas temperature";
          // Dynamic viscosities
          Modelica.Units.SI.DynamicViscosity Mu_water[Rows, N+1] "Node boundary water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_water_node[Rows, N] "Node average water dynamic viscosity";
          Modelica.Units.SI.DynamicViscosity Mu_fg[Rows + 1, N] "Node boundary flue gas dynamic viscosity";
          // Heat capacities Cp
          Units.HeatCapacity Cp_water[Rows, N+1] "Node boundary water Cp";
          Units.HeatCapacity Cp_water_node[Rows, N] "Node average water Cp";
          Units.HeatCapacity Cp_fg[Rows + 1, N] "Node boundary flue gas Cp";
          // Thermal conductivity
          Modelica.Units.SI.ThermalConductivity k_water[Rows, N+1] "Node boundary water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_water_node[Rows, N] "Node average water thermal conductivity";
          Modelica.Units.SI.ThermalConductivity k_fg[Rows + 1, N] "Node boundary flue gas thermal conductivity";

        // ------ Conduction variables ------
          Units.Temperature T_wall[Rows, N] "Node wall average temperature";

          // Convection heat transfer coefficient
          parameter Units.HeatExchangeCoefficient K_conv_water = 2418 "Water side convection heat transfer coefficient";
          parameter Units.HeatExchangeCoefficient K_conv_fg = 76.83 "Flue gase convection heat transfer coefficient"; //  = 82.06
          Real UA "Overall heat transfer coefficient and exchange surface product";
          Real UA_water "Heat transfer coefficient from the water side";
          Real UA_fg "Heat transfer coefficient from the flue gas side";
          Real r_UA "Ratio of water to flue gas heat transfer coefficients";

          // Discretized heat transfer power
          Units.Power dW_water[Rows, N] "Node water heat exchange";
          Units.Power dW_fg[Rows, N] "Node flue gas heat exchange";
          // Biot Number
          // Real Bi[N] "Node Biot number";

        // Parameters of interest
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";
          Units.Temperature T_water_avg "Water overall average temperature";
          //Units.Temperature T_fg_avg "Flue gas overall average temperature";
          Units.Temperature T_wall_avg "Wall overall average temperature";

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------

          // Outlet
          water_side.W = sum(dW_water);
          fg_side.W = sum(dW_fg);

          // Inlet
          for i in 1:Rows loop
            h_water[i, 1] = water_side.h_in;
            state_water[i, 1] = WaterSteamMedium.setState_phX(P_water, h_water[i, 1], Xi_water);
            T_water[i, 1] = WaterSteamMedium.temperature(state_water[i, 1]);
            rho_water[i, 1] = WaterSteamMedium.density(state_water[i, 1]);
            Mu_water[i, 1] = WaterSteamMedium.dynamicViscosity(state_water[i, 1]);
            Cp_water[i, 1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, 1]);
            k_water[i, 1] = WaterSteamMedium.thermalConductivity(state_water[i, 1]);
          end for;

          for j in 1:N loop
           h_fg[1,  j] = fg_side.h_in;
           T_fg[1,  j] = FlueGasesMedium.temperature(state_fg[1,  j]);
           state_fg[1,  j] = FlueGasesMedium.setState_phX(P_fg, h_fg[1,  j], Xi_fg);
           rho_fg[1,  j] = FlueGasesMedium.density(state_fg[1,  j]);
           Mu_fg[1,  j] = FlueGasesMedium.dynamicViscosity(state_fg[1,  j]);
           Cp_fg[1,  j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[1,  j]);
           k_fg[1,  j] = FlueGasesMedium.thermalConductivity(state_fg[1,  j]);
          end for;

          // Pressure
          P_water = water_side.P_in;
          P_fg = fg_side.P_in;
          // Mass flow rate
          Q_water = water_side.Q;
          Q_fg = fg_side.Q;
          // Mass Fraction
          Xi_water = water_side.Xi;
          Xi_fg = fg_side.Xi;

        // ------ Parameters of interest ------
          // IN/OUT temperatures
          T_water_in = water_side.T_in;
          T_water_out = water_side.T_out;
          T_fg_in = fg_side.T_in;
          T_fg_out = fg_side.T_out;
          // Average Temperatures
          T_water_avg = sum(T_water_node)/(N*Rows);
          //T_fg_avg = sum(T_fg_node)/((N*Rows);
          T_wall_avg =  sum(T_wall)/(N*Rows);

          // Overall heat transfer coefficient and exchange surface product
          1/UA = 1/(A_water*K_conv_water) + 1/(A_fg_tubes*K_conv_fg + A_fg_fins*eff_fins*K_conv_fg);
          1/UA_water = 1/(A_water*K_conv_water);
          1/UA_fg = 1/(A_fg_tubes*K_conv_fg + A_fg_fins*eff_fins*K_conv_fg);
          r_UA = UA_water/UA_fg;

        // ------ Discretization computation loop ------
          for i in 1:Rows loop
              for j in 1:N loop
            // Fluids Properties
              // State
              state_water[i, j+1] = WaterSteamMedium.setState_phX(P_water, h_water[i, j+1], Xi_water);
              state_fg[i+1, j] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1, j], Xi_fg);
              // Temperature
              T_water[i, j+1] = WaterSteamMedium.temperature(state_water[i, j+1]);
              T_fg[i+1, j] = FlueGasesMedium.temperature(state_fg[i+1, j]);
              T_water_node[i, j] = 0.5*(T_water[i, j] + T_water[i, j+1]);
              // Density
              rho_water[i, j+1] = WaterSteamMedium.density(state_water[i, j+1]);
              rho_fg[i+1, j] = FlueGasesMedium.density(state_fg[i+1, j]);
              rho_water_node[i, j] = 0.5*(rho_water[i, j] + rho_water[i, j+1]);
              // Dynamic viscosity
              Mu_water[i, j+1] = WaterSteamMedium.dynamicViscosity(state_water[i, j+1]);
              Mu_water_node[i, j] = 0.5*(Mu_water[i, j] + Mu_water[i, j+1]);
              Mu_fg[i+1, j] = FlueGasesMedium.dynamicViscosity(state_fg[i+1, j]);
              // Specific heat capacities Cp
              Cp_water[i, j+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, j+1]);
              Cp_water_node[i, j] = 0.5*(Cp_water[i, j] + Cp_water[i, j+1]);
              Cp_fg[i+1, j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1, j]);
              // Thermal conductivity
              k_water[i, j+1] = WaterSteamMedium.thermalConductivity(state_water[i, j+1]);
              k_water_node[i, j] = 0.5*(k_water[i, j] + k_water[i, j+1]);
              k_fg[i+1, j] = FlueGasesMedium.thermalConductivity(state_fg[i+1, j]);

            // Node energy balance
              // Water side
              dW_water[i, j] = Q_water/Rows*(h_water[i, j+1] - h_water[i, j]);
              // Flue gas side
              dW_fg[i, j] = Q_fg/N*(h_fg[i+1, j] - h_fg[i, j]);
              // Global with wall storage
              dW_water[i, j] + dW_fg[i, j] + dM_wall*Cp_wall*der(T_wall[i, j]) = 0;

      //       // Convection heat transfer coefficient calculation
      //         // Average velocities
      //         U_water_node[i] = Q_water/(rho_water_node[i]*Ac_water);
      //         // Prandtl number
      //         Pr_water[i] = Cp_water_node[i]*Mu_water_node[i]/k_water_node[i];
      //         // Reynold's number
      //         Re_water[i] = rho_water_node[i]*U_water_node[i]*D_in/Mu_water_node[i];
      //         // Nusselt number
      //         Nu_water[i] = K_conv_water[i]*D_in/k_water_node[i];
      //         // Convection correlation: Dittus-Boelter equation
      //         Nu_water[i] = 0.023*Re_water[i]^0.8*Pr_water[i]^0.4;

            // Convection heat transfer equations
              // Water side
              dW_water[i, j] = K_conv_water*dA_water*(T_wall[i, j] - T_water_node[i, j]);
              // Flue gas side
              dW_fg[i, j] = K_conv_fg*(T_wall[i, j] - T_fg[i, j])*(dA_fg_tubes + eff_fins*dA_fg_fin);

            // Biot number
      //         Bi[i] = max(K_conv_water[i], K_conv_fg)*e/K_cond_wall;

          end for;
          end for;

      initial equation

        for i in 1:Rows loop
          for j in 1:N loop
          der(T_wall[i, j]) = 0;
          end for;
        end for;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end Reference_Model;

      model Simplified_model "Added more info on fins and used ESCOA correlation"
       import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;


          parameter Modelica.Units.SI.ThermalConductance UA = 264900 "W/K"; // = 264900

          // Real UA "Overall heat transfer coefficient and exchange surface product";
          // Temperatures
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out "Flue gas outlet temperature";
          // Pressures
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Heat exchanged
          Units.Power W_water;
          Units.Power W_fg;

          Units.Temperature DT_LMTD;

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


        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0), h_out(start=h_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0), h_out(start=h_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        // ------ Boundaries ------
          // Enthalpy
          T_water_in = water_side.T_in;
          T_water_out = water_side.T_out;
          T_fg_in = fg_side.T_in;
          T_fg_out = fg_side.T_out;
          P_water = water_side.P_in;
          P_fg = fg_side.P_in;
          // Mass flow rate
          Q_water = water_side.Q;
          Q_fg = fg_side.Q;
          // Mass Fraction
          water_side.W = W_water;
          fg_side.W = W_fg;

        // Energy Balance
          W_water = -W_fg;
          DT_LMTD = ((T_fg_out - T_water_in) - (T_fg_in - T_water_out))/log((T_fg_out - T_water_in)/(T_fg_in - T_water_out));
          W_water = UA*DT_LMTD;



        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end Simplified_model;

      model Simplified_model_1 "Added more info on fins and used ESCOA correlation"
       import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants = MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

          Modelica.Units.SI.ThermalConductance UA_fg(start=171696.64);
          Modelica.Units.SI.ThermalConductance UA_water(start=1476005.2);
          Modelica.Units.SI.ThermalConductance UA;
          parameter Real r_UA = 39.677;

          // Wall
          parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
          parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";


          // Real UA "Overall heat transfer coefficient and exchange surface product";
          // Temperatures
          Units.Temperature T_water_in "Water inlet temperature";
          Units.Temperature T_water_out(start=T_water_out_0) "Water outlet temperature";
          Units.Temperature T_fg_in "Flue gas inlet temperature";
          Units.Temperature T_fg_out(start=T_fg_out_0) "Flue gas outlet temperature";
          Units.Temperature T_wall(start=T_wall_0) "Tubes wall temperature";
          // Pressures
          Units.Pressure P_water(start=P_water_0) "Water Pressure";
          Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
          Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
          // Heat exchanged
          Units.Power W_water;
          Units.Power W_fg;

          Units.Temperature DT_LMTD_water(start=DT_LMTD_water_0);
          Units.Temperature DT_LMTD_fg(start=DT_LMTD_fg_0);

          // ------ Initialization ------
          parameter Units.Temperature T_wall_0 = 517.5455 + 273.15;
          parameter Units.Pressure P_water_0 = 70e5;
          parameter Units.Pressure P_fg_0 = 1e5;
          parameter Units.PositiveMassFlowRate Q_water_0 = 85;
          parameter Units.PositiveMassFlowRate Q_fg_0 = 640;
          parameter Units.Temperature T_water_out_0 = 560 + 273.15;
          parameter Units.Temperature T_fg_out_0 = 614 + 273.15;
          parameter Units.SpecificEnthalpy h_water_out_0 = 3354324.5;
          parameter Units.SpecificEnthalpy h_fg_out_0 = 912869.94;
          parameter Units.HeatExchangeCoefficient K_conv_water_0 = 2400;
          parameter Units.Temperature DT_LMTD_water_0 = 56 + 273.15;
          parameter Units.Temperature DT_LMTD_fg_0 = 32 + 273.15;

        WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{-10,-110},{10,-90}}),
                                                                                                                iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
        FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{30,-10},{50,10}}),  iconTransformation(extent={{30,-10},{50,10}})));
        FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-50,-10},{-30,10}}),  iconTransformation(extent={{-50,-10},{-30,10}})));
        WaterSteam.BaseClasses.IsoPFlowModel water_side(T_out(start=T_water_out_0)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,20})));
        FlueGases.BaseClasses.IsoPFlowModel  fg_side(T_out(start=T_fg_out_0)) annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
      equation

        r_UA = UA_water/UA_fg;
        1/UA = 1/UA_water + 1/UA_fg;

        // ------ Boundaries ------
          // Enthalpy
          T_water_in = water_side.T_in;
          T_water_out = water_side.T_out;
          T_fg_in = fg_side.T_in;
          T_fg_out = fg_side.T_out;
          P_water = water_side.P_in;
          P_fg = fg_side.P_in;
          // Mass flow rate
          Q_water = water_side.Q;
          Q_fg = fg_side.Q;
          // Mass Fraction
          water_side.W = W_water;
          fg_side.W = W_fg;

        // Energy Balance
          W_water + W_fg + M_wall*Cp_wall*der(T_wall) = 0;
      //     DT_LMTD_water = ((T_wall - T_water_in) - (T_wall - T_water_out))/log((T_wall - T_water_in)/(T_wall - T_water_out));
      //     DT_LMTD_fg = ((T_fg_in - T_wall) - (T_fg_out - T_wall))/log((T_fg_in - T_wall)/(T_fg_out - T_wall));
          DT_LMTD_water = T_wall - (T_water_in + T_water_out)/2;
          DT_LMTD_fg = (T_fg_in + T_fg_out)/2 - T_wall;
          W_water = UA_water*DT_LMTD_water;
          W_fg = - UA_fg*DT_LMTD_fg;

      initial equation
        der(T_wall) = 0;

      equation
        connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                         color={28,108,200}));
        connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                             color={28,108,200}));
        connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                       color={95,95,95}));
        connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                       color={95,95,95}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={-30,0},
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={-15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                rotation=90),
              Rectangle(
                extent={{-100,5},{100,-5}},
                lineColor={0,0,0},
                fillColor={215,215,215},
                fillPattern=FillPattern.Solid,
                origin={15,0},
                rotation=90),
              Rectangle(
                extent={{-100,10},{100,-10}},
                lineColor={0,0,0},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                origin={30,0},
                rotation=90),
              Line(points={{-40,-80},{40,-80}},color={0,0,0}),
              Line(points={{-40,-60},{40,-60}},color={0,0,0}),
              Line(points={{-40,-20},{40,-20}},color={0,0,0}),
              Line(points={{-40,-40},{40,-40}},color={0,0,0}),
              Line(points={{-40,80},{40,80}},  color={0,0,0}),
              Line(points={{-40,60},{40,60}},  color={0,0,0}),
              Line(points={{-40,40},{40,40}},  color={0,0,0}),
              Line(points={{-40,20},{40,20}},  color={0,0,0}),
              Line(points={{-40,0},{40,0}},    color={0,0,0})}),       Diagram(coordinateSystem(preserveAspectRatio=false)));
      end Simplified_model_1;
    end Simplified;

    package TwoPhaseHX
      model SteamDrum
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;
          parameter Real K_tuning = 1;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;

        // Water/Steam properties
          // Drum pressure
          Inputs.InputPressure P_drum "Drum saturation pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_fw "Feed water mass flow rate";
          Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          Units.PositiveMassFlowRate Q_s "Steam mass flow rate";
          Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l "Liquid volume";
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
          Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          Inputs.InputSpecificEnthalpy h_r "Risers enthalpy";
          Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          Units.Density rho_b "Bubble density";
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          Units.Temperature T_wall;
          // Water surface tension
          Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          Units.Area A_ma;
          // Water level;
          Inputs.InputHeight l_water;
          // Escape velocity
          Units.Velocity u_s;

        WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
        WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
        WaterSteam.Connectors.Inlet risers_in annotation (Placement(transformation(extent={{-90,-70},{-70,-50}}), iconTransformation(extent={{-90,-70},{-70,-50}})));
        WaterSteam.Connectors.Outlet downcomers_out annotation (Placement(transformation(extent={{-10,-110},{10,-90}}), iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
        WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
        WaterSteam.BoundaryConditions.Sink Risers_sink annotation (Placement(transformation(extent={{-26,-70},{-6,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel Risers_supply annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
        WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,-4})));
        WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,38})));
        WaterSteam.BoundaryConditions.Source Downcomers_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-36})));
        WaterSteam.BaseClasses.IsoPHFlowModel Downcomers_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-60})));
      equation
        // Variables definition
          // Enthalpies:
          h_s = Steam_extraction.h_out;
          h_fw = FW_supply.h_in;
          h_r = Risers_supply.h_in;
          h_d = Downcomers_extraction.h_out;
          // Pressures
          P_drum = FW_supply.P_in;
          P_drum = Risers_supply.P_in;
          P_drum = Steam_extraction.P_in;
          P_drum = Downcomers_extraction.P_in;
          // Mass flow rates
          Q_fw = FW_supply.Q;
          Q_r = Risers_supply.Q;
          Q_s = Steam_extraction.Q;
          Q_d = Downcomers_extraction.Q;
          // States
          state_l = WaterSteamMedium.setBubbleState(WaterSteamMedium.setSat_p(P_drum));
          state_s = WaterSteamMedium.setDewState(WaterSteamMedium.setSat_p(P_drum));
          // Surface tension
          sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          rho_b = WaterSteamMedium.density(state_s);

        // Enthalpies
        h_s = WaterSteamMedium.specificEnthalpy(state_s);

        // Volume
        V_s = V_D - V_l - V_b;
        V_l = R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5;

        // Interface area
        A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
        // Escape velocity
        u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4);
        // Bubble mass flow rate
        Q_b = K_tuning*A_ma*u_s*rho_s;

        // Mass flow rates
        Q_s = der(V_s*rho_s);
        Q_b = der(V_b*rho_b);
        // Mass balance
        Q_fw + Q_r - Q_s - Q_d = der(V_l*rho_l + V_s*rho_s + V_b*rho_b);
        // Energy balance
        der(V_l*rho_l*h_l + V_s*rho_s*h_s + V_b*rho_b*h_b) - (V_s + V_b + V_l)*der(P_drum) = Q_fw*h_fw + Q_r*h_r - Q_s*h_s - Q_d*h_d - M_D*Cp_D*der(T_wall);



        connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
        connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
        connect(Steam_source.C_out, Steam_extraction.C_in) annotation (Line(points={{0,1},{0,28}}, color={28,108,200}));
        connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
        connect(Downcomers_source.C_out, Downcomers_extraction.C_in) annotation (Line(points={{0,-41},{0,-50}}, color={28,108,200}));
        connect(Downcomers_extraction.C_out, downcomers_out) annotation (Line(points={{0,-70},{0,-100}}, color={28,108,200}));
        connect(Risers_supply.C_in, risers_in) annotation (Line(points={{-60,-60},{-80,-60}}, color={28,108,200}));
        connect(Risers_supply.C_out, Risers_sink.C_in) annotation (Line(points={{-40,-60},{-21,-60}}, color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum;

      model SteamDrum_No_connectors
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;
          parameter Real K_tuning = 1;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;

        // Water/Steam properties
          // Drum pressure
          Inputs.InputPressure P_drum "Drum saturation pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_fw "Feed water mass flow rate";
          Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          Units.PositiveMassFlowRate Q_s "Steam mass flow rate";
          Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l "Liquid volume";
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          input Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
          Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          input Units.SpecificEnthalpy h_r "Risers enthalpy";
          Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          Units.Density rho_b "Bubble density";
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          Units.Temperature T_wall;
          // Water surface tension
          Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          Units.Area A_ma;
          // Water level;
          Inputs.InputHeight l_water;
          // Escape velocity
          Units.Velocity u_s;

      equation
        // Variables definition
      //     // Enthalpies:
      //     h_s = Steam_extraction.h_out;
      //     h_fw = FW_supply.h_in;
      //     h_r = Risers_supply.h_in;
      //     h_d = Downcomers_extraction.h_out;
      //     // Pressures
      //     P_drum = FW_supply.P_in;
      //     P_drum = Risers_supply.P_in;
      //     P_drum = Steam_extraction.P_in;
      //     P_drum = Downcomers_extraction.P_in;
      //     // Mass flow rates
      //     Q_fw = FW_supply.Q;
      //     Q_r = Risers_supply.Q;
      //     Q_s = Steam_extraction.Q;
      //     Q_d = Downcomers_extraction.Q;
          // States
          state_l = WaterSteamMedium.setBubbleState(WaterSteamMedium.setSat_p(P_drum));
          state_s = WaterSteamMedium.setDewState(WaterSteamMedium.setSat_p(P_drum));
          // Surface tension
          sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          rho_b = WaterSteamMedium.density(state_s);

        // Enthalpies
        h_s = WaterSteamMedium.specificEnthalpy(state_s);
        h_l = WaterSteamMedium.specificEnthalpy(state_l);
        h_b = h_s;
        h_d = h_l;

        // Volume
        V_s = V_D - V_l - V_b;
        V_l + V_b = (R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5)*L_D;

        // Interface area
        A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
        // Escape velocity
        u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4);
        // Bubble mass flow rate
        Q_b = K_tuning*A_ma*u_s*rho_s;

        // Mass flow rates
        //Q_s = der(V_s*rho_s);
        //Q_b = der(V_b*rho_b);
        // Mass balance
        Q_fw + Q_r - Q_s - Q_d = der(V_l*rho_l + V_s*rho_s + V_b*rho_b);
        der(V_l*rho_l + V_b*rho_b) = Q_fw + Q_r - Q_d - Q_b;
        // Energy balance
        der(V_l*rho_l*h_l + V_s*rho_s*h_s + V_b*rho_b*h_b) - (V_s + V_b + V_l)*der(P_drum) = Q_fw*h_fw + Q_r*h_r - Q_s*h_s - Q_d*h_d  - M_D*Cp_D*der(T_wall);

        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_No_connectors;

      model SteamDrum_No_connectors_bis
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;
          parameter Real K_tuning = 1;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;

        // Water/Steam properties
          // Drum pressure
          input Units.Pressure P_drum(start=124.4*1e5) "Drum saturation pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_fw "Feed water mass flow rate";
          //Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          input Units.PositiveMassFlowRate Q_s(start=85) "Steam mass flow rate";
          Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          input Units.PositiveMassFlowRate Q_r(start=85*3) "Risers mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          //Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l "Liquid volume";
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          input Units.SpecificEnthalpy h_fw(start=1511463.2 - 3000) "Feedwater enthalpy";
          Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          input Units.SpecificEnthalpy h_r(start=2674485-3000) "Risers enthalpy";
          //Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          //Units.Density rho_b "Bubble density";
          // Masses
          Units.Mass M_s;
          Units.Mass M_l;
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          //Units.Temperature T_wall;
          // Water surface tension
          //Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          //Units.Area A_ma;

          // Water level;
          input Units.Height l_water(start=1.2);
          // Escape velocity
          //Units.Velocity u_s;

      equation
        // Variables definition
      //     // Enthalpies:
      //     h_s = Steam_extraction.h_out;
      //     h_fw = FW_supply.h_in;
      //     h_r = Risers_supply.h_in;
      //     h_d = Downcomers_extraction.h_out;
      //     // Pressures
      //     P_drum = FW_supply.P_in;
      //     P_drum = Risers_supply.P_in;
      //     P_drum = Steam_extraction.P_in;
      //     P_drum = Downcomers_extraction.P_in;
      //     // Mass flow rates
      //     Q_fw = FW_supply.Q;
      //     Q_r = Risers_supply.Q;
      //     Q_s = Steam_extraction.Q;
      //     Q_d = Downcomers_extraction.Q;
          // States
          state_l = WaterSteamMedium.setBubbleState(WaterSteamMedium.setSat_p(P_drum));
          state_s = WaterSteamMedium.setDewState(WaterSteamMedium.setSat_p(P_drum));
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          //rho_b = WaterSteamMedium.density(state_s);

        // Enthalpies
        h_s = WaterSteamMedium.specificEnthalpy(state_s);
        h_l = WaterSteamMedium.specificEnthalpy(state_l);
        //h_b = h_s;

        // Volumes
        V_s = V_D - V_l;
        V_l = (R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5)*L_D;

        // Masses
        M_s = V_s*rho_s;
        M_l = V_l*rho_l;

        // Mass balance
        //der(M_l) + der(M_s) = Q_fw + Q_r - Q_d - Q_s;
        0 = Q_fw + Q_r - Q_d - Q_s;
        Q_d = Q_r;
        // Energy balance
        //der(M_l*h_l) + der(M_s*h_s) = Q_fw*h_fw + Q_r*h_r - Q_d*h_d - Q_s*h_s;
        0 = Q_fw*h_fw + Q_r*h_r - Q_d*h_d - Q_s*h_s;

        // Interface area
        //A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
        // Escape velocity
        //u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4);
        // Bubble mass flow rate
        //Q_b = K_tuning*A_ma*u_s*rho_s;

        // Mass flow rates
        //Q_s = der(V_s*rho_s);
        //Q_b = der(V_b*rho_b);
        // Mass balance
        //Q_fw + Q_r - Q_s - Q_d = der(V_l*rho_l + V_s*rho_s + V_b*rho_b);
        //der(V_l*rho_l + V_b*rho_b) = Q_fw + Q_r - Q_d - Q_b;
        // Energy balance
        //der(V_l*rho_l*h_l + V_s*rho_s*h_s + V_b*rho_b*h_b) - (V_s + V_b + V_l)*der(P_drum) = Q_fw*h_fw + Q_r*h_r - Q_s*h_s - Q_d*h_d  - M_D*Cp_D*der(T_wall);

      // initial equation
      //   der(M_l) = 0;
      //   der(M_s) = 0;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_No_connectors_bis;

      model SteamDrum_Only_Mixing
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;
          parameter Real K_tuning = 1;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;

        // Water/Steam properties
          // Drum pressure
          input Units.Pressure P_drum(start=124.4*1e5) "Drum saturation pressure";
          // Mass flow rates
          input Units.PositiveMassFlowRate Q_fw(start=85) "Feed water mass flow rate";
          //Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          input Units.PositiveMassFlowRate Q_s(start=85) "Steam mass flow rate";
          Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          input Units.PositiveMassFlowRate Q_r(start=85*3) "Risers mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          //Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l "Liquid volume";
          // Enthalpies
          //Units.SpecificEnthalpy h_s "Steam enthalpy";
          //Units.SpecificEnthalpy h_l "Liquid enthalpy";
          //input Units.SpecificEnthalpy h_fw(start=1511463.2 - 3000) "Feedwater enthalpy";
          //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          //input Units.SpecificEnthalpy h_r(start=2674485-3000) "Risers enthalpy";
          //Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          //Units.Density rho_b "Bubble density";
          // Masses
          Units.Mass M_s;
          Units.Mass M_l;
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          //Units.Temperature T_wall;
          // Water surface tension
          //Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          //Units.Area A_ma;

          // Water level;
          Units.Height l_water(start=1.2, fixed=true);
          input Units.Height l_water_in(start=1.2);
          // Escape velocity
          //Units.Velocity u_s;

        Modelica.Blocks.Sources.Ramp ramp(
          height=0.5,
          duration=60,
          startTime=20)  annotation (Placement(transformation(extent={{-40,26},{-20,46}})));
        Modelica.Blocks.Sources.Ramp ramp1(
          height=-0.5,
          duration=60,
          startTime=100) annotation (Placement(transformation(extent={{36,24},{56,44}})));
      equation
        // l_water = l_water_in + ramp.y;
        // Variables definition
      //     // Enthalpies:
      //     h_s = Steam_extraction.h_out;
      //     h_fw = FW_supply.h_in;
      //     h_r = Risers_supply.h_in;
      //     h_d = Downcomers_extraction.h_out;
      //     // Pressures
      //     P_drum = FW_supply.P_in;
      //     P_drum = Risers_supply.P_in;
      //     P_drum = Steam_extraction.P_in;
      //     P_drum = Downcomers_extraction.P_in;
      //     // Mass flow rates
      //     Q_fw = FW_supply.Q;
      //     Q_r = Risers_supply.Q;
      //     Q_s = Steam_extraction.Q;
      //     Q_d = Downcomers_extraction.Q;
          // States
          state_l = WaterSteamMedium.setBubbleState(WaterSteamMedium.setSat_p(P_drum));
          state_s = WaterSteamMedium.setDewState(WaterSteamMedium.setSat_p(P_drum));
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          //rho_b = WaterSteamMedium.density(state_s);

        // Enthalpies
        //h_s = WaterSteamMedium.specificEnthalpy(state_s);
        //h_l = WaterSteamMedium.specificEnthalpy(state_l);
        //h_b = h_s;

        // Volumes
        V_s = V_D - V_l;
        V_l = (R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5)*L_D;

        // Masses
        M_s = V_s*rho_s;
        M_l = V_l*rho_l;

        // Mass balance
        der(V_l)*rho_l + der(V_s)*rho_s = Q_fw + Q_r - Q_d - Q_s;
        //0 = Q_fw + Q_r - Q_d - Q_s;
        Q_d = Q_r;
        // Energy balance
        //der(M_l*h_l) + der(M_s*h_s) = Q_fw*h_fw + Q_r*h_r - Q_d*h_d - Q_s*h_s;
        //0 = Q_fw*h_fw + Q_r*h_r - Q_d*h_d - Q_s*h_s;

        // Interface area
        //A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
        // Escape velocity
        //u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4);
        // Bubble mass flow rate
        //Q_b = K_tuning*A_ma*u_s*rho_s;

        // Mass flow rates
        //Q_s = der(V_s*rho_s);
        //Q_b = der(V_b*rho_b);
        // Mass balance
        //Q_fw + Q_r - Q_s - Q_d = der(V_l*rho_l + V_s*rho_s + V_b*rho_b);
        //der(V_l*rho_l + V_b*rho_b) = Q_fw + Q_r - Q_d - Q_b;
        // Energy balance
        //der(V_l*rho_l*h_l + V_s*rho_s*h_s + V_b*rho_b*h_b) - (V_s + V_b + V_l)*der(P_drum) = Q_fw*h_fw + Q_r*h_r - Q_s*h_s - Q_d*h_d  - M_D*Cp_D*der(T_wall);

      // initial equation
      //   der(V_l) = 0;
      //   der(V_s) = 0;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_Only_Mixing;

      model SteamDrum_add_energy_balance
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;
          parameter Real K_tuning = 1;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;

        // Water/Steam properties
          // Drum pressure
          input Units.Pressure P_drum(start=124.4*1e5) "Drum saturation pressure";
          // Mass flow rates
          input Units.PositiveMassFlowRate Q_fw(start=85) "Feed water mass flow rate";
          //Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          input Units.PositiveMassFlowRate Q_s(start=85) "Steam mass flow rate";
          Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          input Units.PositiveMassFlowRate Q_r(start=85*3) "Risers mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          //Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l "Liquid volume";
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          input Units.SpecificEnthalpy h_fw(start=1511463.2 - 3000) "Feedwater enthalpy";
          Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          input Units.SpecificEnthalpy h_r(start=2674485-3000) "Risers enthalpy";
          //Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          //Units.Density rho_b "Bubble density";
          // Masses
          Units.Mass M_s;
          Units.Mass M_l;
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          //Units.Temperature T_wall;
          // Water surface tension
          //Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          //Units.Area A_ma;

          // Water level;
          Units.Height l_water(start=1.2, fixed=true);
          input Units.Height l_water_in(start=1.2);
          // Escape velocity
          //Units.Velocity u_s;

        Modelica.Blocks.Sources.Ramp ramp(
          height=0.5,
          duration=60,
          startTime=20)  annotation (Placement(transformation(extent={{-40,26},{-20,46}})));
        Modelica.Blocks.Sources.Ramp ramp1(
          height=-0.5,
          duration=60,
          startTime=100) annotation (Placement(transformation(extent={{36,24},{56,44}})));
      equation
        // l_water = l_water_in + ramp.y;
        // Variables definition
      //     // Enthalpies:
      //     h_s = Steam_extraction.h_out;
      //     h_fw = FW_supply.h_in;
      //     h_r = Risers_supply.h_in;
      //     h_d = Downcomers_extraction.h_out;
      //     // Pressures
      //     P_drum = FW_supply.P_in;
      //     P_drum = Risers_supply.P_in;
      //     P_drum = Steam_extraction.P_in;
      //     P_drum = Downcomers_extraction.P_in;
      //     // Mass flow rates
      //     Q_fw = FW_supply.Q;
      //     Q_r = Risers_supply.Q;
      //     Q_s = Steam_extraction.Q;
      //     Q_d = Downcomers_extraction.Q;
          // States
          state_l = WaterSteamMedium.setBubbleState(WaterSteamMedium.setSat_p(P_drum));
          state_s = WaterSteamMedium.setDewState(WaterSteamMedium.setSat_p(P_drum));
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          //rho_b = WaterSteamMedium.density(state_s);

        // Enthalpies
        h_s = WaterSteamMedium.specificEnthalpy(state_s);
        h_l = WaterSteamMedium.specificEnthalpy(state_l);
        //h_b = h_s;

        // Volumes
        V_s = V_D - V_l;
        V_l = (R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5)*L_D;

        // Masses
        M_s = V_s*rho_s;
        M_l = V_l*rho_l;

        // Mass balance
        der(V_l)*rho_l + der(V_s)*rho_s = Q_fw + Q_r - Q_d - Q_s;
        //0 = Q_fw + Q_r - Q_d - Q_s;
        Q_d = Q_r;
        // Energy balance
        der(V_l)*rho_l*h_l + der(V_s)*rho_s*h_s = Q_fw*h_fw + Q_r*h_r - Q_d*h_d - Q_s*h_s;
        //0 = Q_fw*h_fw + Q_r*h_r - Q_d*h_d - Q_s*h_s;

        // Interface area
        //A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
        // Escape velocity
        //u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4);
        // Bubble mass flow rate
        //Q_b = K_tuning*A_ma*u_s*rho_s;

        // Mass flow rates
        //Q_s = der(V_s*rho_s);
        //Q_b = der(V_b*rho_b);
        // Mass balance
        //Q_fw + Q_r - Q_s - Q_d = der(V_l*rho_l + V_s*rho_s + V_b*rho_b);
        //der(V_l*rho_l + V_b*rho_b) = Q_fw + Q_r - Q_d - Q_b;
        // Energy balance
        //der(V_l*rho_l*h_l + V_s*rho_s*h_s + V_b*rho_b*h_b) - (V_s + V_b + V_l)*der(P_drum) = Q_fw*h_fw + Q_r*h_r - Q_s*h_s - Q_d*h_d  - M_D*Cp_D*der(T_wall);

      // initial equation
      //   der(V_l) = 0;
      //   der(V_s) = 0;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_add_energy_balance;

      model SteamDrum_Bracco
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;
          parameter Real K_tuning = 1;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;

        // Water/Steam properties
          // Drum pressure
          Units.Pressure P_drum(start=124.4*1e5, fixed=true) "Drum saturation pressure"; // (start=124.4*1e5)
          // Mass flow rates
          input Units.PositiveMassFlowRate Q_fw(start=82) "Feed water mass flow rate";
          //Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          input Units.PositiveMassFlowRate Q_s(start=82) "Steam mass flow rate";
          //Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          //input Units.PositiveMassFlowRate Q_r(start=85*3) "Risers mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          //Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l(start=2, fixed=true) "Liquid volume";
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          input Units.SpecificEnthalpy h_fw(start=1511463.2 - 3000) "Feedwater enthalpy";
          //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          //input Units.SpecificEnthalpy h_r(start=2674485-3000) "Risers enthalpy";
          //Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Internal energies
          Modelica.Units.SI.SpecificInternalEnergy u_l;
          Modelica.Units.SI.SpecificInternalEnergy u_s;


          // Densities
          Units.Density rho_s(start=73.58) "Steam density";
          Units.Density rho_l(start=647.8067) "Liquid density";
          //Units.Density rho_b "Bubble density";
          // Masses
          Units.Mass M_s;
          Units.Mass M_l;
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          //Units.Temperature T_wall;
          // Water surface tension
          //Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          //Units.Area A_ma;

          // Water level;
          Units.Height l_water(start=1.2);
          //input Units.Height l_water_in(start=1.2);
          // Escape velocity
          //Units.Velocity u_s;
          Real ddph_l;
          Real ddhp_l;
          Real ddph_s;
          Real ddhp_s;
          Real dup_l;
          Real dup_s;

          Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;
          input Real Q_in(start=0); //104e6

        Modelica.Blocks.Sources.Ramp ramp(
          height=0.5,
          duration=60,
          startTime=20)  annotation (Placement(transformation(extent={{-40,26},{-20,46}})));
        Modelica.Blocks.Sources.Ramp ramp1(
          height=-0.5,
          duration=60,
          startTime=100) annotation (Placement(transformation(extent={{36,24},{56,44}})));
      equation

        sat.psat = P_drum;
        sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

        //P_drum =

        // l_water = l_water_in + ramp.y;
        // Variables definition
      //     // Enthalpies:
      //     h_s = Steam_extraction.h_out;
      //     h_fw = FW_supply.h_in;
      //     h_r = Risers_supply.h_in;
      //     h_d = Downcomers_extraction.h_out;
      //     // Pressures
      //     P_drum = FW_supply.P_in;
      //     P_drum = Risers_supply.P_in;
      //     P_drum = Steam_extraction.P_in;
      //     P_drum = Downcomers_extraction.P_in;
      //     // Mass flow rates
      //     Q_fw = FW_supply.Q;
      //     Q_r = Risers_supply.Q;
      //     Q_s = Steam_extraction.Q;
      //     Q_d = Downcomers_extraction.Q;
          // States
          state_l = WaterSteamMedium.setBubbleState(sat);
          state_s = WaterSteamMedium.setDewState(sat);
            //state_l = WaterSteamMedium.setBubbleState(p=P_drum);
            //state_s = WaterSteamMedium.setDewState(p=P_drum);
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          //rho_b = WaterSteamMedium.density(state_s);
        // Enthalpies
        h_s = WaterSteamMedium.specificEnthalpy(state_s);
        h_l = WaterSteamMedium.specificEnthalpy(state_l);

        // Volumes
        V_s = V_D - V_l;
        V_l = (R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5)*L_D;

        // Masses
        M_s = V_s*rho_s;
        M_l = V_l*rho_l;

        // Internal energies
        u_l = WaterSteamMedium.specificInternalEnergy(state_l);
        u_s = WaterSteamMedium.specificInternalEnergy(state_s);

        // Mass balance
        ddph_l = WaterSteamMedium.density_derp_h(state_l);
        ddhp_l = WaterSteamMedium.density_derh_p(state_l);
        ddph_s = WaterSteamMedium.density_derp_h(state_s);
        ddhp_s = WaterSteamMedium.density_derh_p(state_s);
        dup_l = der(h_l) - (der(P_drum)/rho_l - P_drum/rho_l^2*(ddph_l*der(P_drum) + ddhp_l*der(h_l)));
        dup_s = der(h_s) - (der(P_drum)/rho_s - P_drum/rho_s^2*(ddph_s*der(P_drum) + ddhp_s*der(h_s)));

        (ddph_l*der(P_drum) + ddhp_l*der(h_l))*V_l + rho_l*der(V_l)
        + (ddph_s*der(P_drum) + ddhp_s*der(h_s))*V_s + rho_s*der(V_s)
        = Q_fw - Q_s;

        // der(rho_l)*V_l + rho_l*der(V_l) + der(rho_s)*V_s + rho_s*der(V_s) = Q_fw - Q_s;

        // Energy balance
        //Q_fw*h_fw - Q_s*h_s + 104e6 = rho_l*u_l*der(V_l) + rho_s*u_s*der(V_l);

        //((ddph_l*der(P_drum) + ddhp_l*der(h_l))*V_l*u_l
       // + (der(h_l) -
        (ddph_l*V_l*u_l + dup_l*rho_l*V_l
        + ddph_s*V_s*u_s
        + dup_s*rho_s*V_s)*der(P_drum)
        + der(V_l)*rho_l*u_l - der(V_s)*rho_s*u_s
        = Q_fw*h_fw - Q_s*h_s + Q_in;

      //MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium.density_derp_h


      //
      //   // Mass balance
      //   der(V_l*rho_l) + der(V_s*rho_s) = Q_fw + Q_r - Q_d - Q_s;
      //   //0 = Q_fw + Q_r - Q_d - Q_s;
      //   Q_d = Q_r;
      //   // Energy balance
      //   der(V_l*rho_l*h_l) + der(V_s*rho_s*h_s) = Q_fw*h_fw + Q_r*h_r - Q_d*h_d - Q_s*h_s;
      //   //0 = Q_fw*h_fw + Q_r*h_r - Q_d*h_d - Q_s*h_s;

        // Interface area
        //A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
        // Escape velocity
        //u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4);
        // Bubble mass flow rate
        //Q_b = K_tuning*A_ma*u_s*rho_s;

        // Mass flow rates
        //Q_s = der(V_s*rho_s);
        //Q_b = der(V_b*rho_b);
        // Mass balance
        //Q_fw + Q_r - Q_s - Q_d = der(V_l*rho_l + V_s*rho_s + V_b*rho_b);
        //der(V_l*rho_l + V_b*rho_b) = Q_fw + Q_r - Q_d - Q_b;
        // Energy balance
        //der(V_l*rho_l*h_l + V_s*rho_s*h_s + V_b*rho_b*h_b) - (V_s + V_b + V_l)*der(P_drum) = Q_fw*h_fw + Q_r*h_r - Q_s*h_s - Q_d*h_d  - M_D*Cp_D*der(T_wall);

      // initial equation
      //   der(V_l) = 0;
      //   der(V_s) = 0;


      //   der(P_drum) = 0;
      //   der(h_l)= 0;
      //   der(h_s) = 0;
      //   der(V_s) = 0;
      //   der(V_l) = 0;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_Bracco;

      record Prop
        import MetroscopeModelingLibrary.Utilities.Units;
          Units.Temperature T "Temperature";
        Units.Density d "Density";
        Modelica.Units.SI.InternalEnergy u "Specific inner energy";
        Modelica.Units.SI.SpecificEntropy s "Specific entropy";
        Real cp "Specific heat capacity at constant presure";
        Real ddhp "Derivative of density wrt. specific enthalpy at constant pressure";
        Real ddph "Derivative of density wrt. pressure at constant specific enthalpy";
        Real duph(unit="m3/kg") "Derivative of specific inner energy wrt. pressure at constant specific enthalpy";
        Real duhp(unit = "1") "Derivative of specific inner energy wrt. specific enthalpy at constant pressure";
        ThermoSysPro.Units.SI.MassFraction x "Vapor mass fraction";
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end Prop;

      model SteamDrum_Astom_Simple
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;

        // Control strategy
          parameter Boolean Controlled_level = false;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;

        // Initialization parameters
        parameter Units.Pressure P_drum_0 = 121.631e5;
        parameter Modelica.Units.SI.Volume V_l_0 = 20;

        // Water/Steam properties
          // Drum pressure
          Units.Pressure P_drum(start=P_drum_0) "Drum saturation pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
          //Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
          //Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          //Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          //Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
          //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          //Inputs.InputSpecificEnthalpy h_r "Risers enthalpy";
          //Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          //Units.Density rho_b "Bubble density";
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          Units.Temperature T_wall;
          // Water surface tension
          //Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          //Units.Area A_ma;
          // Water level;
          //Inputs.InputHeight l_water;
          // Escape velocity
          //Units.Velocity u_s;

          // Masses
          Units.Mass M_l;
          Units.Mass M_s;

          // Evaporation heat
          Inputs.InputPower W_evap(start=104e6);

          // Set saturation state
          Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

          // Derrivatives
          Real ddph_l;
          Real ddph_s;


        WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
        WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
        WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
        WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,-4})));
        WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,38})));
      equation
        // Variables definition
          // Enthalpies:
          h_s = Steam_source.h_out;
          h_fw = FW_supply.h_in;
          // Pressures
          P_drum = FW_sink.P_in;
          P_drum = Steam_extraction.P_in;
          // Mass flow rates
          Q_fw = FW_supply.Q;
          //Q_r = Risers_supply.Q;
          Q_s = Steam_extraction.Q;
          //Q_d = Downcomers_extraction.Q;
          // Set saturation state
          sat.psat = P_drum;
          sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          // States
          state_l = WaterSteamMedium.setBubbleState(sat);
          state_s = WaterSteamMedium.setDewState(sat);
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          //rho_b = WaterSteamMedium.density(state_s);

          // Enthalpies
          h_s = WaterSteamMedium.specificEnthalpy(state_s);
          h_l = WaterSteamMedium.specificEnthalpy(state_l);

          // Masses
          M_s = rho_s*V_s;
          M_l = rho_l*V_l;

          // Volume
          V_s = V_D - V_l;

          // Mass balance
          Q_fw - Q_s = V_s*(ddph_s*der(P_drum)) + rho_s*der(V_s)
                     + V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l);

          // Energy Balance
          W_evap + Q_fw*h_fw - Q_s*h_s = h_s*V_s*ddph_s*der(P_drum) + rho_s*V_s*der(h_s) + rho_s*h_s*der(V_s)
                                       + h_l*V_l*ddph_l*der(P_drum) + rho_l*V_l*der(h_l) + rho_l*h_l*der(V_l)
                                       + V_D*der(P_drum)
                                       + M_D*Cp_D*der(T_wall);

          // Derrivatives
          ddph_l = WaterSteamMedium.density_derp_h(state_l);
          ddph_s = WaterSteamMedium.density_derp_h(state_s);

          if Controlled_level then
            der(V_l) = 0;
          end if;

      initial equation
        der(P_drum) = 0;




      equation
        connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
        connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
        connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
        connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_Astom_Simple;

      model SteamDrum_Sunil
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;

        // Control strategy
          parameter Boolean Controlled_level = false;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;
          parameter Real K_tuning = 1;

        // Initialization parameters
        parameter Units.Pressure P_drum_0 = 121.631e5;
        parameter Modelica.Units.SI.Volume V_l_0 = 20;

        // Water/Steam properties
          // Drum pressure
          Units.Pressure P_drum(start=P_drum_0) "Drum saturation pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
          Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
          Units.PositiveMassFlowRate Q_d(start=80) "Downcomers mass flow rate";
          Units.PositiveMassFlowRate Q_r(start=80) "Risers mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          Units.SpecificEnthalpy h_fw(start=1431487.1) "Feedwater enthalpy";
          Units.SpecificEnthalpy h_d(start=1509058.1) "Downcomers enthalpy";
          Units.SpecificEnthalpy h_r(start=2675842.2) "Risers enthalpy";
          //Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          Units.Density rho_b "Bubble density";
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          Units.Temperature T_wall;
          // Water surface tension
          Modelica.Units.SI.SurfaceTension sigma_s;
          //Interface Area
          Units.Area A_ma;
          // Water level;
          Units.Height l_water(start=1.2, fixed=true);
          // Escape velocity
          Units.Velocity u_s;
          // Masses
          Units.Mass M_l;
          Units.Mass M_s;
          // Set saturation state
          Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;


          // Derrivatives
          Real ddph_l;
          Real ddph_s;
          Real ddph_b;

        WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
        WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
        WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
        WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,-4})));
        WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,38})));
        WaterSteam.BaseClasses.IsoPHFlowModel Downcomers_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-60})));
        WaterSteam.Connectors.Outlet downcomers_out annotation (Placement(transformation(extent={{-10,-110},{10,-90}}), iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.BoundaryConditions.Sink Risers_sink annotation (Placement(transformation(extent={{-26,-70},{-6,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel Risers_supply annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
        WaterSteam.Connectors.Inlet risers_in annotation (Placement(transformation(extent={{-90,-70},{-70,-50}}), iconTransformation(extent={{-90,-70},{-70,-50}})));
        WaterSteam.BoundaryConditions.Source Downcomers_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-28})));
      equation
        // Variables definition
          // Enthalpies:
          h_s = Steam_extraction.h_out;
          h_fw = FW_supply.h_in;
          h_r = Risers_supply.h_in;
          h_d = Downcomers_extraction.h_out;
          // Pressures
          P_drum = FW_supply.P_in;
          P_drum = Risers_supply.P_in;
          P_drum = Steam_extraction.P_in;
          P_drum = Downcomers_extraction.P_in;
          // Mass flow rates
          Q_fw = FW_supply.Q;
          Q_r = Risers_supply.Q;
          Q_s = Steam_extraction.Q;
          Q_d = Downcomers_extraction.Q;
          // Set saturation state
          sat.psat = P_drum;
          sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          // Wall temperature is assumed equal to the saturation temperature
          T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);


          // States
          state_l = WaterSteamMedium.setBubbleState(sat);
          state_s = WaterSteamMedium.setDewState(sat);

          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          rho_b = WaterSteamMedium.density(state_s); // Steam bubble density = sat. density

          // Enthalpies
          h_s = WaterSteamMedium.specificEnthalpy(state_s);
          h_l = WaterSteamMedium.specificEnthalpy(state_l);

          // Masses
          M_s = rho_s*V_s + rho_b*V_b;
          M_l = rho_l*V_l;

          // Volume
          V_s = V_D - V_l - V_b;
          V_l + V_b = (R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5)*L_D;

          // Mass balance
          // Total mass balance
          Q_fw + Q_r - Q_s - Q_d = V_s*(ddph_s*der(P_drum)) + rho_s*der(V_s)
                                 + V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l)
                                 + V_b*(ddph_b*der(P_drum)) + rho_b*der(V_b);

          // Mass balance on control volume with liquid and bubble region:
          Q_fw + Q_r - Q_d - Q_b = V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l)
                                 + V_b*(ddph_b*der(P_drum)) + rho_b*der(V_b);

          Q_fw*h_fw + Q_r*h_r - Q_s*h_s - Q_d*h_d = h_l*V_l*ddph_l*der(P_drum) + rho_l*V_l*der(h_l)
                                                  + V_D*h_s*ddph_s*der(P_drum) + rho_s*V_D*der(h_s)
                                                  - V_l*h_s*ddph_s*der(P_drum)
                                                  + der(V_l)*(rho_l*h_l - rho_s*h_s)
                                                  + M_D*Cp_D*der(T_wall);

          //Q_r*h_r - Q_b*h_d = 104e6;

          // Bubble mass flow rate
          // Surface tension
          sigma_s = WaterSteamMedium.surfaceTension(sat);
          // Interface area
          A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
          // Escape velocity
          u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4);
          // Bubble mass flow rate
          Q_b = K_tuning*A_ma*u_s*rho_s;

          // Derrivatives
          ddph_l = WaterSteamMedium.density_derp_h(state_l);
          ddph_s = WaterSteamMedium.density_derp_h(state_s);
          ddph_b = WaterSteamMedium.density_derp_h(state_s);

          if Controlled_level then
            der(V_l) = 0;
          end if;

      initial equation
        der(P_drum) = 0;
        //der(V_b) = 0;
        //der(V_l) = 0;

      equation
        connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
        connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
        connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
        connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
        connect(Downcomers_extraction.C_out, downcomers_out) annotation (Line(points={{0,-70},{0,-70},{0,-100}}, color={28,108,200}));
        connect(Downcomers_extraction.C_in, Downcomers_source.C_out) annotation (Line(points={{0,-50},{0,-33}}, color={28,108,200}));
        connect(Risers_sink.C_in, Risers_supply.C_out) annotation (Line(points={{-21,-60},{-40,-60}}, color={28,108,200}));
        connect(Risers_supply.C_in, risers_in) annotation (Line(points={{-60,-60},{-80,-60}}, color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_Sunil;

      model SteamDrum_Sunil_noConToEvap
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;

        // Control strategy
          parameter Boolean Controlled_level = false;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;
          parameter Real K_tuning = 1;

        // Initialization parameters
        parameter Units.Pressure P_drum_0 = 121.631e5;
        parameter Modelica.Units.SI.Volume V_l_0 = 20;

        // Water/Steam properties
          // Drum pressure
          Units.Pressure P_drum(start=P_drum_0) "Drum saturation pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_fw(start=80, fixed=true) "Feed water mass flow rate";
          Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          Units.PositiveMassFlowRate Q_s(start=80, fixed=true) "Steam mass flow rate";
          Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
          Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          Units.SpecificEnthalpy h_r "Risers enthalpy";
          //Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          Units.Density rho_b "Bubble density";
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          Units.Temperature T_wall;
          // Water surface tension
          Modelica.Units.SI.SurfaceTension sigma_s;
          //Interface Area
          Units.Area A_ma;
          // Water level;
          Units.Height l_water;
          // Escape velocity
          Units.Velocity u_s;
          // Masses
          //Units.Mass M_l;
          //Units.Mass M_s;
          // Set saturation state
          Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;
          // Downcomers and risers states
          WaterSteamMedium.ThermodynamicState d_state;
          WaterSteamMedium.ThermodynamicState r_state;

          // Derrivatives
          Real ddph_l;
          Real ddph_s;
          Real ddph_b;

        WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{40,-70},{20,-50}})));
        WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{8,-70},{-12,-50}})));
        WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{52,-70},{72,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
        WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-20,-4})));
        WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={-20,38})));
        WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-90,70},{-70,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
      equation
        // Variables definition
          // Enthalpies:
          h_s = Steam_extraction.h_out;
          h_fw = FW_supply.h_in;
          //h_r = Risers_supply.h_in;
          //h_d = Downcomers_extraction.h_out;
          // Pressures
          P_drum = FW_supply.P_in;
          //P_drum = Risers_supply.P_in;
          P_drum = Steam_extraction.P_in;
          //P_drum = Downcomers_extraction.P_in;
          // Mass flow rates
          Q_fw = FW_supply.Q;
          //Q_r = Risers_supply.Q;
          Q_s = Steam_extraction.Q;
          //Q_d = Downcomers_extraction.Q;
          // Set saturation state
          sat.psat = P_drum;
          sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);
          // Downcomers and risers states
          //r_state.p = P_drum;
          //r_state.h = h_r;
          r_state = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium.setState_phX(P_drum, h_r);
          h_r = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium.specificEnthalpy(r_state);
          d_state = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium.setState_phX(P_drum, h_d);
          h_d = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium.specificEnthalpy(d_state);
          //d_state.p = P_drum;
          //d_state.h = h_d;


          // Wall temperature is assumed equal to the saturation temperature
          T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          // States
          state_l = WaterSteamMedium.setBubbleState(sat);
          state_s = WaterSteamMedium.setDewState(sat);

          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          rho_b = WaterSteamMedium.density(state_s); // Steam bubble density = sat. density

          // Enthalpies
          h_s = WaterSteamMedium.specificEnthalpy(state_s);
          h_l = WaterSteamMedium.specificEnthalpy(state_l);
          //h_b = WaterSteamMedium.specificEnthalpy(state_s);

          // Masses
          //M_s = rho_s*V_s + rho_b*V_b;
          //M_l = rho_l*V_l;

          // Volume
          V_s = V_D - V_l - V_b;
          V_l + V_b = (R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5)*L_D;

          // Mass balance
          // Total mass balance
          Q_fw + Q_r - Q_s - Q_d = V_s*(ddph_s*der(P_drum)) + rho_s*der(V_s)
                                 + V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l)
                                 + V_b*(ddph_b*der(P_drum)) + rho_b*der(V_b);

          // Mass balance on control volume with liquid and bubble region:
          Q_fw + Q_r - Q_d - Q_b = V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l)
                                 + V_b*(ddph_b*der(P_drum)) + rho_b*der(V_b);

          Q_fw*h_fw + Q_r*h_r - Q_s*h_s - Q_d*h_d = h_l*V_l*ddph_l*der(P_drum) + rho_l*V_l*der(h_l)
                                                  + V_D*h_s*ddph_s*der(P_drum) + rho_s*V_D*der(h_s)
                                                  - V_l*h_s*ddph_s*der(P_drum)
                                                  + der(V_l)*(rho_l*h_l - rho_s*h_s)
                                                  + M_D*Cp_D*der(T_wall);

          Q_r*h_r - Q_d*h_d = 104e6;

          // Bubble mass flow rate
          // Surface tension
          sigma_s = WaterSteamMedium.surfaceTension(sat);
          // Interface area
          A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
          // Escape velocity
          u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4);
          // Bubble mass flow rate
          Q_b = K_tuning*A_ma*u_s*rho_s;

          // Derrivatives
          ddph_l = WaterSteamMedium.density_derp_h(state_l);
          ddph_s = WaterSteamMedium.density_derp_h(state_s);
          ddph_b = WaterSteamMedium.density_derp_h(state_s);

          if Controlled_level then
            der(V_l) = 0;
          end if;

      initial equation
        der(P_drum) = 0;

      equation
        connect(FW_supply.C_in,fw_in)  annotation (Line(points={{40,-60},{62,-60}}, color={28,108,200}));
        connect(FW_supply.C_out,FW_sink. C_in) annotation (Line(points={{20,-60},{3,-60}},  color={28,108,200}));
        connect(Steam_extraction.C_out,steam_out)  annotation (Line(points={{-20,48},{-20,80},{-80,80}},
                                                                                                     color={28,108,200}));
        connect(Steam_extraction.C_in,Steam_source. C_out) annotation (Line(points={{-20,28},{-20,1}},
                                                                                                   color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_Sunil_noConToEvap;

      model SteamDrum_TSP
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;

        // Control strategy
          parameter Boolean Controlled_level = false;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;

        // Initialization parameters
        parameter Units.Pressure P_drum_0 = 125.631e5;
        parameter Modelica.Units.SI.Volume V_l_0 = 20;

        // Water/Steam properties
          // Drum pressure
          Units.Pressure P_drum(start=P_drum_0) "Drum saturation pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
          Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
          Units.PositiveMassFlowRate Q_d(start=80) "Downcomers mass flow rate";
          Units.PositiveMassFlowRate Q_r(start=80) "Risers mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          //Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          Units.SpecificEnthalpy h_l_sat "Liquid enthalpy";
          Units.SpecificEnthalpy h_s_sat "Liquid enthalpy";
          Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
          Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          Units.SpecificEnthalpy h_r "Risers enthalpy";
          //Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          //Units.Density rho_b "Bubble density";
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          WaterSteamMedium.ThermodynamicState state_s_sat "Steam state";
          WaterSteamMedium.ThermodynamicState state_l_sat "Liquid state";
          // Wall temperature
          //Units.Temperature T_wall;
          // Water surface tension
          //Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          //Units.Area A_ma;
          // Water level;
          //Inputs.InputHeight l_water;
          // Escape velocity
          //Units.Velocity u_s;

          // Masses
          Units.Mass M_l;
          Units.Mass M_s;

          // Evaporation heat
          Inputs.InputPower W_evap(start=104e6);

          // Set saturation state
          Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

          // Derrivatives
          Real ddph_l;
          Real ddph_s;
          Real ddhp_l;
          Real ddhp_s;

          // Riser steam quality
          Real x_r;

          // Internal Energies
          Real u_l;
          Real u_s;

          // Heat exchange
          Real W_ls;
          Real W_wl;
          Real W_ws;


        WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
        WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
        WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
        WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,-4})));
        WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,38})));
        WaterSteam.BaseClasses.IsoPHFlowModel Downcomers_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-60})));
        WaterSteam.Connectors.Outlet downcomers_out annotation (Placement(transformation(extent={{-10,-110},{10,-90}}), iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.BoundaryConditions.Sink Risers_sink annotation (Placement(transformation(extent={{-26,-70},{-6,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel Risers_supply annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
        WaterSteam.Connectors.Inlet risers_in annotation (Placement(transformation(extent={{-90,-70},{-70,-50}}), iconTransformation(extent={{-90,-70},{-70,-50}})));
        WaterSteam.BoundaryConditions.Source Downcomers_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-28})));
      equation
        // Variables definition
          // Enthalpies:
          h_s = Steam_source.h_out;
          h_fw = FW_supply.h_in;
          h_r = Risers_supply.h_in;
          h_d = Downcomers_extraction.h_out;
          // Pressures
          P_drum = FW_sink.P_in;
          P_drum = Steam_extraction.P_in;
          P_drum = Risers_supply.P_in;
          P_drum = Downcomers_extraction.P_in;
          // Mass flow rates
          Q_fw = FW_supply.Q;
          Q_r = Risers_supply.Q;
          Q_s = risers_in.Q;
          Q_d = - downcomers_out.Q;
          // Set saturation state
          sat.psat = P_drum;
          sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          //T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          // States
          state_l = WaterSteamMedium.setState_phX(P_drum, h_l);
          state_s = WaterSteamMedium.setState_phX(P_drum, h_s);
          state_l_sat = WaterSteamMedium.setBubbleState(sat);
          state_s_sat = WaterSteamMedium.setDewState(sat);
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          //rho_b = WaterSteamMedium.density(state_s);

          // Enthalpies
          //h_s = WaterSteamMedium.specificEnthalpy(state_s);
          //h_l = WaterSteamMedium.specificEnthalpy(state_l);
          h_l_sat = WaterSteamMedium.specificEnthalpy(state_l_sat);
          h_s_sat = WaterSteamMedium.specificEnthalpy(state_l_sat);

          // Internal Energies
          u_l = h_l - P_drum/rho_l;
          u_s = h_s - P_drum/rho_s;

          // Riser steam quality
          if (h_r > h_s_sat) then
            x_r = 1;
          elseif (h_r < h_l_sat) then
            x_r = 0;
          else
            x_r = (h_r - h_l)/(h_s - h_l);
          end if;

          // Masses
          M_s = rho_s*V_s;
          M_l = rho_l*V_l;

          // Volume
          V_s = V_D - V_l;

          // Mass balance
            // Liquid phase
            Q_fw + (1 - x_r)*Q_r - Q_d - Q_b
            = V_l*(ddph_l*der(P_drum) + ddhp_l*der(h_l)) + rho_l*der(V_l);

            // Vapor phase
            Q_b + x_r*Q_r - Q_s
            = V_s*(ddph_s*der(P_drum) + ddhp_s*der(h_s)) + rho_s*der(V_s);

          // Energy balance
            // Liquid phase
            Q_fw*(h_fw - u_l) + (1 + x_r)*Q_r*((if (x_r > 0) then h_l_sat else h_r) - u_l) - Q_d*(h_d - u_l) - Q_b*(h_s_sat - u_l)
            + W_ls - W_wl
            = V_l*((P_drum/rho_l*ddph_l - 1)*der(P_drum) + (P_drum/rho_l*ddhp_l + rho_l)*der(h_l));

            // Vapor phase
            Q_b*(h_s_sat - u_s) + x_r*Q_r*((if (x_r < 1) then h_s_sat else h_r) - u_s) - Q_s*(h_s - u_s)
            - W_ls - W_ws
            = V_s*((P_drum/rho_s*ddph_s - 1)*der(P_drum) + (P_drum/rho_s*ddhp_s + rho_s)*der(h_l));

            // Wall
            //M_D*Cp_D*der(T_wall) = W_wl + W_ws;

          // Simplifications
            W_ls = 0; // To be checked
            W_wl = 0;
            W_ws = 0;
            Q_b = 0;

          // Energy Input
          h_r - h_d = W_evap/Q_r;





          // Derrivatives
          ddph_l = WaterSteamMedium.density_derp_h(state_l);
          ddph_s = WaterSteamMedium.density_derp_h(state_s);
          ddhp_l = WaterSteamMedium.density_derh_p(state_l);
          ddhp_s = WaterSteamMedium.density_derh_p(state_s);

          if Controlled_level then
            der(V_l) = 0;
          end if;

      initial equation
        der(P_drum) = 0;

      equation
        connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
        connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
        connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
        connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
        connect(Downcomers_extraction.C_out,downcomers_out)  annotation (Line(points={{0,-70},{0,-100}},         color={28,108,200}));
        connect(Downcomers_extraction.C_in,Downcomers_source. C_out) annotation (Line(points={{0,-50},{0,-33}}, color={28,108,200}));
        connect(Risers_sink.C_in,Risers_supply. C_out) annotation (Line(points={{-21,-60},{-40,-60}}, color={28,108,200}));
        connect(Risers_supply.C_in,risers_in)  annotation (Line(points={{-60,-60},{-80,-60}}, color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_TSP;

      model SteamDrum_TSP_NoCon
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;

        // Control strategy
          parameter Boolean Controlled_level = false;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;

        // Initialization parameters
        parameter Units.Pressure P_drum_0 = 125.631e5;
        parameter Modelica.Units.SI.Volume V_l_0 = 20;

        // Water/Steam properties
          // Drum pressure
          Units.Pressure P_drum(start=P_drum_0) "Drum saturation pressure";
          // Mass flow rates
          Inputs.InputPositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
          Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          Inputs.InputPositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
          Units.PositiveMassFlowRate Q_d(start=80) "Downcomers mass flow rate";
          Units.PositiveMassFlowRate Q_r(start=80) "Risers mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          //Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          Units.SpecificEnthalpy h_l_sat "Liquid enthalpy";
          Units.SpecificEnthalpy h_s_sat "Liquid enthalpy";
          Inputs.InputSpecificEnthalpy h_fw "Feedwater enthalpy";
          Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          Units.SpecificEnthalpy h_r "Risers enthalpy";
          //Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          //Units.Density rho_b "Bubble density";
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          WaterSteamMedium.ThermodynamicState state_s_sat "Steam state";
          WaterSteamMedium.ThermodynamicState state_l_sat "Liquid state";
          // Wall temperature
          //Units.Temperature T_wall;
          // Water surface tension
          //Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          //Units.Area A_ma;
          // Water level;
          //Inputs.InputHeight l_water;
          // Escape velocity
          //Units.Velocity u_s;

          // Masses
          Units.Mass M_l;
          Units.Mass M_s;

          // Evaporation heat
          Inputs.InputPower W_evap(start=104e6);

          // Set saturation state
          Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

          // Derrivatives
          Real ddph_l;
          Real ddph_s;
          Real ddhp_l;
          Real ddhp_s;

          // Riser steam quality
          Real x_r;

          // Internal Energies
          Real u_l;
          Real u_s;

          // Heat exchange
          Real W_ls;
          Real W_wl;
          Real W_ws;

      equation
        // Variables definition
          // Enthalpies:
          //h_s = Steam_source.h_out;
          //h_fw = FW_supply.h_in;
          //h_r = Risers_supply.h_in;
          //h_d = Downcomers_extraction.h_out;
          // Pressures
          //P_drum = FW_sink.P_in;
          //P_drum = Steam_extraction.P_in;
          //P_drum = Risers_supply.P_in;
          //P_drum = Downcomers_extraction.P_in;
          // Mass flow rates
          //Q_fw = FW_supply.Q;
          //Q_r = Risers_supply.Q;
          //Q_s = risers_in.Q;
          //Q_d = - downcomers_out.Q;
          // Set saturation state
          sat.psat = P_drum;
          sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          //T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          // States
          state_l = WaterSteamMedium.setState_phX(P_drum, h_l);
          state_s = WaterSteamMedium.setState_phX(P_drum, h_s);
          state_l_sat = WaterSteamMedium.setBubbleState(sat);
          state_s_sat = WaterSteamMedium.setDewState(sat);
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          //rho_b = WaterSteamMedium.density(state_s);

          // Enthalpies
          //h_s = WaterSteamMedium.specificEnthalpy(state_s);
          //h_l = WaterSteamMedium.specificEnthalpy(state_l);
          h_l_sat = WaterSteamMedium.specificEnthalpy(state_l_sat);
          h_s_sat = WaterSteamMedium.specificEnthalpy(state_l_sat);

          // Internal Energies
          u_l = h_l - P_drum/rho_l;
          u_s = h_s - P_drum/rho_s;

          // Riser steam quality
          if (h_r > h_s_sat) then
            x_r = 1;
          elseif (h_r < h_l_sat) then
            x_r = 0;
          else
            x_r = (h_r - h_l)/(h_s - h_l);
          end if;

          // Masses
          M_s = rho_s*V_s;
          M_l = rho_l*V_l;

          // Volume
          V_s = V_D - V_l;

          // Mass balance
            // Liquid phase
            Q_fw + (1 - x_r)*Q_r - Q_d - Q_b
            = V_l*(ddph_l*der(P_drum) + ddhp_l*der(h_l)) + rho_l*der(V_l);

            // Vapor phase
            Q_b + x_r*Q_r - Q_s
            = V_s*(ddph_s*der(P_drum) + ddhp_s*der(h_s)) + rho_s*der(V_s);

          // Energy balance
            // Liquid phase
            Q_fw*(h_fw - u_l) + (1 + x_r)*Q_r*((if (x_r > 0) then h_l_sat else h_r) - u_l) - Q_d*(h_d - u_l) - Q_b*(h_s_sat - u_l)
            + W_ls - W_wl
            = V_l*((P_drum/rho_l*ddph_l - 1)*der(P_drum) + (P_drum/rho_l*ddhp_l + rho_l)*der(h_l));

            // Vapor phase
            Q_b*(h_s_sat - u_s) + x_r*Q_r*((if (x_r < 1) then h_s_sat else h_r) - u_s) - Q_s*(h_s - u_s)
            - W_ls - W_ws
            = V_s*((P_drum/rho_s*ddph_s - 1)*der(P_drum) + (P_drum/rho_s*ddhp_s + rho_s)*der(h_l));

            // Wall
            //M_D*Cp_D*der(T_wall) = W_wl + W_ws;

          // Simplifications
            W_ls = 0; // To be checked
            W_wl = 0;
            W_ws = 0;
            Q_b = 0;

          // Energy Input
          Q_r*h_r - Q_d*h_d = W_evap;

          // Derrivatives
          ddph_l = WaterSteamMedium.density_derp_h(state_l);
          ddph_s = WaterSteamMedium.density_derp_h(state_s);
          ddhp_l = WaterSteamMedium.density_derh_p(state_l);
          ddhp_s = WaterSteamMedium.density_derh_p(state_s);

          if Controlled_level then
            der(V_l) = 0;
          end if;

      initial equation
        der(P_drum) = 0;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_TSP_NoCon;

      model SteamDrum_Astom_Simple_withLoop
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;

        // Control strategy
          parameter Boolean Controlled_level = false;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;

        // Initialization parameters
        parameter Units.Pressure P_drum_0 = 121.631e5;
        parameter Modelica.Units.SI.Volume V_l_0 = 20;

        // Water/Steam properties
          // Drum pressure
          Units.Pressure P_drum(start=P_drum_0) "Drum saturation pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
          //Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
          //Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          //Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
          Units.PositiveMassFlowRate Q_loop(start=80) "Evaporator loop mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          //Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
          //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          Units.SpecificEnthalpy h_r "Risers enthalpy";
          //Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          //Units.Density rho_b "Bubble density";
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          Units.Temperature T_wall;
          // Water surface tension
          //Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          //Units.Area A_ma;
          // Water level;
          //Inputs.InputHeight l_water;
          // Escape velocity
          //Units.Velocity u_s;

          // Masses
          Units.Mass M_l;
          Units.Mass M_s;

          // Evaporation heat
          Inputs.InputPower W_evap(start=104e6);

          // Set saturation state
          Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

          // Derrivatives
          Real ddph_l;
          Real ddph_s;

          // Riser steam quality
          Inputs.InputReal x_r;

        WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
        WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
        WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
        WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,-4})));
        WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,38})));
      equation
        // Variables definition
          // Enthalpies:
          h_s = Steam_source.h_out;
          h_fw = FW_supply.h_in;
          //h_l = Downcomers_extraction.h_out;
          //h_r = Risers_supply.h_in;
          // Pressures
          P_drum = FW_sink.P_in;
          P_drum = Steam_extraction.P_in;
          //P_drum = Risers_supply.P_in;
          //P_drum = Downcomers_extraction.P_in;
          // Mass flow rates
          Q_fw = FW_supply.Q;
          //Q_loop = Risers_supply.Q;
          Q_s = Steam_extraction.Q;
          //Q_loop = Downcomers_extraction.Q;
          // Set saturation state
          sat.psat = P_drum;
          sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          // States
          state_l = WaterSteamMedium.setBubbleState(sat);
          state_s = WaterSteamMedium.setDewState(sat);
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          //rho_b = WaterSteamMedium.density(state_s);

          // Enthalpies
          h_s = WaterSteamMedium.specificEnthalpy(state_s);
          h_l = WaterSteamMedium.specificEnthalpy(state_l);

          // Masses
          M_s = rho_s*V_s;
          M_l = rho_l*V_l;

          // Volume
          V_s = V_D - V_l;

          // Mass balance
          Q_fw - Q_s = V_s*(ddph_s*der(P_drum)) + rho_s*der(V_s)
                     + V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l);

          // Energy Balance
          Q_loop*(h_r - h_l)  + Q_fw*h_fw - Q_s*h_s = h_s*V_s*ddph_s*der(P_drum) + rho_s*V_s*der(h_s) + rho_s*h_s*der(V_s)
                                       + h_l*V_l*ddph_l*der(P_drum) + rho_l*V_l*der(h_l) + rho_l*h_l*der(V_l)
                                       + V_D*der(P_drum)
                                       + M_D*Cp_D*der(T_wall);

          W_evap = Q_loop*(h_r - h_l);
          x_r = (h_r - h_l)/(h_s - h_l);


          // Derrivatives
          ddph_l = WaterSteamMedium.density_derp_h(state_l);
          ddph_s = WaterSteamMedium.density_derp_h(state_s);

          if Controlled_level then
            der(V_l) = 0;
          end if;

      initial equation
        der(P_drum) = 0;

      equation
        connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
        connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
        connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
        connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_Astom_Simple_withLoop;

      model SteamDrum_Astom_Simple_withLoop_AddConnectors
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;

        // Control strategy
          parameter Boolean Controlled_level = false;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;

        // Initialization parameters
        parameter Units.Pressure P_drum_0 = 121.631e5;
        parameter Modelica.Units.SI.Volume V_l_0 = 20;

        // Water/Steam properties
          // Drum pressure
          Units.Pressure P_drum(start=P_drum_0, fixed=false) "Drum saturation pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
          //Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
          //Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          //Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
          Units.PositiveMassFlowRate Q_loop(start=80) "Evaporator loop mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          //Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
          //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          Units.SpecificEnthalpy h_r "Risers enthalpy";
          //Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          //Units.Density rho_b "Bubble density";
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          Units.Temperature T_wall;
          // Water surface tension
          //Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          //Units.Area A_ma;
          // Water level;
          //Inputs.InputHeight l_water;
          // Escape velocity
          //Units.Velocity u_s;

          // Masses
          Units.Mass M_l;
          Units.Mass M_s;

          // Evaporation heat
          Units.Power W_evap(start=104e6);

          // Set saturation state
          Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

          // Derrivatives
          Real ddph_l;
          Real ddph_s;

          // Riser steam quality
          Inputs.InputReal x_r;

        WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
        WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
        WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
        WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,-4})));
        WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,38})));
        WaterSteam.BaseClasses.IsoPHFlowModel Downcomers_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-60})));
        WaterSteam.BoundaryConditions.Sink Risers_sink annotation (Placement(transformation(extent={{-26,-70},{-6,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel Risers_supply annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
        WaterSteam.BoundaryConditions.Source Downcomers_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-28})));
        WaterSteam.Connectors.Outlet downcomers_out annotation (Placement(transformation(extent={{-10,-110},{10,-90}}), iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Inlet risers_in annotation (Placement(transformation(extent={{-90,-70},{-70,-50}}), iconTransformation(extent={{-90,-70},{-70,-50}})));
      equation
        // Variables definition
          // Enthalpies:
          h_s = Steam_source.h_out;
          h_fw = FW_supply.h_in;
          h_l = Downcomers_extraction.h_out;
          h_r = Risers_supply.h_in;
          // Pressures
          P_drum = FW_sink.P_in;
          P_drum = Steam_extraction.P_in;
          // P_drum = Risers_supply.P_in;
          P_drum = Downcomers_extraction.P_in;
          // Mass flow rates
          Q_fw = FW_supply.Q;
          Q_loop = Risers_supply.Q;
          Q_s = Steam_extraction.Q;
          //Q_loop = Downcomers_extraction.Q;
          // Set saturation state
          sat.psat = P_drum;
          sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          // States
          state_l = WaterSteamMedium.setBubbleState(sat);
          state_s = WaterSteamMedium.setDewState(sat);
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          //rho_b = WaterSteamMedium.density(state_s);

          // Enthalpies
          h_s = WaterSteamMedium.specificEnthalpy(state_s);
          h_l = WaterSteamMedium.specificEnthalpy(state_l);

          // Masses
          M_s = rho_s*V_s;
          M_l = rho_l*V_l;

          // Volume
          V_s = V_D - V_l;

          // Mass balance
          Q_fw - Q_s = V_s*(ddph_s*der(P_drum)) + rho_s*der(V_s)
                     + V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l);

          // Energy Balance
          Q_loop*(h_r - h_l)  + Q_fw*h_fw - Q_s*h_s = h_s*V_s*ddph_s*der(P_drum) + rho_s*V_s*der(h_s) + rho_s*h_s*der(V_s)
                                       + h_l*V_l*ddph_l*der(P_drum) + rho_l*V_l*der(h_l) + rho_l*h_l*der(V_l)
                                       + V_D*der(P_drum)
                                       + M_D*Cp_D*der(T_wall);

          W_evap = Q_loop*(h_r - h_l);
          x_r = (h_r - h_l)/(h_s - h_l);


          // Derrivatives
          ddph_l = WaterSteamMedium.density_derp_h(state_l);
          ddph_s = WaterSteamMedium.density_derp_h(state_s);


      initial equation
        //der(P_drum) = 0;
        //der(V_l) = 0;
        //W_evap = Q_s*h_s - Q_fw*h_fw;


      equation
        connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
        connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
        connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
        connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
        connect(Downcomers_extraction.C_out,downcomers_out)  annotation (Line(points={{0,-70},{0,-100}},         color={28,108,200}));
        connect(Downcomers_extraction.C_in,Downcomers_source. C_out) annotation (Line(points={{0,-50},{0,-33}}, color={28,108,200}));
        connect(Risers_sink.C_in,Risers_supply. C_out) annotation (Line(points={{-21,-60},{-40,-60}}, color={28,108,200}));
        connect(Risers_supply.C_in,risers_in)  annotation (Line(points={{-60,-60},{-80,-60}}, color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_Astom_Simple_withLoop_AddConnectors;

      model SteamDrum_Astom_Sunil_Prog
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;

        // Control strategy
          parameter Boolean Controlled_level = false;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;
          parameter Real K_tuning = 1;

        // Initialization parameters
        parameter Units.Pressure P_drum_0 = 121.631e5;
        parameter Modelica.Units.SI.Volume V_l_0 = 20;

        // Water/Steam properties
          // Drum pressure
          Units.Pressure P_drum(start=P_drum_0) "Drum saturation pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
          Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
          //Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          //Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
          Units.PositiveMassFlowRate Q_loop(start=80) "Evaporator loop mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          Modelica.Units.SI.Volume V_b "Bubble volume";
          Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
          //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          Units.SpecificEnthalpy h_r "Risers enthalpy";
          Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          Units.Density rho_b "Bubble density";
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          Units.Temperature T_wall;
          // Water surface tension
          //Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          //Units.Area A_ma;
          // Water level;
          //Inputs.InputHeight l_water;
          // Escape velocity
          //Units.Velocity u_s;

          // Masses
          Units.Mass M_l;
          Units.Mass M_s;

          // Evaporation heat
          Units.Power W_evap(start=104e6);

          // Set saturation state
          Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

          // Derrivatives
          Real ddph_l;
          Real ddph_s;
          Real ddph_b;

          // Riser steam quality
          Inputs.InputReal x_r;

          // Water level;
          Units.Height l_water(start=1.2, fixed=true);

          // Water surface tension
          Modelica.Units.SI.SurfaceTension sigma_s;
          //Interface Area
          Units.Area A_ma;
          // Escape velocity
          Units.Velocity u_s;



        WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
        WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
        WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
        WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,-4})));
        WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,38})));
        WaterSteam.BaseClasses.IsoPHFlowModel Downcomers_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-60})));
        WaterSteam.BoundaryConditions.Sink Risers_sink annotation (Placement(transformation(extent={{-26,-70},{-6,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel Risers_supply annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
        WaterSteam.BoundaryConditions.Source Downcomers_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-28})));
        WaterSteam.Connectors.Outlet downcomers_out annotation (Placement(transformation(extent={{-10,-110},{10,-90}}), iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Inlet risers_in annotation (Placement(transformation(extent={{-90,-70},{-70,-50}}), iconTransformation(extent={{-90,-70},{-70,-50}})));
      equation
        // Variables definition
          // Enthalpies:
          h_s = Steam_source.h_out;
          h_fw = FW_supply.h_in;
          h_l = Downcomers_extraction.h_out;
          h_r = Risers_supply.h_in;
          // Pressures
          P_drum = FW_sink.P_in;
          P_drum = Steam_extraction.P_in;
          // P_drum = Risers_supply.P_in;
          P_drum = Downcomers_extraction.P_in;
          // Mass flow rates
          Q_fw = FW_supply.Q;
          Q_loop = Risers_supply.Q;
          Q_s = Steam_extraction.Q;
          //Q_loop = Downcomers_extraction.Q;
          // Set saturation state
          sat.psat = P_drum;
          sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          // States
          state_l = WaterSteamMedium.setBubbleState(sat);
          state_s = WaterSteamMedium.setDewState(sat);
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          rho_b = WaterSteamMedium.density(state_s);

          // Enthalpies
          h_s = WaterSteamMedium.specificEnthalpy(state_s);
          h_l = WaterSteamMedium.specificEnthalpy(state_l);
          h_b = WaterSteamMedium.specificEnthalpy(state_s); // Bubble are computed at saturated pressure

          // Masses
          M_s = rho_s*V_s;
          M_l = rho_l*V_l;

          // Volume
          V_D = V_s + V_l + V_b;
          V_l + V_b = (R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5)*L_D;


          // Total mass balance
          Q_fw - Q_s = V_s*(ddph_s*der(P_drum)) + rho_s*der(V_s)
                      + V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l)
                      + V_b*(ddph_b*der(P_drum)) + rho_b*der(V_b);


          // Mass balance on control volume with liquid and bubble region:
          Q_fw - Q_b = V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l)
                                 + V_b*(ddph_b*der(P_drum)) + rho_b*der(V_b);


          // Energy Balance
          Q_loop*(h_r - h_l)  + Q_fw*h_fw - Q_s*h_s = h_s*V_s*ddph_s*der(P_drum) + rho_s*V_s*der(h_s) + rho_s*h_s*der(V_s)
                                       + h_l*V_l*ddph_l*der(P_drum) + rho_l*V_l*der(h_l) + rho_l*h_l*der(V_l)
                                       + V_D*der(P_drum)
                                       + M_D*Cp_D*der(T_wall);

          W_evap = Q_loop*(h_r - h_l);
          x_r = (h_r - h_l)/(h_s - h_l);

          // Bubble mass flow rate
          // Surface tension
          sigma_s = WaterSteamMedium.surfaceTension(sat);
          // Interface area
          A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
          // Escape velocity
          u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4);
          // Bubble mass flow rate
          Q_b = K_tuning*A_ma*u_s*rho_s;

          // Derrivatives
          ddph_l = WaterSteamMedium.density_derp_h(state_l);
          ddph_s = WaterSteamMedium.density_derp_h(state_s);
          ddph_b = WaterSteamMedium.density_derp_h(state_s);

          if Controlled_level then
            der(V_l + V_b) = 0;
          end if;

      initial equation
        der(P_drum) = 0;

      equation
        connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
        connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
        connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
        connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
        connect(Downcomers_extraction.C_out,downcomers_out)  annotation (Line(points={{0,-70},{0,-100}},         color={28,108,200}));
        connect(Downcomers_extraction.C_in,Downcomers_source. C_out) annotation (Line(points={{0,-50},{0,-33}}, color={28,108,200}));
        connect(Risers_sink.C_in,Risers_supply. C_out) annotation (Line(points={{-21,-60},{-40,-60}}, color={28,108,200}));
        connect(Risers_supply.C_in,risers_in)  annotation (Line(points={{-60,-60},{-80,-60}}, color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_Astom_Sunil_Prog;

      model SteamDrum_Astom_Sunil_Prog_1
         import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        import MetroscopeModelingLibrary.Utilities.Constants;
        import CorrelationConstants =
               MetroscopeModelingLibrary.DynamicComponents.Correlations;

        package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
        package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

        // Parameter
        parameter Boolean steady_state = false;

        // Constants
          parameter Real pi = Constants.pi;
          parameter Real g = Constants.g;

        // Control strategy
          parameter Boolean Controlled_level = false;

        // Drum dimensions
          parameter Modelica.Units.SI.Volume V_D = 55.8;
          parameter Units.Mass M_D = 111030;
          parameter Units.HeatCapacity Cp_D = 232;
          parameter Units.Length D_D = 2.6;
          parameter Units.Length R_D = D_D/2;
          parameter Units.Length L_D = 11.160;
          parameter Real K_tuning = 1;

        // Initialization parameters
        parameter Units.Pressure P_drum_0 = 121.631e5;
        parameter Modelica.Units.SI.Volume V_l_0 = 20;

        // Water/Steam properties
          // Drum pressure
          Units.Pressure P_drum(start=P_drum_0) "Drum saturation pressure";
          // Mass flow rates
          Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
          Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
          Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
          //Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
          //Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
          Units.PositiveMassFlowRate Q_loop(start=80) "Evaporator loop mass flow rate";
          // Volumes
          Modelica.Units.SI.Volume V_s "Steam volume";
          Modelica.Units.SI.Volume V_b(start=5, fixed=false) "Bubble volume";
          Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume";
          Modelica.Units.SI.Volume V_lb  "Bubble and liquid volume";
          // Enthalpies
          Units.SpecificEnthalpy h_s "Steam enthalpy";
          Units.SpecificEnthalpy h_l "Liquid enthalpy";
          Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
          //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
          Units.SpecificEnthalpy h_r "Risers enthalpy";
          Units.SpecificEnthalpy h_b "Bubble enthalpy";
          // Densities
          Units.Density rho_s "Steam density";
          Units.Density rho_l "Liquid density";
          Units.Density rho_b "Bubble density";
          // States
          WaterSteamMedium.ThermodynamicState state_s "Steam state";
          WaterSteamMedium.ThermodynamicState state_l "Liquid state";
          // Wall temperature
          Units.Temperature T_wall;
          // Water surface tension
          //Modelica.Units.SI.SurfaceTension sigma_s;
          // Water mass fraction
          //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
          // Interface Area
          //Units.Area A_ma;
          // Water level;
          //Inputs.InputHeight l_water;
          // Escape velocity
          //Units.Velocity u_s;

          // Masses
          Units.Mass M_l;
          Units.Mass M_s;

          // Evaporation heat
          Units.Power W_evap(start=104e6);

          // Set saturation state
          Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

          // Derrivatives
          Real ddph_l;
          Real ddph_s;
          Real ddph_b;

          // Riser steam quality
          Inputs.InputReal x_r;

          // Water level;
          Units.Height l_water(start=1.2);

          // Water surface tension
          Modelica.Units.SI.SurfaceTension sigma_s;
          //Interface Area
          Units.Area A_ma;
          // Escape velocity
          Units.Velocity u_s;


        WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
        WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
        WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
        WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,-4})));
        WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,38})));
        WaterSteam.BaseClasses.IsoPHFlowModel Downcomers_extraction annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-60})));
        WaterSteam.BoundaryConditions.Sink Risers_sink annotation (Placement(transformation(extent={{-26,-70},{-6,-50}})));
        WaterSteam.BaseClasses.IsoPHFlowModel Risers_supply annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
        WaterSteam.BoundaryConditions.Source Downcomers_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=270,
              origin={0,-28})));
        WaterSteam.Connectors.Outlet downcomers_out annotation (Placement(transformation(extent={{-10,-110},{10,-90}}), iconTransformation(extent={{-10,-110},{10,-90}})));
        WaterSteam.Connectors.Inlet risers_in annotation (Placement(transformation(extent={{-90,-70},{-70,-50}}), iconTransformation(extent={{-90,-70},{-70,-50}})));
      equation
        // Variables definition
          // Enthalpies:
          h_s = Steam_source.h_out;
          h_fw = FW_supply.h_in;
          h_l = Downcomers_extraction.h_out;
          h_r = Risers_supply.h_in;
          // Pressures
          P_drum = FW_sink.P_in;
          P_drum = Steam_extraction.P_in;
          // P_drum = Risers_supply.P_in;
          P_drum = Downcomers_extraction.P_in;
          // Mass flow rates
          Q_fw = FW_supply.Q;
          Q_loop = Risers_supply.Q;
          Q_s = Steam_extraction.Q;
          //Q_loop = Downcomers_extraction.Q;
          // Set saturation state
          sat.psat = P_drum;
          sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

          // States
          state_l = WaterSteamMedium.setBubbleState(sat);
          state_s = WaterSteamMedium.setDewState(sat);
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_l = WaterSteamMedium.density(state_l);
          rho_b = WaterSteamMedium.density(state_s);

          // Enthalpies
          h_s = WaterSteamMedium.specificEnthalpy(state_s);
          h_l = WaterSteamMedium.specificEnthalpy(state_l);
          h_b = WaterSteamMedium.specificEnthalpy(state_s); // Bubble are computed at saturated pressure

          // Masses
          M_s = rho_s*V_s;
          M_l = rho_l*V_l;

          // Volume
          V_D = V_s + V_l + V_b;
          V_lb = V_l + V_b;
          V_lb = (R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5)*L_D;

          // Total mass balance
          Q_fw - Q_s = V_s*(ddph_s*der(P_drum)) + rho_s*der(V_s)
                      + V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l)
                      + V_b*(ddph_b*der(P_drum)) + rho_b*der(V_b);

          // Mass balance on control volume with liquid and bubble region:
          Q_fw - Q_b = V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l)
                     + V_b*(ddph_b*der(P_drum)) + rho_b*der(V_b);

      //     // Energy Balance
      //     Q_loop*(h_r - h_l)  + Q_fw*h_fw - Q_s*h_s = h_s*V_s*ddph_s*der(P_drum) + rho_s*V_s*der(h_s) + rho_s*h_s*der(V_s)
      //                                  + h_l*V_l*ddph_l*der(P_drum) + rho_l*V_l*der(h_l) + rho_l*h_l*der(V_l)
      //                                  + V_D*der(P_drum)
      //                                  + M_D*Cp_D*der(T_wall);

          Q_loop*(h_r - h_l) + Q_fw*h_fw - Q_s*h_s = h_l*V_l*ddph_l*der(P_drum) + rho_l*V_l*der(h_l)
                                                  + V_D*h_s*ddph_s*der(P_drum) + rho_s*V_D*der(h_s)
                                                  - V_l*h_s*ddph_s*der(P_drum)
                                                  + der(V_l)*(rho_l*h_l - rho_s*h_s)
                                                  + M_D*Cp_D*der(T_wall);

          W_evap = Q_loop*(h_r - h_l);
          x_r = (h_r - h_l)/(h_s - h_l);

          // Bubble mass flow rate
          // Surface tension
          sigma_s = WaterSteamMedium.surfaceTension(sat);
          // Interface area
          A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
          // Escape velocity
          u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4);
          // Bubble mass flow rate
          Q_b = K_tuning*A_ma*u_s*rho_s;

          // Derrivatives
          ddph_l = WaterSteamMedium.density_derp_h(state_l);
          ddph_s = WaterSteamMedium.density_derp_h(state_s);
          ddph_b = WaterSteamMedium.density_derp_h(state_s);




          if Controlled_level then
            der(V_lb) = 0;
          end if;

      initial equation

        if steady_state then
            der(V_b) = 0;
            //der(V_s) = 0;
            //der(V_b) = 0;
            //der(T_wall) = 0;
            der(P_drum) = 0;
            // Q_s = Q_fw;
        else
          der(V_b) = 0;

          end if;


      //   der(P_drum) = 0;
      //   Q_fw = Q_s;
      //   der(V_b) = 0;
      //   //W_evap = Q_s*h_s - Q_fw*h_fw;
      //   //Q_b = Q_s;


      equation
        connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
        connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
        connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
        connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
        connect(Downcomers_extraction.C_out,downcomers_out)  annotation (Line(points={{0,-70},{0,-100}},         color={28,108,200}));
        connect(Downcomers_extraction.C_in,Downcomers_source. C_out) annotation (Line(points={{0,-50},{0,-33}}, color={28,108,200}));
        connect(Risers_sink.C_in,Risers_supply. C_out) annotation (Line(points={{-21,-60},{-40,-60}}, color={28,108,200}));
        connect(Risers_supply.C_in,risers_in)  annotation (Line(points={{-60,-60},{-80,-60}}, color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Polygon(
                points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                lineThickness=1,
                smooth=Smooth.Bezier,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineColor={85,170,255}),
              Ellipse(
                extent={{-100,100},{100,-100}},
                lineColor={135,135,135},
                lineThickness=1),
              Line(
                points={{-32,-20}},
                color={135,135,135},
                thickness=1),
              Ellipse(
                extent={{-48,-60},{-40,-68}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-26,-52},{-18,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-38},{-2,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-10,-62},{-2,-70}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-46,-38},{-38,-46}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{6,-52},{14,-60}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{28,-58},{36,-66}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{10,-72},{18,-80}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-32,-76},{-24,-84}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-78,-32},{-70,-40}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                lineColor={0,0,0}),
              Ellipse(
                extent={{-42,-26},{-34,-34}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-18,-16},{-10,-24}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,-14},{-22,-22}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{12,-24},{20,-32}},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end SteamDrum_Astom_Sunil_Prog_1;

      package SteamDrumModels
        model SteamDrum_Astom_1
           import MetroscopeModelingLibrary.Utilities.Units;
          import MetroscopeModelingLibrary.Utilities.Units.Inputs;
          import MetroscopeModelingLibrary.Utilities.Constants;
          import CorrelationConstants =
                 MetroscopeModelingLibrary.DynamicComponents.Correlations;

          package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
          package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

          // Constants
            parameter Real pi = Constants.pi;
            parameter Real g = Constants.g;

          // Control strategy
            parameter Boolean Controlled_level = false;

          // Drum dimensions
            parameter Modelica.Units.SI.Volume V_D = 55.8;
            parameter Units.Mass M_D = 111030;
            parameter Units.HeatCapacity Cp_D = 232;
            parameter Units.Length D_D = 2.6;
            parameter Units.Length R_D = D_D/2;
            parameter Units.Length L_D = 11.160;

          // Initialization parameters
          parameter Units.Pressure P_drum_0 = 121.631e5;
          parameter Modelica.Units.SI.Volume V_l_0 = 20;

          // Water/Steam properties
            // Drum pressure
            Units.Pressure P_drum(start=P_drum_0) "Drum saturation pressure";
            // Mass flow rates
            Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
            //Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
            Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
            //Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
            //Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
            // Volumes
            Modelica.Units.SI.Volume V_s "Steam volume";
            //Modelica.Units.SI.Volume V_b "Bubble volume";
            Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
            // Enthalpies
            Units.SpecificEnthalpy h_s "Steam enthalpy";
            Units.SpecificEnthalpy h_l "Liquid enthalpy";
            Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
            //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
            //Inputs.InputSpecificEnthalpy h_r "Risers enthalpy";
            //Units.SpecificEnthalpy h_b "Bubble enthalpy";
            // Densities
            Units.Density rho_s "Steam density";
            Units.Density rho_l "Liquid density";
            //Units.Density rho_b "Bubble density";
            // States
            WaterSteamMedium.ThermodynamicState state_s "Steam state";
            WaterSteamMedium.ThermodynamicState state_l "Liquid state";
            // Wall temperature
            Units.Temperature T_wall;
            // Water surface tension
            //Modelica.Units.SI.SurfaceTension sigma_s;
            // Water mass fraction
            //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
            // Interface Area
            //Units.Area A_ma;
            // Water level;
            //Inputs.InputHeight l_water;
            // Escape velocity
            //Units.Velocity u_s;

            // Masses
            Units.Mass M_l;
            Units.Mass M_s;

            // Evaporation heat
            Inputs.InputPower W_evap(start=104e6);

            // Set saturation state
            Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

            // Derrivatives
            Real ddph_l;
            Real ddph_s;


          WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
          WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
          WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
          WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
          WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,-4})));
          WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,38})));
        equation
          // Variables definition
            // Enthalpies:
            h_s = Steam_source.h_out;
            h_fw = FW_supply.h_in;
            // Pressures
            P_drum = FW_sink.P_in;
            P_drum = Steam_extraction.P_in;
            // Mass flow rates
            Q_fw = FW_supply.Q;
            //Q_r = Risers_supply.Q;
            Q_s = Steam_extraction.Q;
            //Q_d = Downcomers_extraction.Q;
            // Set saturation state
            sat.psat = P_drum;
            sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

            T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

            // States
            state_l = WaterSteamMedium.setBubbleState(sat);
            state_s = WaterSteamMedium.setDewState(sat);
            // Surface tension
            //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
            // Densities
            rho_s = WaterSteamMedium.density(state_s);
            rho_l = WaterSteamMedium.density(state_l);
            //rho_b = WaterSteamMedium.density(state_s);

            // Enthalpies
            h_s = WaterSteamMedium.specificEnthalpy(state_s);
            h_l = WaterSteamMedium.specificEnthalpy(state_l);

            // Masses
            M_s = rho_s*V_s;
            M_l = rho_l*V_l;

            // Volume
            V_s = V_D - V_l;

            // Mass balance
            Q_fw - Q_s = V_s*(ddph_s*der(P_drum)) + rho_s*der(V_s)
                       + V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l);

            // Energy Balance
            W_evap + Q_fw*h_fw - Q_s*h_s = h_s*V_s*ddph_s*der(P_drum) + rho_s*V_s*der(h_s) + rho_s*h_s*der(V_s)
                                         + h_l*V_l*ddph_l*der(P_drum) + rho_l*V_l*der(h_l) + rho_l*h_l*der(V_l)
                                         + V_D*der(P_drum)
                                         + M_D*Cp_D*der(T_wall);

            // Derrivatives
            ddph_l = WaterSteamMedium.density_derp_h(state_l);
            ddph_s = WaterSteamMedium.density_derp_h(state_s);

            if Controlled_level then
              der(V_l) = 0;
            end if;

        initial equation
          der(P_drum) = 0;

        equation
          connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
          connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
          connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
          connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Polygon(
                  points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                  lineThickness=1,
                  smooth=Smooth.Bezier,
                  fillColor={85,170,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={85,170,255}),
                Ellipse(
                  extent={{-100,100},{100,-100}},
                  lineColor={135,135,135},
                  lineThickness=1),
                Line(
                  points={{-32,-20}},
                  color={135,135,135},
                  thickness=1),
                Ellipse(
                  extent={{-48,-60},{-40,-68}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-26,-52},{-18,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-38},{-2,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-62},{-2,-70}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-46,-38},{-38,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{6,-52},{14,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{28,-58},{36,-66}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{10,-72},{18,-80}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-32,-76},{-24,-84}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-78,-32},{-70,-40}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-42,-26},{-34,-34}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-18,-16},{-10,-24}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-30,-14},{-22,-22}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{12,-24},{20,-32}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
        end SteamDrum_Astom_1;

        model SteamDrum_Astom_2
           import MetroscopeModelingLibrary.Utilities.Units;
          import MetroscopeModelingLibrary.Utilities.Units.Inputs;
          import MetroscopeModelingLibrary.Utilities.Constants;
          import CorrelationConstants =
                 MetroscopeModelingLibrary.DynamicComponents.Correlations;

          package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
          package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

          // Constants
            parameter Real pi = Constants.pi;
            parameter Real g = Constants.g;

          // Control strategy
            parameter Boolean Controlled_level = false;

          // Drum dimensions
            parameter Modelica.Units.SI.Volume V_D = 55.8;
            parameter Units.Mass M_D = 111030;
            //parameter Units.HeatCapacity Cp_D = 232;
            parameter Units.Length D_D = 2.6;
            parameter Units.Length R_D = D_D/2;
            parameter Units.Length L_D = 11.160;

          // Evaporator
            parameter Units.Length D_r = 38.1e-3;
            parameter Units.Area A_r = 0.25*pi*D_r*N_tubes_r;
            parameter Units.Length L_r = 18.29;
            parameter Integer N_tubes_r = 1472; // P&ID 1560
            parameter Modelica.Units.SI.Volume V_r = A_r*L_r;
            parameter Units.Mass M_r = 159376;
            parameter Units.HeatCapacity Cp = 472;

          // Downcomers
            parameter Units.Length D_o_dc = 505e-3;
            parameter Units.Length e_dc = 38.1e-3;
            parameter Units.Length L_dc = 18.29;
            parameter Units.Density rho_steel = 7850;
            parameter Units.Area A_dc = N_dc*0.25*pi*(D_o_dc - 2*e_dc)^2;
            parameter Integer N_dc = 3;
            parameter Modelica.Units.SI.Volume V_dc = A_dc*L_dc;
            parameter Units.Mass M_dc = N_dc*0.25*pi*(D_o_dc^2 - (D_o_dc - 2*e_dc)^2)*L_dc*rho_steel;
            parameter Real k_friction = 25;

          // Total mass and volume
            parameter Units.Mass M_t = M_D + M_r + M_dc;
            parameter Modelica.Units.SI.Volume V_t = V_D + V_r + V_dc;

          // Initialization parameters
          parameter Units.Pressure P_drum_0 = 121.631e5;
          parameter Modelica.Units.SI.Volume V_wt_0 = 20;

          // Water/Steam properties
            // Drum pressure
            Units.Pressure P_drum(start=P_drum_0) "Drum saturation pressure";
            // Mass flow rates
            Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
            Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
            Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
            Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
            Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
            // Volumes
            Modelica.Units.SI.Volume V_st "Steam volume";
            Modelica.Units.SI.Volume V_b "Bubble volume";
            Modelica.Units.SI.Volume V_wt(start=V_wt_0, fixed=true) "Total water volume under the level"; //
            Modelica.Units.SI.Volume V_wd "Water volume in the drum";
            //Modelica.Units.SI.Volume V_l "Liquid volume";
            // Enthalpies
            Units.SpecificEnthalpy h_s "Steam enthalpy";
            Units.SpecificEnthalpy h_l "Liquid enthalpy";
            Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
            Units.SpecificEnthalpy h_c "Condensation enthalpy";
            //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
            //Inputs.InputSpecificEnthalpy h_r "Risers enthalpy";
            //Units.SpecificEnthalpy h_b "Bubble enthalpy";
            // Densities
            Units.Density rho_s "Steam density";
            Units.Density rho_l "Liquid density";
            //Units.Density rho_b "Bubble density";
            // States
            WaterSteamMedium.ThermodynamicState state_s "Steam state";
            WaterSteamMedium.ThermodynamicState state_l "Liquid state";
            // Wall temperature
            Units.Temperature T_wall;
            // Water surface tension
            //Modelica.Units.SI.SurfaceTension sigma_s;
            // Water mass fraction
            //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
            // Interface Area
            //Units.Area A_ma;
            // Water level;
            //Inputs.InputHeight l_water;
            // Escape velocity
            //Units.Velocity u_s;

            // Masses
            //Units.Mass M_l;
            //Units.Mass M_s;

            // Evaporation heat
            Inputs.InputPower W_evap(start=104e6);

            // Set saturation state
            Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

            // Derrivatives
            Real ddph_l;
            Real ddph_s;

            // Riser mass and volume fractions
            Real x_r; // Steam mass fraction of the riser
            Real x_vr_mean; // Steam mean volume fraction in the riser

          WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
          WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
          WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
          WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
          WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,-4})));
          WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,38})));
        equation
          // Variables definition
            // Enthalpies:
            h_s = Steam_source.h_out;
            h_fw = FW_supply.h_in;
            // Pressures
            P_drum = FW_sink.P_in;
            P_drum = Steam_extraction.P_in;
            // Mass flow rates
            Q_fw = FW_supply.Q;
            //Q_r = Risers_supply.Q;
            Q_s = Steam_extraction.Q;
            //Q_d = Downcomers_extraction.Q;
            // Set saturation state
            sat.psat = P_drum;
            sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

            T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

            // States
            state_l = WaterSteamMedium.setBubbleState(sat);
            state_s = WaterSteamMedium.setDewState(sat);
            // Surface tension
            //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
            // Densities
            rho_s = WaterSteamMedium.density(state_s);
            rho_l = WaterSteamMedium.density(state_l);
            //rho_b = WaterSteamMedium.density(state_s);

            // Enthalpies
            h_s = WaterSteamMedium.specificEnthalpy(state_s);
            h_l = WaterSteamMedium.specificEnthalpy(state_l);
            h_c = h_s - h_l;

            // Masses
            //M_s = rho_s*V_st;
            //M_l = rho_l*V_wt;

            // Volume
            V_t = V_st + V_wt;
            V_wd = V_wt - V_dc - (1 - x_vr_mean)*V_r;
            //V_l = V_wt - V_b;


            // Steam mean volume fraction in risers
            x_vr_mean = rho_l/(rho_l - rho_s)*(1 - rho_s/((rho_l - rho_s)*x_r)*log(1 + (rho_l - rho_s)*x_r/rho_s));

            // Mass balance
            Q_fw - Q_s = V_st*(ddph_s*der(P_drum)) + rho_s*der(V_st)
                       + V_wt*(ddph_l*der(P_drum)) + rho_l*der(V_wt);

            // Energy Balance
            W_evap + Q_fw*h_fw - Q_s*h_s = h_s*V_st*ddph_s*der(P_drum) + rho_s*V_st*der(h_s) + rho_s*h_s*der(V_st)
                                         + h_l*V_wt*ddph_l*der(P_drum) + rho_l*V_wt*der(h_l) + rho_l*h_l*der(V_wt)
                                         + V_t*der(P_drum)
                                         + M_t*Cp*der(T_wall);

            // Mass balance in riser section
            Q_d - Q_r = x_vr_mean*V_r*ddph_s*der(P_drum) + rho_s*V_r*der(x_vr_mean)
                      + V_r*ddph_l*der(P_drum)
                      - x_vr_mean*V_r*ddph_l*der(P_drum) - rho_l*V_r*der(x_vr_mean);

            // Energy balance in the riser section
            W_evap + Q_d*h_l - (x_r*h_c + h_l)*Q_r = h_s*x_vr_mean*V_r*ddph_s*der(P_drum) + rho_s*h_s*V_r*der(x_vr_mean) + rho_s*x_vr_mean*V_r*der(h_s)
                                                   + h_l*V_r*ddph_l*der(P_drum) + rho_l*V_r*der(h_l)
                                                   - h_l*x_vr_mean*V_r*ddph_l*der(P_drum) - rho_l*h_l*V_r*der(x_vr_mean) - rho_l*x_vr_mean*V_r*der(h_l)
                                                   - V_r*der(P_drum)
                                                   + M_r*Cp*der(T_wall);

            // Mass balance for the steam under the liquid level
            x_r*Q_r - Q_b = ddph_s*der(P_drum)*V_b + rho_s*der(V_b);

            // Mass flow rate in the downcomers
            0.5*k_friction*Q_d^2 = rho_l*A_dc*(rho_l - rho_s)*g*x_vr_mean*V_r;



            // Derrivatives
            ddph_l = WaterSteamMedium.density_derp_h(state_l);
            ddph_s = WaterSteamMedium.density_derp_h(state_s);



            if Controlled_level then
              der(V_wt) = 0;
            end if;

        initial equation
          der(P_drum) = 0;

        equation
          connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
          connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
          connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
          connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Polygon(
                  points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                  lineThickness=1,
                  smooth=Smooth.Bezier,
                  fillColor={85,170,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={85,170,255}),
                Ellipse(
                  extent={{-100,100},{100,-100}},
                  lineColor={135,135,135},
                  lineThickness=1),
                Line(
                  points={{-32,-20}},
                  color={135,135,135},
                  thickness=1),
                Ellipse(
                  extent={{-48,-60},{-40,-68}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-26,-52},{-18,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-38},{-2,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-62},{-2,-70}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-46,-38},{-38,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{6,-52},{14,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{28,-58},{36,-66}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{10,-72},{18,-80}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-32,-76},{-24,-84}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-78,-32},{-70,-40}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-42,-26},{-34,-34}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-18,-16},{-10,-24}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-30,-14},{-22,-22}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{12,-24},{20,-32}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
        end SteamDrum_Astom_2;

        model SteamDrum_Astom_SimplifiedGeometry_3
           import MetroscopeModelingLibrary.Utilities.Units;
          import MetroscopeModelingLibrary.Utilities.Units.Inputs;
          import MetroscopeModelingLibrary.Utilities.Constants;
          import CorrelationConstants =
                 MetroscopeModelingLibrary.DynamicComponents.Correlations;

          package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
          package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

          // Constants
            parameter Real pi = Constants.pi;
            parameter Real g = Constants.g;

          // Control strategy
            parameter Boolean Controlled_wevel = false;

          // Drum dimensions
            parameter Modelica.Units.SI.Volume V_D = 40;
            parameter Units.Mass M_d = M_t - M_r;
            //parameter Units.HeatCapacity Cp_D = 232;
            //parameter Units.Length D_D = 2.6;
            //parameter Units.Length R_D = D_D/2;
            //parameter Units.Length L_D = 11.160;

          // Evaporator
            //parameter Units.Length D_r = 38.1e-3;
            //parameter Units.Area A_r = 0.25*pi*D_r*N_tubes_r;
            //parameter Units.Length L_r = 18.29;
            //parameter Integer N_tubes_r = 1472; // P&ID 1560
            parameter Modelica.Units.SI.Volume V_r = 37;
            parameter Units.Mass M_r = 160000;
            parameter Units.HeatCapacity Cp = 472;

          // Downcomers
            //parameter Units.Length D_o_dc = 505e-3;
            //parameter Units.Length e_dc = 38.1e-3;
            //parameter Units.Length L_dc = 18.29;
            //parameter Units.Density rho_steel = 7850;
            Units.Area A_dc; //  = 0.08161446
            parameter Units.Area A_d = 20;
            //parameter Integer N_dc = 3;
            parameter Modelica.Units.SI.Volume V_dc = 11;
            //parameter Units.Mass M_dc = N_dc*0.25*pi*(D_o_dc^2 - (D_o_dc - 2*e_dc)^2)*L_dc*rho_steel;
            parameter Real k_friction = 25;

            parameter Real beta = 0.3;

          // Total mass and volume
            parameter Units.Mass M_t = 300000;
            parameter Modelica.Units.SI.Volume V_t = V_D + V_r + V_dc;
            parameter Real T_d = 12;

          // Initialization parameters
          parameter Units.Pressure p_0 = 85e5;
          parameter Modelica.Units.SI.Volume V_wt_0 = 57.1;

          // Water/Steam properties
            // Drum pressure
            Units.Pressure p(start=p_0) "Drum saturation pressure";
            // Mass flow rates
            Units.PositiveMassFlowRate Q_f "Feed water mass flow rate";
            Units.PositiveMassFlowRate Q_sd "Bubbles mass flow rate";
            Units.PositiveMassFlowRate Q_s "Steam mass flow rate";
            Units.PositiveMassFlowRate Q_dc "Downcomers mass flow rate";
            Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
            Units.PositiveMassFlowRate Q_cd "Condensation mass flow rate in the drum";
            Units.PositiveMassFlowRate Q_ct "Total condensation mass flow rate";
            // Volumes
            Modelica.Units.SI.Volume V_st "Steam volume";
            Modelica.Units.SI.Volume V_sd(start=4.8) "Bubble volume";
            Modelica.Units.SI.Volume V_0_sd "Hypothetical volume of bubbles";
            Modelica.Units.SI.Volume V_wt(start=V_wt_0, fixed=true) "Total water volume under the level"; //
            Modelica.Units.SI.Volume V_wd "Water volume in the drum";
            //Modelica.Units.SI.Volume V_w "Liquid volume";
            // Enthalpies
            Units.SpecificEnthalpy h_s "Steam enthalpy";
            Units.SpecificEnthalpy h_w "Liquid enthalpy";
            Units.SpecificEnthalpy h_f "Feedwater enthalpy";
            Units.SpecificEnthalpy h_c "Condensation enthalpy";
            //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
            //Inputs.InputSpecificEnthalpy h_r "Risers enthalpy";
            //Units.SpecificEnthalpy h_b "Bubble enthalpy";
            // Densities
            Units.Density rho_s "Steam density";
            Units.Density rho_w "Liquid density";
            //Units.Density rho_b "Bubble density";
            // States
            WaterSteamMedium.ThermodynamicState state_s "Steam state";
            WaterSteamMedium.ThermodynamicState state_w "Liquid state";
            // Wall temperature
            Units.Temperature T_wall;
            // Water surface tension
            //Modelica.Units.SI.SurfaceTension sigma_s;
            // Water mass fraction
            //Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
            // Interface Area
            //Units.Area A_ma;
            // Water level;
            //Inputs.InputHeight l_water;
            // Escape velocity
            //Units.Velocity u_s;

            // Masses
            //Units.Mass M_w;
            //Units.Mass M_s;

            // Evaporation heat
            Inputs.InputPower W_evap(start=104e6);

            // Set saturation state
            Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

            // Derrivatives
            Real ddph_w;
            Real ddph_s;

            // Riser mass and volume fractions
            Real x_r(start=0.051); // Steam mass fraction of the riser
            Real x_vr_mean; // Steam mean volume fraction in the riser

          WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
          WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
          WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
          WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
          WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,-4})));
          WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,38})));
        equation
          // Variables definition
            // Enthalpies:
            h_s = Steam_source.h_out;
            h_f = FW_supply.h_in;
            // Pressures
            p = FW_sink.P_in;
            p = Steam_extraction.P_in;
            // Mass flow rates
            Q_f = FW_supply.Q;
            //Q_r = Risers_supply.Q;
            Q_s = Steam_extraction.Q;
            //Q_dc = Downcomers_extraction.Q;
            // Set saturation state
            sat.psat = p;
            sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(p);

            // Assume that metal temperature is equal to the saturation pressure
            T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(p);

            // States
            state_w = WaterSteamMedium.setBubbleState(sat);
            state_s = WaterSteamMedium.setDewState(sat);
            // Surface tension
            //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(p));
            // Densities
            rho_s = WaterSteamMedium.density(state_s);
            rho_w = WaterSteamMedium.density(state_w);
            //rho_b = WaterSteamMedium.density(state_s);

            // Enthalpies
            h_s = WaterSteamMedium.specificEnthalpy(state_s);
            h_w = WaterSteamMedium.specificEnthalpy(state_w);
            h_c = h_s - h_w;

            // Masses
            //M_s = rho_s*V_st;
            //M_w = rho_w*V_wt;


            // Volume
            V_t = V_st + V_wt;
            V_wd = V_wt - V_dc - (1 - x_vr_mean)*V_r;
            //V_w = V_wt - V_sd;

            // Steam mean volume fraction in risers
            x_vr_mean = rho_w/(rho_w - rho_s)*(1 - rho_s/((rho_w - rho_s)*x_r)*log(1 + (rho_w - rho_s)*x_r/rho_s));

            // Global mass balance
            Q_f - Q_s = V_st*(ddph_s*der(p)) + rho_s*der(V_st)
                      + V_wt*(ddph_w*der(p)) + rho_w*der(V_wt);

            // Energy Balance
            W_evap + Q_f*h_f - Q_s*h_s = h_s*V_st*ddph_s*der(p) + rho_s*V_st*der(h_s) + rho_s*h_s*der(V_st)
                                       + h_w*V_wt*ddph_w*der(p) + rho_w*V_wt*der(h_w) + rho_w*h_w*der(V_wt)
                                       - V_t*der(p)
                                       + M_t*Cp*der(T_wall);

            // Mass balance for riser section
            Q_dc - Q_r = x_vr_mean*V_r*ddph_s*der(p) + rho_s*V_r*der(x_vr_mean)
                       + V_r*ddph_w*der(p)
                       - x_vr_mean*V_r*ddph_w*der(p) - rho_w*V_r*der(x_vr_mean);

            // Energy balance for the riser section
            W_evap + Q_dc*h_w - (x_r*h_c + h_w)*Q_r = h_s*x_vr_mean*V_r*ddph_s*der(p) + rho_s*h_s*V_r*der(x_vr_mean) + rho_s*x_vr_mean*V_r*der(h_s)
                                                    + h_w*V_r*ddph_w*der(p) + rho_w*V_r*der(h_w)
                                                    - h_w*x_vr_mean*V_r*ddph_w*der(p) - rho_w*h_w*V_r*der(x_vr_mean) - rho_w*x_vr_mean*V_r*der(h_w)
                                                    - V_r*der(p)
                                                    + M_r*Cp*der(T_wall);

            // Mass balance for the steam under the liquid level
            x_r*Q_r - Q_sd - Q_cd = ddph_s*der(p)*V_sd + rho_s*der(V_sd);
            Q_cd = (h_w - h_f)/h_c*Q_f + 1/h_c*(rho_s*V_sd*der(h_s) + rho_w*V_wd*der(h_w) - (V_sd + V_wd)*der(p) + M_d*Cp*der(T_wall));
            Q_ct = (h_w - h_f)/h_c*Q_f + 1/h_c*(rho_s*V_st*der(h_s) + rho_w*V_wt*der(h_w) - V_t*der(p) + M_t*Cp*der(T_wall));

            // Mass flow rate on steam through the surface
            Q_sd = rho_s/T_d*(V_sd - V_0_sd) + x_r*Q_dc + x_r*beta*(Q_dc - Q_r);

            // Mass flow rate in the downcomers
            0.5*k_friction*Q_dc^2 = rho_w*A_dc*(rho_w - rho_s)*g*x_vr_mean*V_r;

            // Derrivatives
            ddph_w = WaterSteamMedium.density_derp_h(state_w);
            ddph_s = WaterSteamMedium.density_derp_h(state_s);

          //der(p) = 0;
          //der(V_wt) = 0;
          //der(x_r) = 0;
          //der(V_sd) = 0;

        initial equation
          //der(p) = 0;
          //der(V_wt) = 0;
          der(x_r) = 0;
          der(V_sd) = 0;



        equation
          connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
          connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
          connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
          connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Polygon(
                  points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                  lineThickness=1,
                  smooth=Smooth.Bezier,
                  fillColor={85,170,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={85,170,255}),
                Ellipse(
                  extent={{-100,100},{100,-100}},
                  lineColor={135,135,135},
                  lineThickness=1),
                Line(
                  points={{-32,-20}},
                  color={135,135,135},
                  thickness=1),
                Ellipse(
                  extent={{-48,-60},{-40,-68}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-26,-52},{-18,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-38},{-2,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-62},{-2,-70}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-46,-38},{-38,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{6,-52},{14,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{28,-58},{36,-66}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{10,-72},{18,-80}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-32,-76},{-24,-84}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-78,-32},{-70,-40}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-42,-26},{-34,-34}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-18,-16},{-10,-24}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-30,-14},{-22,-22}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{12,-24},{20,-32}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
        end SteamDrum_Astom_SimplifiedGeometry_3;

        model SteamDrum_Astom_SimplifiedGeometry_4
           import MetroscopeModelingLibrary.Utilities.Units;
          import MetroscopeModelingLibrary.Utilities.Units.Inputs;
          import MetroscopeModelingLibrary.Utilities.Constants;
          import CorrelationConstants =
                 MetroscopeModelingLibrary.DynamicComponents.Correlations;

          package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
          package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

          // Constants
            parameter Real pi = Constants.pi;
            parameter Real g = Constants.g;

          // Dimensions and properties
            parameter Modelica.Units.SI.Volume V_D = 40 "Drum volume";
            parameter Units.Mass M_d = M_t - M_r "Drum mass";
            parameter Units.Mass M_t = 300000 "Total mass of the system";
            parameter Modelica.Units.SI.Volume V_r = 37 "Riser volume";
            parameter Units.Mass M_r = 160000 "Riser mass";
            parameter Units.HeatCapacity Cp = 472 "Heat capacity of the metal";
            parameter Units.Area A_dc = 0.371 "Downcomers cross sectional surface";
            parameter Units.Area A_d = 20 "Drum wet area at normal operating level";
            parameter Modelica.Units.SI.Volume V_dc = 11 "Downcomers volume";
            parameter Real k_friction = 25 "Friction coefficient in the downcomers";
            parameter Real beta = 0.3 "Constant";
            parameter Modelica.Units.SI.Volume V_t = V_D + V_r + V_dc;
            parameter Modelica.Units.SI.Volume V_0_sd = 7.662651 "Hypothetical volume of bubbles";
            parameter Real T_d = 12 "Steam residence time in the drum";

          // Initialization parameters
            parameter Units.Pressure p_0 = 85e5;
            parameter Modelica.Units.SI.Volume V_wt_0 = 57.1;

          // Water/Steam properties
            // Drum pressure
            Units.Pressure p(start=p_0) "Drum saturation pressure";
            // Mass flow rates
            Units.PositiveMassFlowRate Q_f "Feed water mass flow rate";
            Units.PositiveMassFlowRate Q_sd "Steam mass flow rate across the surface";
            Units.PositiveMassFlowRate Q_s "Steam mass flow rate";
            Units.PositiveMassFlowRate Q_dc "Downcomers mass flow rate";
            Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
            Units.PositiveMassFlowRate Q_cd "Condensation mass flow rate";
            // Volumes
            Modelica.Units.SI.Volume V_st "Steam volume";
            Modelica.Units.SI.Volume V_sd(start=4.8) "Bubble volume";
            Modelica.Units.SI.Volume V_wt(start=V_wt_0, fixed=true) "Total water volume under the level";
            Modelica.Units.SI.Volume V_wd "Water volume in the drum";
            // Enthalpies
            Units.SpecificEnthalpy h_s "Steam enthalpy";
            Units.SpecificEnthalpy h_w "Liquid enthalpy";
            Units.SpecificEnthalpy h_f "Feedwater enthalpy";
            Units.SpecificEnthalpy h_c "Condensation enthalpy";
            // Densities
            Units.Density rho_s "Steam density";
            Units.Density rho_w "Liquid density";
            // States
            WaterSteamMedium.ThermodynamicState state_s "Steam state";
            WaterSteamMedium.ThermodynamicState state_w "Liquid state";
            // Wall temperature
            Units.Temperature T_wall "Metal wall temperature";
            // Set saturation state
            Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat "Saturation state for properties calculation";
            // Riser mass and volume fractions
            Real x_r(start=0.051) "Steam mass fraction of the riser";
            Real x_vr_mean "Steam mean volume fraction in the riser";
            // Density derrivatives
            Real ddph_w "Density derivative with respect to the pressure at constant enthalpy for water";
            Real ddph_s "Density derivative with respect to the pressure at constant enthalpy for steam";
            Real ddhp_w "Density derivative with respect to the enthalpy at constant pressure for water";
            Real ddhp_s "Density derivative with respect to the enthalpy at constant pressure for steam";

          // Evaporation heat
          Inputs.InputPower W_evap(start=104e6) "Heat rate to the risers";
          // Drum water level
          Units.Height l "Water level";

          WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
          WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
          WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
          WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
          WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,-4})));
          WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,38})));
        equation
          // Connectors
            // Enthalpies:
            h_s = Steam_source.h_out;
            h_f = FW_supply.h_in;
            // Pressures
            p = FW_sink.P_in;
            p = Steam_extraction.P_in;
            // Mass flow rates
            Q_f = FW_supply.Q;
            Q_s = Steam_extraction.Q;

          // Water/Steam properties
          // Set saturation state
          sat.psat = p;
          sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(p);
          // Assume that metal temperature is equal to the saturation pressure
          T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(p);

          // States
          state_w = WaterSteamMedium.setBubbleState(sat);
          state_s = WaterSteamMedium.setDewState(sat);
          // Surface tension
          //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(p));
          // Densities
          rho_s = WaterSteamMedium.density(state_s);
          rho_w = WaterSteamMedium.density(state_w);
          //rho_b = WaterSteamMedium.density(state_s);

          // Enthalpies
          h_s = WaterSteamMedium.specificEnthalpy(state_s);
          h_w = WaterSteamMedium.specificEnthalpy(state_w);
          h_c = h_s - h_w;

            // Masses
            //M_s = rho_s*V_st;
            //M_w = rho_w*V_wt;

            // Volume
            V_t = V_st + V_wt;
            V_wd = V_wt - V_dc - (1 - x_vr_mean)*V_r;

            // Steam mean volume fraction in risers
            x_vr_mean = rho_w/(rho_w - rho_s)*(1 - rho_s/((rho_w - rho_s)*x_r)*log(1 + (rho_w - rho_s)*x_r/rho_s));

            // Global mass balance
            Q_f - Q_s = V_st*((ddph_s*der(p) + ddhp_s*der(h_s))) + rho_s*der(V_st)
                      + V_wt*((ddph_w*der(p) + ddhp_w*der(h_w))) + rho_w*der(V_wt);

            // Energy Balance
            W_evap + Q_f*h_f - Q_s*h_s = h_s*V_st*(ddph_s*der(p) + ddhp_s*der(h_s)) + rho_s*V_st*der(h_s) + rho_s*h_s*der(V_st)
                                       + h_w*V_wt*(ddph_w*der(p) + ddhp_w*der(h_w)) + rho_w*V_wt*der(h_w) + rho_w*h_w*der(V_wt)
                                       - V_t*der(p)
                                       + M_t*Cp*der(T_wall);

            // Mass balance for riser section
            Q_dc - Q_r = x_vr_mean*V_r*(ddph_s*der(p) + ddhp_s*der(h_s)) + rho_s*V_r*der(x_vr_mean)
                       + V_r*(ddph_w*der(p) + ddhp_w*der(h_w))
                       - x_vr_mean*V_r*(ddph_w*der(p) + ddhp_w*der(h_w)) - rho_w*V_r*der(x_vr_mean);

            // Energy balance for the riser section
            W_evap + Q_dc*h_w - (x_r*h_c + h_w)*Q_r = h_s*x_vr_mean*V_r*(ddph_s*der(p) + ddhp_s*der(h_s)) + rho_s*h_s*V_r*der(x_vr_mean) + rho_s*x_vr_mean*V_r*der(h_s)
                                                    + h_w*V_r*(ddph_w*der(p) + ddhp_w*der(h_w)) + rho_w*V_r*der(h_w)
                                                    - h_w*x_vr_mean*V_r*(ddph_w*der(p) + ddhp_w*der(h_w)) - rho_w*h_w*V_r*der(x_vr_mean) - rho_w*x_vr_mean*V_r*der(h_w)
                                                    - V_r*der(p)
                                                    + M_r*Cp*der(T_wall);

            // Mass balance for the steam under the liquid level
            x_r*Q_r - Q_sd - Q_cd = (ddph_s*der(p) + ddhp_s*der(h_s))*V_sd + rho_s*der(V_sd);
            Q_cd = (h_w - h_f)/h_c*Q_f + 1/h_c*(rho_s*V_sd*der(h_s) + rho_w*V_wd*der(h_w) - (V_sd + V_wd)*der(p) + M_d*Cp*der(T_wall));

            // Mass flow rate on steam through the surface
            Q_sd = rho_s/T_d*(V_sd - V_0_sd) + x_r*Q_dc + x_r*beta*(Q_dc - Q_r);

            // Mass flow rate in the downcomers
            0.5*k_friction*Q_dc^2 = rho_w*A_dc*(rho_w - rho_s)*g*x_vr_mean*V_r;

            // Level equation
            l = (V_wd + V_sd)/A_d;

            // Derrivatives
            ddph_w = WaterSteamMedium.density_derp_h(state_w);
            ddph_s = WaterSteamMedium.density_derp_h(state_s);
            ddhp_w = WaterSteamMedium.density_derh_p(state_w);
            ddhp_s = WaterSteamMedium.density_derh_p(state_s);


        initial equation
          der(x_r) = 0;
          der(V_sd) = 0;

        equation
          connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
          connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
          connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
          connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Polygon(
                  points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                  lineThickness=1,
                  smooth=Smooth.Bezier,
                  fillColor={85,170,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={85,170,255}),
                Ellipse(
                  extent={{-100,100},{100,-100}},
                  lineColor={135,135,135},
                  lineThickness=1),
                Line(
                  points={{-32,-20}},
                  color={135,135,135},
                  thickness=1),
                Ellipse(
                  extent={{-48,-60},{-40,-68}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-26,-52},{-18,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-38},{-2,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-62},{-2,-70}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-46,-38},{-38,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{6,-52},{14,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{28,-58},{36,-66}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{10,-72},{18,-80}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-32,-76},{-24,-84}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-78,-32},{-70,-40}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-42,-26},{-34,-34}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-18,-16},{-10,-24}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-30,-14},{-22,-22}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{12,-24},{20,-32}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
        end SteamDrum_Astom_SimplifiedGeometry_4;

        model SteamDrum_Sunil_1
           import MetroscopeModelingLibrary.Utilities.Units;
          import MetroscopeModelingLibrary.Utilities.Units.Inputs;
          import MetroscopeModelingLibrary.Utilities.Constants;
          import CorrelationConstants =
                 MetroscopeModelingLibrary.DynamicComponents.Correlations;

          package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
          package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

          // Constants
            parameter Real pi = Constants.pi;
            parameter Real g = Constants.g;

          // Control strategy
            parameter Boolean Controlled_level = false;

          // Drum dimensions
            parameter Modelica.Units.SI.Volume V_D = 55.8;
            parameter Units.Mass M_D = 111030;
            parameter Units.HeatCapacity Cp_D = 232;
            parameter Units.Length D_D = 2.6;
            parameter Units.Length R_D = D_D/2;
            parameter Units.Length L_D = 11.160;
            parameter Real K_tuning = 1;

          // Initialization parameters
          parameter Units.Pressure p_0 = 121.631e5;
          parameter Modelica.Units.SI.Volume V_l_0 = 20;

          // Water/Steam properties
            // Drum pressure
            Units.Pressure p(start=p_0) "Drum saturation pressure";
            // Mass flow rates
            Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
            Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
            Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
            Units.PositiveMassFlowRate Q_d(start=80) "Downcomers mass flow rate";
            Units.PositiveMassFlowRate Q_r(start=80) "Risers mass flow rate";
            // Volumes
            Modelica.Units.SI.Volume V_s "Steam volume";
            Modelica.Units.SI.Volume V_b "Bubble volume";
            Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
            // Enthalpies
            Units.SpecificEnthalpy h_s "Steam enthalpy";
            Units.SpecificEnthalpy h_l "Liquid enthalpy";
            Units.SpecificEnthalpy h_fw(start=1431487.1) "Feedwater enthalpy";
            Units.SpecificEnthalpy h_d(start=1509058.1) "Downcomers enthalpy";
            Units.SpecificEnthalpy h_r(start=2675842.2) "Risers enthalpy";
            //Units.SpecificEnthalpy h_b "Bubble enthalpy";
            // Densities
            Units.Density rho_s "Steam density";
            Units.Density rho_l "Liquid density";
            Units.Density rho_b "Bubble density";
            // States
            WaterSteamMedium.ThermodynamicState state_s "Steam state";
            WaterSteamMedium.ThermodynamicState state_l "Liquid state";
            // Wall temperature
            Units.Temperature T_wall;
            // Water surface tension
            Modelica.Units.SI.SurfaceTension sigma_s;
            //Interface Area
            Units.Area A_ma;
            // Water level;
            Units.Height l_water(start=1.2, fixed=true);
            // Escape velocity
            Units.Velocity u_s;
            // Masses
            Units.Mass M_l;
            Units.Mass M_s;
            // Set saturation state
            Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

            // Derrivatives
            Real ddph_l;
            Real ddph_s;
            Real ddph_b;

          WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
          WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
          WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
          WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
          WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,-4})));
          WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,38})));
          WaterSteam.BaseClasses.IsoPHFlowModel Downcomers_extraction annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={0,-60})));
          WaterSteam.Connectors.Outlet downcomers_out annotation (Placement(transformation(extent={{-10,-110},{10,-90}}), iconTransformation(extent={{-10,-110},{10,-90}})));
          WaterSteam.BoundaryConditions.Sink Risers_sink annotation (Placement(transformation(extent={{-26,-70},{-6,-50}})));
          WaterSteam.BaseClasses.IsoPHFlowModel Risers_supply annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
          WaterSteam.Connectors.Inlet risers_in annotation (Placement(transformation(extent={{-90,-70},{-70,-50}}), iconTransformation(extent={{-90,-70},{-70,-50}})));
          WaterSteam.BoundaryConditions.Source Downcomers_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={0,-28})));
        equation
          // Variables definition
            // Enthalpies:
            h_s = Steam_extraction.h_out;
            h_fw = FW_supply.h_in;
            h_r = Risers_supply.h_in;
            h_d = Downcomers_extraction.h_out;
            // Pressures
            p = FW_supply.P_in;
            p = Risers_supply.P_in;
            p = Steam_extraction.P_in;
            p = Downcomers_extraction.P_in;
            // Mass flow rates
            Q_fw = FW_supply.Q;
            Q_r = Risers_supply.Q;
            Q_s = Steam_extraction.Q;
            Q_d = Downcomers_extraction.Q;
            // Set saturation state
            sat.psat = p;
            sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(p);

            // Wall temperature is assumed equal to the saturation temperature
            T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(p);

            // States
            state_l = WaterSteamMedium.setBubbleState(sat);
            state_s = WaterSteamMedium.setDewState(sat);

            // Densities
            rho_s = WaterSteamMedium.density(state_s);
            rho_l = WaterSteamMedium.density(state_l);
            rho_b = WaterSteamMedium.density(state_s); // Steam bubble density = sat. density

            // Enthalpies
            h_s = WaterSteamMedium.specificEnthalpy(state_s);
            h_l = WaterSteamMedium.specificEnthalpy(state_l);

            // Masses
            M_s = rho_s*V_s + rho_b*V_b;
            M_l = rho_l*V_l;

            // Volume
            V_s = V_D - V_l - V_b; // Verified eq 2
            V_l + V_b = (R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5)*L_D;

            // Mass balance
            // Total mass balance in drum
            Q_fw + Q_r - Q_s - Q_d = der(p)*(V_l*ddph_l + V_D*ddph_s - V_l*ddph_s)
                                   + der(V_l)*(rho_l - rho_s)
                                   + der(V_b)*rho_s; // Verified eq 3

            // Mass balance on control volume with liquid and bubble region:
            Q_fw + Q_r - Q_d - Q_b = der(p)*(V_l*ddph_l + V_b*ddph_b)
                                   + der(V_l)*rho_s
                                   + der(V_b)*rho_b; // Verified eq 5

            Q_fw*h_fw + Q_r*h_r - Q_s*h_s - Q_d*h_d = h_l*V_l*ddph_l*der(p) + rho_l*V_l*der(h_l)
                                                    + V_D*h_s*ddph_s*der(p) + rho_s*V_D*der(h_s)
                                                    - V_l*h_s*ddph_s*der(p)
                                                    + der(V_l)*(rho_l*h_l - rho_s*h_s)
                                                    + M_D*Cp_D*der(T_wall); // Verified eq 11

            //Q_r*h_r - Q_b*h_d = 104e6;

            // Bubble mass flow rate
            // Surface tension
            sigma_s = WaterSteamMedium.surfaceTension(sat);
            // Interface area
            A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
            // Escape velocity
            u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4); // Verified eq 7
            // Bubble mass flow rate
            Q_b = K_tuning*A_ma*u_s*rho_s; // Verified eq 6

            // Derrivatives
            ddph_l = WaterSteamMedium.density_derp_h(state_l);
            ddph_s = WaterSteamMedium.density_derp_h(state_s);
            ddph_b = WaterSteamMedium.density_derp_h(state_s);

            if Controlled_level then
              der(V_l) = 0;
            end if;

        initial equation
          der(p) = 0;
          //der(V_b) = 0;
          //der(V_l) = 0;

        equation
          connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
          connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
          connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
          connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
          connect(Downcomers_extraction.C_out, downcomers_out) annotation (Line(points={{0,-70},{0,-70},{0,-100}}, color={28,108,200}));
          connect(Downcomers_extraction.C_in, Downcomers_source.C_out) annotation (Line(points={{0,-50},{0,-33}}, color={28,108,200}));
          connect(Risers_sink.C_in, Risers_supply.C_out) annotation (Line(points={{-21,-60},{-40,-60}}, color={28,108,200}));
          connect(Risers_supply.C_in, risers_in) annotation (Line(points={{-60,-60},{-80,-60}}, color={28,108,200}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Polygon(
                  points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                  lineThickness=1,
                  smooth=Smooth.Bezier,
                  fillColor={85,170,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={85,170,255}),
                Ellipse(
                  extent={{-100,100},{100,-100}},
                  lineColor={135,135,135},
                  lineThickness=1),
                Line(
                  points={{-32,-20}},
                  color={135,135,135},
                  thickness=1),
                Ellipse(
                  extent={{-48,-60},{-40,-68}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-26,-52},{-18,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-38},{-2,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-62},{-2,-70}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-46,-38},{-38,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{6,-52},{14,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{28,-58},{36,-66}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{10,-72},{18,-80}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-32,-76},{-24,-84}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-78,-32},{-70,-40}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-42,-26},{-34,-34}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-18,-16},{-10,-24}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-30,-14},{-22,-22}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{12,-24},{20,-32}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
        end SteamDrum_Sunil_1;

        model SteamDrum_Sunil_2
           import MetroscopeModelingLibrary.Utilities.Units;
          import MetroscopeModelingLibrary.Utilities.Units.Inputs;
          import MetroscopeModelingLibrary.Utilities.Constants;
          import CorrelationConstants =
                 MetroscopeModelingLibrary.DynamicComponents.Correlations;

          package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
          package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

          // Constants
            parameter Real pi = Constants.pi;
            parameter Real g = Constants.g;

          // Drum dimensions
            parameter Modelica.Units.SI.Volume V_D = 55.8;
            parameter Units.Mass M_D = 111030;
            parameter Units.HeatCapacity Cp_D = 232;
            parameter Units.Length D_D = 2.6;
            parameter Units.Length R_D = D_D/2;
            parameter Units.Length L_D = 11.160;
            parameter Real K_tuning = 1;

          // Initialization parameters
          parameter Units.Pressure p_0 = 121.631e5;
          parameter Modelica.Units.SI.Volume V_l_0 = 20;

          // Water/Steam properties
            // Drum pressure
            Units.Pressure p(start=p_0) "Drum saturation pressure";
            // Mass flow rates
            Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
            Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
            Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
            Units.PositiveMassFlowRate Q_d(start=80) "Downcomers mass flow rate";
            Units.PositiveMassFlowRate Q_r(start=80) "Risers mass flow rate";
            // Volumes
            Modelica.Units.SI.Volume V_s "Steam volume";
            Modelica.Units.SI.Volume V_b "Bubble volume";
            Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
            // Enthalpies
            Units.SpecificEnthalpy h_s "Steam enthalpy";
            Units.SpecificEnthalpy h_l "Liquid enthalpy";
            Units.SpecificEnthalpy h_fw(start=1431487.1) "Feedwater enthalpy";
            Units.SpecificEnthalpy h_d(start=1509058.1) "Downcomers enthalpy";
            Units.SpecificEnthalpy h_r(start=2675842.2) "Risers enthalpy";
            // Densities
            Units.Density rho_s "Steam density";
            Units.Density rho_l "Liquid density";
            Units.Density rho_b "Bubble density";
            // States
            WaterSteamMedium.ThermodynamicState state_s "Steam state";
            WaterSteamMedium.ThermodynamicState state_l "Liquid state";
            // Wall temperature
            Units.Temperature T_wall;
            // Water surface tension
            Modelica.Units.SI.SurfaceTension sigma_s;
            //Interface Area
            Units.Area A_ma;
            // Water level;
            Units.Height l_water(start=1.2, fixed=true);
            // Escape velocity
            Units.Velocity u_s;
            // Masses
            //Units.Mass M_l;
            //Units.Mass M_s;
            // Set saturation state
            Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

            // Derrivatives
            Real ddph_l;
            Real ddph_s;
            Real ddph_b;

          WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
          WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
          WaterSteam.BaseClasses.IsoPHFlowModel FW_supply annotation (Placement(transformation(extent={{60,-70},{40,-50}})));
          WaterSteam.BoundaryConditions.Sink FW_sink annotation (Placement(transformation(extent={{28,-70},{8,-50}})));
          WaterSteam.BoundaryConditions.Source Steam_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,-4})));
          WaterSteam.BaseClasses.IsoPHFlowModel Steam_extraction annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,38})));
          WaterSteam.BaseClasses.IsoPHFlowModel Downcomers_extraction annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={0,-60})));
          WaterSteam.Connectors.Outlet downcomers_out annotation (Placement(transformation(extent={{-10,-110},{10,-90}}), iconTransformation(extent={{-10,-110},{10,-90}})));
          WaterSteam.BoundaryConditions.Sink Risers_sink annotation (Placement(transformation(extent={{-26,-70},{-6,-50}})));
          WaterSteam.BaseClasses.IsoPHFlowModel Risers_supply annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
          WaterSteam.Connectors.Inlet risers_in annotation (Placement(transformation(extent={{-90,-70},{-70,-50}}), iconTransformation(extent={{-90,-70},{-70,-50}})));
          WaterSteam.BoundaryConditions.Source Downcomers_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=270,
                origin={0,-28})));
        equation
          // Variables definition
            // Enthalpies:
            h_s = Steam_extraction.h_out;
            h_fw = FW_supply.h_in;
            h_r = Risers_supply.h_in;
            h_d = Downcomers_extraction.h_out;
            // Pressures
            p = FW_supply.P_in;
            p = Risers_supply.P_in;
            p = Steam_extraction.P_in;
            p = Downcomers_extraction.P_in;
            // Mass flow rates
            Q_fw = FW_supply.Q;
            Q_r = Risers_supply.Q;
            Q_s = Steam_extraction.Q;
            Q_d = Downcomers_extraction.Q;
            // Set saturation state
            sat.psat = p;
            sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(p);

            // Wall temperature is assumed equal to the saturation temperature
            T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(p);

            // States
            state_l = WaterSteamMedium.setBubbleState(sat);
            state_s = WaterSteamMedium.setDewState(sat);

            // Densities
            rho_s = WaterSteamMedium.density(state_s);
            rho_l = WaterSteamMedium.density(state_l);
            rho_b = WaterSteamMedium.density(state_s); // Steam bubble density = sat. density

            // Enthalpies
            h_s = WaterSteamMedium.specificEnthalpy(state_s);
            h_l = WaterSteamMedium.specificEnthalpy(state_l);

            // Masses
            //M_s = rho_s*V_s + rho_b*V_b;
            //M_l = rho_l*V_l;

            // Volume
            V_s = V_D - V_l - V_b; // Verified eq 2
            V_l + V_b = (R_D^2*acos(1 - l_water/R_D) - (R_D - l_water)*(R_D^2 - (R_D - l_water)^2)^0.5)*L_D;

            // Mass balance
            // Total mass balance in drum
            Q_fw + Q_r - Q_s - Q_d = der(p)*(V_l*ddph_l + V_D*ddph_s - V_l*ddph_s)
                                   + der(V_l)*(rho_l - rho_s)
                                   + der(V_b)*rho_s; // Verified eq 3

            // Mass balance on control volume with liquid and bubble region:
            Q_fw + Q_r - Q_d - Q_b = der(p)*(V_l*ddph_l + V_b*ddph_b)
                                   + der(V_l)*rho_s
                                   + der(V_b)*rho_b; // Verified eq 5

            // Energy balance in drum
            Q_fw*h_fw + Q_r*h_r - Q_s*h_s - Q_d*h_d = h_l*V_l*ddph_l*der(p) + rho_l*V_l*der(h_l)
                                                    + V_D*h_s*ddph_s*der(p) + rho_s*V_D*der(h_s)
                                                    - V_l*h_s*ddph_s*der(p)
                                                    + der(V_l)*(rho_l*h_l - rho_s*h_s)
                                                    + M_D*Cp_D*der(T_wall); // Verified eq 11

            Q_d = 18.29*Q_fw;
            h_d = h_l;
            //Q_r*h_r - Q_b*h_d = 104e6;

            // Bubble mass flow rate
            // Surface tension
            sigma_s = WaterSteamMedium.surfaceTension(sat);
            // Interface area
            A_ma = 2*(D_D*l_water - l_water^2)^0.5*L_D;
            // Escape velocity
            u_s = 1.41*((g*sigma_s*(rho_l - rho_s))/rho_l^2)^(1/4); // Verified eq 7
            // Bubble mass flow rate
            Q_b = K_tuning*A_ma*u_s*rho_s; // Verified eq 6

            // Derrivatives
            ddph_l = WaterSteamMedium.density_derp_h(state_l);
            ddph_s = WaterSteamMedium.density_derp_h(state_s);
            ddph_b = WaterSteamMedium.density_derp_h(state_s);


        initial equation
          //der(p) = 0;
          //der(V_b) = 0;
          //der(V_l) = 0;

        equation
          connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
          connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
          connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
          connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
          connect(Downcomers_extraction.C_out, downcomers_out) annotation (Line(points={{0,-70},{0,-70},{0,-100}}, color={28,108,200}));
          connect(Downcomers_extraction.C_in, Downcomers_source.C_out) annotation (Line(points={{0,-50},{0,-33}}, color={28,108,200}));
          connect(Risers_sink.C_in, Risers_supply.C_out) annotation (Line(points={{-21,-60},{-40,-60}}, color={28,108,200}));
          connect(Risers_supply.C_in, risers_in) annotation (Line(points={{-60,-60},{-80,-60}}, color={28,108,200}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Polygon(
                  points={{-98,-20},{98,-20},{98,-20},{92,-40},{80,-60},{60,-80},{40,-92},{20,-98},{0,-100},{-20,-98},{-40,-92},{-60,-80},{-78,-62},{-92,-40},{-98,-20},{-98,-20}},
                  lineThickness=1,
                  smooth=Smooth.Bezier,
                  fillColor={85,170,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={85,170,255}),
                Ellipse(
                  extent={{-100,100},{100,-100}},
                  lineColor={135,135,135},
                  lineThickness=1),
                Line(
                  points={{-32,-20}},
                  color={135,135,135},
                  thickness=1),
                Ellipse(
                  extent={{-48,-60},{-40,-68}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-26,-52},{-18,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-38},{-2,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-10,-62},{-2,-70}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-46,-38},{-38,-46}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{6,-52},{14,-60}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{28,-58},{36,-66}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{10,-72},{18,-80}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-32,-76},{-24,-84}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-78,-32},{-70,-40}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid,
                  lineColor={0,0,0}),
                Ellipse(
                  extent={{-42,-26},{-34,-34}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-18,-16},{-10,-24}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-30,-14},{-22,-22}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{12,-24},{20,-32}},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
        end SteamDrum_Sunil_2;
      end SteamDrumModels;
    end TwoPhaseHX;
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Polygon(
            points={{-66,-70},{-48,-70},{-48,-64},{-50,-52},{-54,-42},{-58,-32},{-60,-26},{-60,-14},{-60,-6},{-54,10},{-52,14},{-50,22},{-48,32},{-48,38},{-32,38},{-56,70},{-56,70},{-80,38},{-64,38},{-64,32},{-66,26},{-68,18},{-72,10},{-74,4},{-76,-4},{-76,-14},{-76,-26},{-74,-34},{-70,-46},{-66,-56},{-66,-70}},
            lineColor={238,46,47},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-10,-70},{8,-70},{8,-64},{6,-52},{2,-42},{-2,-32},{-4,-26},{-4,-14},{-4,-6},{2,10},{4,14},{6,22},{8,32},{8,38},{24,38},{0,70},{0,70},{-24,38},{-8,38},{-8,32},{-10,26},{-12,18},{-16,10},{-18,4},{-20,-4},{-20,-14},{-20,-26},{-18,-34},{-14,-46},{-10,-56},{-10,-70}},
            lineColor={238,46,47},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{48,-70},{66,-70},{66,-64},{64,-52},{60,-42},{56,-32},{54,-26},{54,-14},{54,-6},{60,10},{62,14},{64,22},{66,32},{66,38},{82,38},{58,70},{58,70},{34,38},{50,38},{50,32},{48,26},{46,18},{42,10},{40,4},{38,-4},{38,-14},{38,-26},{40,-34},{44,-46},{48,-56},{48,-70}},
            lineColor={238,46,47},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid)}));
  end HeatExchangers;

  package Tests

    package One_pass_HX
      model CounterCurrent_MonoPhasicHX_LCM_Test
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        HeatExchangers.One_pass_HX.CounterCurrent_MonoPhasicHX_LCM HX(
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
      end CounterCurrent_MonoPhasicHX_LCM_Test;

      model CounterCurrent_MonoPhasicHX_2NodesConduction_Test
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        HeatExchangers.One_pass_HX.CounterCurrent_MonoPhasicHX_2NodesConduction HX(
          N_tubes_row=184,
          Rows=2,
          K_cond_wall=27,
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
      end CounterCurrent_MonoPhasicHX_2NodesConduction_Test;

      model CounterCurrent_MonoPhasicHX_1DConduction_plate_Test
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        HeatExchangers.One_pass_HX.CounterCurrent_MonoPhasicHX_1DConduction_plate HX(
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
      end CounterCurrent_MonoPhasicHX_1DConduction_plate_Test;

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

      model CrossCurrent_MonoPhasicHX_LCM_ConstantK_Test
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
        HeatExchangers.One_pass_HX.CrossCurrent_MonoPhasicHX_LCM_ConstantK crossCurrent_MonoPhasicHX_LCM_ConstantK annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(hot_source.C_out, crossCurrent_MonoPhasicHX_LCM_ConstantK.fg_inlet) annotation (Line(points={{-79,0},{-4,0}}, color={95,95,95}));
        connect(crossCurrent_MonoPhasicHX_LCM_ConstantK.fg_outlet, hot_sink.C_in) annotation (Line(points={{4,0},{79,0}}, color={95,95,95}));
        connect(crossCurrent_MonoPhasicHX_LCM_ConstantK.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-45.5},{2.77556e-16,-45.5},{2.77556e-16,-81}}, color={28,108,200}));
        connect(crossCurrent_MonoPhasicHX_LCM_ConstantK.water_outlet, cold_sink.C_in) annotation (Line(points={{0,10},{0,44.5},{-9.4369e-16,44.5},{-9.4369e-16,79}}, color={28,108,200}));
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
      end CrossCurrent_MonoPhasicHX_LCM_ConstantK_Test;

      model CrossCurrent_MonoPhasicHX_LCM_Test
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
        HeatExchangers.One_pass_HX.CrossCurrent_MonoPhasicHX_LCM crossCurrent_MonoPhasicHX_2NodesConduction(
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          N_tubes_row=184,
          Rows=2,
          T_wall_0=773.15) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(hot_source.C_out, crossCurrent_MonoPhasicHX_2NodesConduction.fg_inlet) annotation (Line(points={{-79,0},{-4,0}}, color={95,95,95}));
        connect(crossCurrent_MonoPhasicHX_2NodesConduction.fg_outlet, hot_sink.C_in) annotation (Line(points={{4,0},{79,0}}, color={95,95,95}));
        connect(crossCurrent_MonoPhasicHX_2NodesConduction.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-45.5},{2.77556e-16,-45.5},{2.77556e-16,-81}}, color={28,108,200}));
        connect(crossCurrent_MonoPhasicHX_2NodesConduction.water_outlet, cold_sink.C_in) annotation (Line(points={{0,10},{0,44.5},{-9.4369e-16,44.5},{-9.4369e-16,79}}, color={28,108,200}));
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
      end CrossCurrent_MonoPhasicHX_LCM_Test;

      model CrossCurrent_MonoPhasicHX_2NodesConduction_Test
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
        HeatExchangers.One_pass_HX.CrossCurrent_MonoPhasicHX_2NodesConduction crossCurrent_MonoPhasicHX_LCM(
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          N_tubes_row=184,
          Rows=2,
          T_wall_0=773.15) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(hot_source.C_out, crossCurrent_MonoPhasicHX_LCM.fg_inlet) annotation (Line(points={{-79,0},{-4,0}}, color={95,95,95}));
        connect(crossCurrent_MonoPhasicHX_LCM.fg_outlet, hot_sink.C_in) annotation (Line(points={{4,0},{79,0}}, color={95,95,95}));
        connect(crossCurrent_MonoPhasicHX_LCM.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-45.5},{2.77556e-16,-45.5},{2.77556e-16,-81}}, color={28,108,200}));
        connect(crossCurrent_MonoPhasicHX_LCM.water_outlet, cold_sink.C_in) annotation (Line(points={{0,10},{0,44.5},{-9.4369e-16,44.5},{-9.4369e-16,79}}, color={28,108,200}));
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
      end CrossCurrent_MonoPhasicHX_2NodesConduction_Test;

      model CrossCurrent_MonoPhasicHX_1DConduction_Test
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
        HeatExchangers.One_pass_HX.CrossCurrent_MonoPhasicHX_1DConduction_radial crossCurrent_MonoPhasicHX_LCM(
          D_out=0.0381,
          e=0.003048,
          L=18.29,
          N_tubes_row=184,
          Rows=2,
          T_wall_0=773.15) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(hot_source.C_out, crossCurrent_MonoPhasicHX_LCM.fg_inlet) annotation (Line(points={{-79,0},{-4,0}}, color={95,95,95}));
        connect(crossCurrent_MonoPhasicHX_LCM.fg_outlet, hot_sink.C_in) annotation (Line(points={{4,0},{79,0}}, color={95,95,95}));
        connect(crossCurrent_MonoPhasicHX_LCM.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-45.5},{2.77556e-16,-45.5},{2.77556e-16,-81}}, color={28,108,200}));
        connect(crossCurrent_MonoPhasicHX_LCM.water_outlet, cold_sink.C_in) annotation (Line(points={{0,10},{0,44.5},{-9.4369e-16,44.5},{-9.4369e-16,79}}, color={28,108,200}));
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
      end CrossCurrent_MonoPhasicHX_1DConduction_Test;

      model CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_Test
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        HeatExchangers.One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK HX(
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
              origin={0,42})));
        FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
        FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{72,-10},{92,10}})));
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

        connect(HX.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-77}},                        color={28,108,200}));
        connect(HX.water_outlet, cold_sink.C_in) annotation (Line(points={{0,12},{0,26.5},{-9.4369e-16,26.5},{-9.4369e-16,37}},
                                                                                                                         color={28,108,200}));
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
      end CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_Test;

      model CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage_Test
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        HeatExchangers.One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage HX(
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
              origin={0,42})));
        FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
        FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{72,-10},{92,10}})));
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

        connect(HX.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-77}},                        color={28,108,200}));
        connect(HX.water_outlet, cold_sink.C_in) annotation (Line(points={{0,12},{0,26.5},{-9.4369e-16,26.5},{-9.4369e-16,37}},
                                                                                                                         color={28,108,200}));
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
      end CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage_Test;

      model CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_Test
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        HeatExchangers.One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM           HX(
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
              origin={0,42})));
        FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
        FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{72,-10},{92,10}})));
        Modelica.Blocks.Sources.Step step(height=100, startTime=100) annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=-20,
          duration=60,
          startTime=300) annotation (Placement(transformation(extent={{60,60},{80,80}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(HX.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-77}},                        color={28,108,200}));
        connect(HX.water_outlet, cold_sink.C_in) annotation (Line(points={{0,12},{0,26.5},{-9.4369e-16,26.5},{-9.4369e-16,37}},
                                                                                                                         color={28,108,200}));
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
      end CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_Test;

      model CrossCurrent_1NodePerRow_MonoPhasicHX_2NodesConduction_Test
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        HeatExchangers.One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_2NodesConduction
                                                                                       HX(
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
              origin={0,42})));
        FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
        FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{72,-10},{92,10}})));
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

        connect(HX.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-77}},                        color={28,108,200}));
        connect(HX.water_outlet, cold_sink.C_in) annotation (Line(points={{0,12},{0,26.5},{-9.4369e-16,26.5},{-9.4369e-16,37}},
                                                                                                                         color={28,108,200}));
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
      end CrossCurrent_1NodePerRow_MonoPhasicHX_2NodesConduction_Test;

      model CrossCurrent_1NodePerRow_MonoPhasicHX_1DConduction_radial_Test
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;
        HeatExchangers.One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_1DConduction_radial
                                                                                       HX(
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
              origin={0,42})));
        FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
        FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{72,-10},{92,10}})));
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

        connect(HX.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-77}},                        color={28,108,200}));
        connect(HX.water_outlet, cold_sink.C_in) annotation (Line(points={{0,12},{0,26.5},{-9.4369e-16,26.5},{-9.4369e-16,37}},
                                                                                                                         color={28,108,200}));
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
      end CrossCurrent_1NodePerRow_MonoPhasicHX_1DConduction_radial_Test;
      annotation (Icon(graphics={
            Rectangle(
              lineColor={200,200,200},
              fillColor={248,248,248},
              fillPattern=FillPattern.HorizontalCylinder,
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Rectangle(
              lineColor={128,128,128},
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={78,138,73},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
    end One_pass_HX;

    package Multiple_pass_HX
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

      model HX_3_pass_Test
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
        Modelica.Blocks.Sources.Step step(height=100, startTime=100) annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=-20,
          duration=60,
          startTime=300) annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
        HeatExchangers.Multiple_pass_HX.HX_3_pass hX_3_pass annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{10,10},{-10,-10}},
              rotation=180,
              origin={44,40})));
        WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
              extent={{10,-10},{-10,10}},
              rotation=180,
              origin={-44,-40})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(hot_source.C_out,hX_3_pass. fg_inlet) annotation (Line(points={{-39,0},{-10,0}}, color={95,95,95}));
        connect(hX_3_pass.fg_outlet, hot_sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={95,95,95}));
        connect(hX_3_pass.inlet, cold_source.C_out) annotation (Line(points={{-6,-10},{-6,-40},{-39,-40}},
                                                                                                     color={28,108,200}));
        connect(hX_3_pass.outlet, cold_sink.C_in) annotation (Line(points={{6,10},{6,40},{39,40}},    color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor={0,140,72},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor={0,140,72},
                      fillColor={0,140,72},
                      pattern=LinePattern.None,
                      fillPattern=FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},{60,60}})),
          experiment(
            StopTime=500,
            __Dymola_NumberOfIntervals=1000,
            __Dymola_Algorithm="Dassl"));
      end HX_3_pass_Test;

      model HX_4_pass_Test
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
        HeatExchangers.Multiple_pass_HX.HX_4_pass HX_4_pass annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
        WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=180,
              origin={44,40})));
        WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-44,40})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(hot_source.C_out, HX_4_pass.fg_inlet) annotation (Line(points={{-39,0},{-10,0}}, color={95,95,95}));
        connect(HX_4_pass.fg_outlet, hot_sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={95,95,95}));
        connect(cold_source.C_out, HX_4_pass.inlet) annotation (Line(points={{-39,40},{-6,40},{-6,10}}, color={28,108,200}));
        connect(cold_sink.C_in, HX_4_pass.outlet) annotation (Line(points={{39,40},{6,40},{6,10}}, color={28,108,200}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor={0,140,72},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor={0,140,72},
                      fillColor={0,140,72},
                      pattern=LinePattern.None,
                      fillPattern=FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}),  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},{60,60}})),
          experiment(
            StopTime=500,
            __Dymola_NumberOfIntervals=1000,
            __Dymola_Algorithm="Dassl"));
      end HX_4_pass_Test;
      annotation (Icon(graphics={
            Rectangle(
              lineColor={200,200,200},
              fillColor={248,248,248},
              fillPattern=FillPattern.HorizontalCylinder,
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Rectangle(
              lineColor={128,128,128},
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={78,138,73},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
    end Multiple_pass_HX;

    package Simplified

      model Base_model_OHTC_Test
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
        HeatExchangers.Simplified.Base_model_OHTC                          base_model_OHTC(N=1)                    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(hot_source.C_out, base_model_OHTC.fg_inlet) annotation (Line(points={{-79,0},{-4,0}}, color={95,95,95}));
        connect(base_model_OHTC.fg_outlet, hot_sink.C_in) annotation (Line(points={{4,0},{79,0}}, color={95,95,95}));
        connect(base_model_OHTC.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-45.5},{2.77556e-16,-45.5},{2.77556e-16,-81}}, color={28,108,200}));
        connect(base_model_OHTC.water_outlet, cold_sink.C_in) annotation (Line(points={{0,10},{0,44.5},{-9.4369e-16,44.5},{-9.4369e-16,79}}, color={28,108,200}));
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
      end Base_model_OHTC_Test;

      model Base_model_OHTC_Test2
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;

          // Boundary conditions
        input Real P_hot_source(start = 1, min = 0, nominal = 1) "barA";
        input Real Q_hot_source(start = 661.61) "kg/s";
        input Utilities.Units.Temperature T_hot_source(start = 619.8) "degC";

        input Real P_cold_source(start = 165.4, min = 1.5, nominal = 100) "barA";
        input Utilities.Units.MassFlowRate Q_cold_source(start = 88.08) "kg/s";
        input Real T_cold_source(start = 535.5, min = 130, nominal = 150) "degC";

        WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=90,
              origin={0,-86})));
        WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{10,-10},{-10,10}},
              rotation=270,
              origin={0,84})));
        FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
        FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{74,-10},{94,10}})));
        HeatExchangers.Simplified.Reference_Model                          reference_Model(
          D_out=38.1e-3,
          e=5.2e-3,
          L=19.81,
          N_tubes_row=126,
          Rows=4,
          fg_path_width=14.52,
          S_T=88e-3,
          S_L=117e-3,
          S_f=0.00833,
          H_fin=0.01295,
          e_fin=0.8e-3,
          N=10)                                                                                                    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15;
        hot_source.Q_out = - Q_hot_source;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(hot_source.C_out, reference_Model.fg_inlet) annotation (Line(points={{-79,0},{-4,0}}, color={95,95,95}));
        connect(reference_Model.fg_outlet, hot_sink.C_in) annotation (Line(points={{4,0},{79,0}}, color={95,95,95}));
        connect(reference_Model.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-81}},                                         color={28,108,200}));
        connect(reference_Model.water_outlet, cold_sink.C_in) annotation (Line(points={{0,10},{0,79}},                                       color={28,108,200}));
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
      end Base_model_OHTC_Test2;

      model Base_model_OHTC_Test3
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;

          // Boundary conditions
        input Real P_hot_source(start = 1, min = 0, nominal = 1) "barA";
        input Real Q_hot_source(start = 661.61) "kg/s";
        input Utilities.Units.Temperature T_hot_source(start = 619.8) "degC";

        input Real P_cold_source(start = 165.4, min = 1.5, nominal = 100) "barA";
        input Utilities.Units.MassFlowRate Q_cold_source(start = 88.08) "kg/s";
        input Real T_cold_source(start = 535.5, min = 130, nominal = 150) "degC";

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
        HeatExchangers.One_pass_HX.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM
                                                                           crossCurrent_1NodePerRow_MonoPhasicHX_LCM(
          D_out=38.1e-3,
          e=5.2e-3,
          L=19.81,
          N_tubes_row=126,
          Rows=4,
          fg_path_width=14.52,
          S_T=88e-3,
          S_L=117e-3,
          S_f=0.00833,
          H_fin=0.01295,
          e_fin=0.8e-3,
          eff_fins=0.75,
          N=10)                                                                                                    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(hot_source.C_out, crossCurrent_1NodePerRow_MonoPhasicHX_LCM.fg_inlet) annotation (Line(points={{-79,0},{-4,0}}, color={95,95,95}));
        connect(crossCurrent_1NodePerRow_MonoPhasicHX_LCM.fg_outlet, hot_sink.C_in) annotation (Line(points={{4,0},{79,0}}, color={95,95,95}));
        connect(crossCurrent_1NodePerRow_MonoPhasicHX_LCM.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-45.5},{2.77556e-16,-45.5},{2.77556e-16,-81}}, color={28,108,200}));
        connect(crossCurrent_1NodePerRow_MonoPhasicHX_LCM.water_outlet, cold_sink.C_in) annotation (Line(points={{0,10},{0,44.5},{-9.4369e-16,44.5},{-9.4369e-16,79}}, color={28,108,200}));
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
      end Base_model_OHTC_Test3;

      model Simplified_model_Test
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;

          // Boundary conditions
        input Real P_hot_source(start = 1, min = 0, nominal = 1) "barA";
        input Real Q_hot_source(start = 661.61) "kg/s";
        input Utilities.Units.Temperature T_hot_source(start = 619.8) "degC";

        input Real P_cold_source(start = 165.4, min = 1.5, nominal = 100) "barA";
        input Utilities.Units.MassFlowRate Q_cold_source(start = 88.08) "kg/s";
        input Real T_cold_source(start = 535.5, min = 130, nominal = 150) "degC";

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
        HeatExchangers.Simplified.Simplified_model HX                                     annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15;
        hot_source.Q_out = - Q_hot_source;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        HX.water_side.T_out = 588.2 + 273.15;


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
      end Simplified_model_Test;

      model Simplified_model_1_Test
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;

          // Boundary conditions
        input Real P_hot_source(start = 1, min = 0, nominal = 1) "barA";
        input Real Q_hot_source(start = 661.61) "kg/s";
        input Utilities.Units.Temperature T_hot_source(start = 619.8) "degC";

        input Real P_cold_source(start = 165.4, min = 1.5, nominal = 100) "barA";
        input Utilities.Units.MassFlowRate Q_cold_source(start = 88.08) "kg/s";
        input Real T_cold_source(start = 535.5, min = 130, nominal = 150) "degC";

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
        HeatExchangers.Simplified.Simplified_model HX                                     annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15;
        hot_source.Q_out = - Q_hot_source;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        // HX.water_side.T_out = 588.2 + 273.15;

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
      end Simplified_model_1_Test;

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

      model Reference_Model_B
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
        HeatExchangers.Simplified.Reference_Model                          reference_Model(N=10)                   annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(hot_source.C_out,reference_Model. fg_inlet) annotation (Line(points={{-79,0},{-4,0}}, color={95,95,95}));
        connect(reference_Model.fg_outlet, hot_sink.C_in) annotation (Line(points={{4,0},{79,0}}, color={95,95,95}));
        connect(reference_Model.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-45.5},{2.77556e-16,-45.5},{2.77556e-16,-81}}, color={28,108,200}));
        connect(reference_Model.water_outlet, cold_sink.C_in) annotation (Line(points={{0,10},{0,44.5},{-9.4369e-16,44.5},{-9.4369e-16,79}}, color={28,108,200}));
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
      end Reference_Model_B;

      model Simplified_model_B
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
        HeatExchangers.Simplified.Simplified_model                         simplified_model(UA=153805.22)          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        connect(hot_source.C_out, simplified_model.fg_inlet) annotation (Line(points={{-79,0},{-4,0}}, color={95,95,95}));
        connect(simplified_model.fg_outlet, hot_sink.C_in) annotation (Line(points={{4,0},{79,0}}, color={95,95,95}));
        connect(simplified_model.water_inlet, cold_source.C_out) annotation (Line(points={{0,-10},{0,-45.5},{2.77556e-16,-45.5},{2.77556e-16,-81}}, color={28,108,200}));
        connect(simplified_model.water_outlet, cold_sink.C_in) annotation (Line(points={{0,10},{0,44.5},{-9.4369e-16,44.5},{-9.4369e-16,79}}, color={28,108,200}));
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
      end Simplified_model_B;

      model Simplified_model_1_B
        import MetroscopeModelingLibrary.Utilities.Units;
        import MetroscopeModelingLibrary.Utilities.Units.Inputs;

          // Boundary conditions
        input Real P_hot_source(start = 1, min = 0, nominal = 1) "barA";
        input Real Q_hot_source(start = 658.695) "kg/s";
        input Utilities.Units.Temperature T_hot_source(start = 633.7) "degC";

        input Real P_cold_source(start = 121.2, min = 1.5, nominal = 100) "barA";
        input Utilities.Units.MassFlowRate Q_cold_source(start = 84.06) "kg/s";
        input Real T_cold_source(start = 498.8, min = 130, nominal = 150) "degC";

        input Modelica.Units.SI.ThermalConductance UA(start=155746.7); // Ref = 153805.22

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
        HeatExchangers.Simplified.Simplified_model_1 HX(r_UA=8.596588, T_wall_0=790.15) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        HX.UA = UA;

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
      end Simplified_model_1_B;

      model Simplified_model_1_reverse_B
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
        HeatExchangers.Simplified.Simplified_model_1 HX(r_UA=8.596588, T_wall_0=790.15) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
        hot_source.P_out = P_hot_source*1e5;
        hot_source.T_out = T_hot_source + 273.15 + ramp.y;
        hot_source.Q_out = - Q_hot_source + step.y;

        cold_source.P_out = P_cold_source*1e5;
        cold_source.T_out = 273.15 + T_cold_source;
        cold_source.Q_out = - Q_cold_source;

        HX.T_water_out = 564.3427 + 273.15;

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
      end Simplified_model_1_reverse_B;
      annotation (Icon(graphics={
            Rectangle(
              lineColor={200,200,200},
              fillColor={248,248,248},
              fillPattern=FillPattern.HorizontalCylinder,
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Rectangle(
              lineColor={128,128,128},
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={78,138,73},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
    end Simplified;

    package TwoPhaseHX
      model Steam_Drum_Test

        output Utilities.Units.Pressure P_FW_source;
        output Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
        input Real T_FW_source(start = 315.6, min = 130, nominal = 150);


        HeatExchangers.TwoPhaseHX.SteamDrum steamDrum annotation (Placement(transformation(extent={{-10,40},{10,60}})));
        WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
        WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={84,20})));
        WaterSteam.Pipes.HeatLoss evaporator annotation (Placement(transformation(extent={{-20,-50},{-40,-30}})));
      equation

        FW_source.P_out = P_FW_source*1e5;
        FW_source.T_out = 273.15 + T_FW_source;
        FW_source.Q_out = - Q_FW_source;


        connect(steamDrum.steam_out, Steam_sink.C_in) annotation (Line(
            points={{-6,58},{-6,60},{-79,60}},
            color={238,46,47},
            thickness=1,
            pattern=LinePattern.Dash));
        connect(steamDrum.fw_in, FW_source.C_out) annotation (Line(
            points={{8.2,44},{34,44},{34,20},{79,20}},
            color={28,108,200},
            thickness=1));
        connect(evaporator.C_in, steamDrum.downcomers_out) annotation (Line(
            points={{-20,-40},{0,-40},{0,40}},
            color={28,108,200},
            thickness=1));
        connect(evaporator.C_out, steamDrum.risers_in) annotation (Line(
            points={{-40,-40},{-60,-40},{-60,44},{-8,44}},
            color={238,46,47},
            thickness=1,
            pattern=LinePattern.Dash));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor={0,140,72},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor={0,140,72},
                      fillColor={0,140,72},
                      pattern=LinePattern.None,
                      fillPattern=FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end Steam_Drum_Test;

      model SteamDrum_Astom_Simple_Test

        //input Utilities.Units.Pressure P_FW_source(start=121);

        output Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
        input Real T_FW_source(start = 315.6, min = 130, nominal = 150);
        output Real Q_s(start = 84.06);
        input Real W_evap(start=104.60049e6);

        HeatExchangers.TwoPhaseHX.SteamDrum_Astom_Simple
                                            steamDrum_Astom_Simple(Controlled_level=false)
                                                      annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
        WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
        WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={84,-60})));
        Modelica.Blocks.Sources.Step step(height=2e6,startTime=100)  annotation (Placement(transformation(extent={{60,60},{80,80}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=10e6,
          duration=60,
          startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
      equation

        FW_source.P_out = 121*1e5;
        FW_source.T_out = 273.15 + T_FW_source;
        FW_source.Q_out = - Q_FW_source;
        Steam_sink.Q_in = Q_s;  //+ ramp.y
        steamDrum_Astom_Simple.W_evap = W_evap + step.y;  //+ step.y
        Q_s = 84.06/104.6649e6*steamDrum_Astom_Simple.W_evap;
        //steamDrum_Astom_Simple.P_drum = 121e5/104.6e6*steamDrum_Astom_Simple.W_evap;


        connect(steamDrum_Astom_Simple.steam_out, Steam_sink.C_in) annotation (Line(
            points={{0,14},{0,60},{-79,60}},
            color={238,46,47},
            thickness=1,
            pattern=LinePattern.Dash));
        connect(steamDrum_Astom_Simple.fw_in, FW_source.C_out) annotation (Line(
            points={{14.2,0},{70,0},{70,-60},{79,-60}},
            color={28,108,200},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor={0,140,72},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor={0,140,72},
                      fillColor={0,140,72},
                      pattern=LinePattern.None,
                      fillPattern=FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
      end SteamDrum_Astom_Simple_Test;

      model SteamDrum_Sunil_Test

        output Utilities.Units.Pressure P_FW_source;

        input Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
        input Real T_FW_source(start = 300, min = 130, nominal = 150);
        input Real Q_s(start = 84.06);
        input Real W_evap(start=200.60049e6);

        HeatExchangers.TwoPhaseHX.SteamDrum_Sunil steamDrum_Astom_Complete(Controlled_level=true)  annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
        WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
        WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={84,-60})));
        Modelica.Blocks.Sources.Step step(height=7e6,startTime=100)  annotation (Placement(transformation(extent={{60,60},{80,80}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=10e6,
          duration=60,
          startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
      equation

        FW_source.P_out = P_FW_source*1e5;
        FW_source.T_out = 273.15 + T_FW_source;
        FW_source.Q_out = - Q_FW_source;
        Steam_sink.Q_in = Q_s;
        //evaporator.W = W_evap;

        connect(steamDrum_Astom_Complete.steam_out, Steam_sink.C_in) annotation (Line(
            points={{0,14},{0,60},{-79,60}},
            color={238,46,47},
            thickness=1,
            pattern=LinePattern.Dash));
        connect(steamDrum_Astom_Complete.fw_in, FW_source.C_out) annotation (Line(
            points={{14.2,0},{70,0},{70,-60},{79,-60}},
            color={28,108,200},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor={0,140,72},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor={0,140,72},
                      fillColor={0,140,72},
                      pattern=LinePattern.None,
                      fillPattern=FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
      end SteamDrum_Sunil_Test;

      model SteamDrum_Sunil_noConToEvap_Test

        output Utilities.Units.Pressure P_FW_source;

        input Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
        input Real T_FW_source(start = 315.6, min = 130, nominal = 150);
        input Real Q_s(start = 84.06);
        input Real W_evap(start=104.60049e6);

        HeatExchangers.TwoPhaseHX.SteamDrum_Sunil_noConToEvap
                                                  steamDrum_Astom_Complete(Controlled_level=false) annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
        WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
        WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={84,-60})));
        Modelica.Blocks.Sources.Step step(height=7e6,startTime=100)  annotation (Placement(transformation(extent={{60,60},{80,80}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=10e6,
          duration=60,
          startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
      equation

        FW_source.P_out = P_FW_source*1e5;
        FW_source.T_out = 273.15 + T_FW_source;
        FW_source.Q_out = - Q_FW_source;
        Steam_sink.Q_in = Q_s;
        //evaporator.W = W_evap;

        connect(steamDrum_Astom_Complete.steam_out, Steam_sink.C_in) annotation (Line(
            points={{0,14},{0,60},{-79,60}},
            color={238,46,47},
            thickness=1,
            pattern=LinePattern.Dash));
        connect(steamDrum_Astom_Complete.fw_in, FW_source.C_out) annotation (Line(
            points={{14.2,0},{70,0},{70,-60},{79,-60}},
            color={28,108,200},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor={0,140,72},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor={0,140,72},
                      fillColor={0,140,72},
                      pattern=LinePattern.None,
                      fillPattern=FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
      end SteamDrum_Sunil_noConToEvap_Test;

      model SteamDrum_TSP_Test

        output Utilities.Units.Pressure P_FW_source;

        input Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
        input Real T_FW_source(start = 315.6, min = 130, nominal = 150);
        input Real Q_s(start = 84.06);
        input Real W_evap(start=104.60049e6);

        HeatExchangers.TwoPhaseHX.SteamDrum_TSP
                                            steamDrum_TSP(Controlled_level=false)
                                                      annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
        WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
        WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={84,-60})));
        Modelica.Blocks.Sources.Step step(height=7e6,startTime=100)  annotation (Placement(transformation(extent={{60,60},{80,80}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=10e6,
          duration=60,
          startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
      equation

        FW_source.P_out = P_FW_source*1e5;

        steamDrum_TSP.P_drum = 121e5;

        FW_source.T_out = 273.15 + T_FW_source;
        //FW_source.Q_out = Q_FW_source;
        Steam_sink.Q_in = Q_s;  //+ ramp.y
       steamDrum_TSP.W_evap = W_evap;           //+ step.y
        //rsteamDrum_TSP.x_r = 0.5; //2675842.2;
        //steamDrum_TSP.h_d = riser.h_out;
        //evap.W = W_evap;

        connect(steamDrum_TSP.steam_out, Steam_sink.C_in) annotation (Line(
            points={{0,14},{0,60},{-79,60}},
            color={238,46,47},
            thickness=1,
            pattern=LinePattern.Dash));
        connect(steamDrum_TSP.fw_in, FW_source.C_out) annotation (Line(
            points={{14.2,0},{70,0},{70,-60},{79,-60}},
            color={28,108,200},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor={0,140,72},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor={0,140,72},
                      fillColor={0,140,72},
                      pattern=LinePattern.None,
                      fillPattern=FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
      end SteamDrum_TSP_Test;

      model SteamDrum_Astom_Simple_withLoop_Test

        //input Utilities.Units.Pressure P_FW_source(start=121);

        output Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
        input Real T_FW_source(start = 315.6, min = 130, nominal = 150);
        input Real Q_s(start = 84.06);
        input Real W_evap(start=104.60049e6);
        input Real x_r(start=0.5);

        HeatExchangers.TwoPhaseHX.SteamDrum_Astom_Simple_withLoop
                                            steamDrum_Astom_Simple_withLoop(Controlled_level=true)
                                                      annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
        WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
        WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={84,-60})));
        Modelica.Blocks.Sources.Step step(height=2e6,startTime=100)  annotation (Placement(transformation(extent={{60,60},{80,80}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=10e6,
          duration=60,
          startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
      equation

        //FW_source.P_out = 121*1e5;
        FW_source.T_out = 273.15 + T_FW_source;
        FW_source.Q_out = - Q_FW_source;
        Steam_sink.Q_in = Q_s;  //+ ramp.y
        steamDrum_Astom_Simple_withLoop.W_evap = W_evap + step.y;
                                                          //+ step.y
        //Q_s =84.06/104.6649e6*steamDrum_Astom_Simple_withLoop.W_evap;
        //steamDrum_Astom_Simple.P_drum = 121e5/104.6e6*steamDrum_Astom_Simple.W_evap;
        steamDrum_Astom_Simple_withLoop.x_r = 0.5;

        connect(steamDrum_Astom_Simple_withLoop.steam_out, Steam_sink.C_in) annotation (Line(
            points={{0,14},{0,60},{-79,60}},
            color={238,46,47},
            thickness=1,
            pattern=LinePattern.Dash));
        connect(steamDrum_Astom_Simple_withLoop.fw_in, FW_source.C_out) annotation (Line(
            points={{14.2,0},{70,0},{70,-60},{79,-60}},
            color={28,108,200},
            thickness=1));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor={0,140,72},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor={0,140,72},
                      fillColor={0,140,72},
                      pattern=LinePattern.None,
                      fillPattern=FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
      end SteamDrum_Astom_Simple_withLoop_Test;

      model SteamDrum_Astom_Simple_withLoop_AddConnectors_Test

        //input Utilities.Units.Pressure P_FW_source(start=121);

        output Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
        input Real T_FW_source(start = 315.6, min = 130, nominal = 150);
        input Real Q_s(start = 84.06);
        input Real W_evap(start=105.18123e6);
        input Real x_r(start=0.5);

        HeatExchangers.TwoPhaseHX.SteamDrum_Astom_Simple_withLoop_AddConnectors
                                            steamDrum_Astom_Simple_withLoop_AddConnectors(Controlled_level=false)
                                                      annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
        WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
        WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={84,-60})));
        Modelica.Blocks.Sources.Step step(height=2e6,startTime=100)  annotation (Placement(transformation(extent={{20,60},{40,80}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=2e6,
          duration=20,
          startTime=100) annotation (Placement(transformation(extent={{20,20},{40,40}})));
        WaterSteam.Pipes.HeatLoss risers annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=270,
              origin={-20,-52})));
        Modelica.Blocks.Sources.Step step1(height=-2e6, startTime=200)
                                                                     annotation (Placement(transformation(extent={{60,60},{80,80}})));
        Modelica.Blocks.Sources.Ramp ramp1(
          height=-2e6,
          duration=20,
          startTime=200) annotation (Placement(transformation(extent={{60,20},{80,40}})));
      equation

        FW_source.P_out = 121e5 + 0.1*ramp.y + 0.1*ramp1.y;
        FW_source.T_out = 273.15 + T_FW_source;
        FW_source.Q_out = - Q_FW_source;
        Steam_sink.Q_in = Q_s + 1e-6*ramp.y + 1e-6*ramp1.y;  //+ ramp.y
        steamDrum_Astom_Simple_withLoop_AddConnectors.W_evap = W_evap + ramp.y + ramp1.y;
        //steamDrum_Astom_Simple_withLoop_AddConnectors.Q_loop = 180;
                                                          //+ step.y
        //Q_s =84.06/104.6649e6*steamDrum_Astom_Simple_withLoop.W_evap;
        //steamDrum_Astom_Simple.P_drum = 121e5/104.6e6*steamDrum_Astom_Simple.W_evap;
        steamDrum_Astom_Simple_withLoop_AddConnectors.x_r = 0.5;
        //der(steamDrum_Astom_Simple_withLoop_AddConnectors.P_drum) = 0;





        connect(steamDrum_Astom_Simple_withLoop_AddConnectors.steam_out, Steam_sink.C_in) annotation (Line(
            points={{0,14},{0,60},{-79,60}},
            color={238,46,47},
            thickness=1,
            pattern=LinePattern.Dash));
        connect(steamDrum_Astom_Simple_withLoop_AddConnectors.fw_in, FW_source.C_out) annotation (Line(
            points={{14.2,0},{70,0},{70,-60},{79,-60}},
            color={28,108,200},
            thickness=1));
        connect(risers.C_in, steamDrum_Astom_Simple_withLoop_AddConnectors.downcomers_out) annotation (Line(
            points={{-20,-62},{-20,-80},{6,-80},{6,-4}},
            color={28,108,200},
            thickness=1));
        connect(risers.C_out, steamDrum_Astom_Simple_withLoop_AddConnectors.risers_in) annotation (Line(
            points={{-20,-42},{-20,0},{-2,0}},
            color={238,46,47},
            thickness=1,
            pattern=LinePattern.Dash));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor={0,140,72},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor={0,140,72},
                      fillColor={0,140,72},
                      pattern=LinePattern.None,
                      fillPattern=FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
      end SteamDrum_Astom_Simple_withLoop_AddConnectors_Test;

      model SteamDrum_Astom_Sunil_Prog_Test

        //input Utilities.Units.Pressure P_FW_source(start=121);

        input Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
        input Real T_FW_source(start = 315.6, min = 130, nominal = 150);
        output Real Q_s(start = 84.06);
        input Real W_evap(start=104.60049e6);
        input Real x_r(start=0.5);

        HeatExchangers.TwoPhaseHX.SteamDrum_Astom_Sunil_Prog_1
                                            steamDrum_Astom_Sunil_Prog_1_1(steady_state=true, Controlled_level=false)
                                                      annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
        WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
        WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=180,
              origin={84,-60})));
        Modelica.Blocks.Sources.Step step(height=2e6,startTime=100)  annotation (Placement(transformation(extent={{60,60},{80,80}})));
        Modelica.Blocks.Sources.Ramp ramp(
          height=10e6,
          duration=60,
          startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
        WaterSteam.Pipes.HeatLoss risers annotation (Placement(transformation(
              extent={{10,10},{-10,-10}},
              rotation=270,
              origin={-20,-52})));
      equation

        FW_source.P_out = 121*1e5;
        FW_source.T_out = 273.15 + T_FW_source;
        FW_source.Q_out = - Q_FW_source;
        Steam_sink.Q_in = Q_s;  //+ ramp.y
        steamDrum_Astom_Sunil_Prog_1_1.W_evap = W_evap;

                                                          //+ step.y
        //Q_s =84.06/104.6649e6*steamDrum_Astom_Simple_withLoop.W_evap;
        //steamDrum_Astom_Simple.P_drum = 121e5/104.6e6*steamDrum_Astom_Simple.W_evap;
        steamDrum_Astom_Sunil_Prog_1_1.x_r = x_r;

        connect(steamDrum_Astom_Sunil_Prog_1_1.steam_out, Steam_sink.C_in) annotation (Line(
            points={{0,14},{0,60},{-79,60}},
            color={238,46,47},
            thickness=1,
            pattern=LinePattern.Dash));
        connect(steamDrum_Astom_Sunil_Prog_1_1.fw_in, FW_source.C_out) annotation (Line(
            points={{14.2,0},{70,0},{70,-60},{79,-60}},
            color={28,108,200},
            thickness=1));
        connect(risers.C_in, steamDrum_Astom_Sunil_Prog_1_1.downcomers_out) annotation (Line(
            points={{-20,-62},{-20,-80},{6,-80},{6,-4}},
            color={28,108,200},
            thickness=1));
        connect(risers.C_out, steamDrum_Astom_Sunil_Prog_1_1.risers_in) annotation (Line(
            points={{-20,-42},{-20,0},{-2,0}},
            color={238,46,47},
            thickness=1,
            pattern=LinePattern.Dash));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
              Ellipse(lineColor={0,140,72},
                      fillColor={255,255,255},
                      fillPattern=FillPattern.Solid,
                      extent={{-100,-100},{100,100}}),
              Polygon(lineColor={0,140,72},
                      fillColor={0,140,72},
                      pattern=LinePattern.None,
                      fillPattern=FillPattern.Solid,
                      points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
          experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
      end SteamDrum_Astom_Sunil_Prog_Test;

      package SteamDrumModels
        model SteamDrum_Astom_1_Test

          //input Utilities.Units.Pressure P_FW_source(start=121);

          output Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
          input Real T_FW_source(start = 315.6, min = 130, nominal = 150);
          output Real Q_s(start = 84.06);
          input Real W_evap(start=104.60049e6);

          HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Astom_1
                                              steamDrum_Astom_1_1(   Controlled_level=false)
                                                        annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
          WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
          WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={84,-60})));
          Modelica.Blocks.Sources.Step step(height=2e6,startTime=100)  annotation (Placement(transformation(extent={{60,60},{80,80}})));
          Modelica.Blocks.Sources.Ramp ramp(
            height=10e6,
            duration=60,
            startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
        equation

          FW_source.P_out = 121*1e5;
          FW_source.T_out = 273.15 + T_FW_source;
          FW_source.Q_out = - Q_FW_source;
          Steam_sink.Q_in = Q_s;  //+ ramp.y
          steamDrum_Astom_1_1.W_evap = W_evap + step.y;     //+ step.y
          Q_s =84.06/104.6649e6*steamDrum_Astom_1_1.W_evap;
          //steamDrum_Astom_Simple.P_drum = 121e5/104.6e6*steamDrum_Astom_Simple.W_evap;

          connect(steamDrum_Astom_1_1.steam_out, Steam_sink.C_in) annotation (Line(
              points={{0,14},{0,60},{-79,60}},
              color={238,46,47},
              thickness=1,
              pattern=LinePattern.Dash));
          connect(steamDrum_Astom_1_1.fw_in, FW_source.C_out) annotation (Line(
              points={{14.2,0},{70,0},{70,-60},{79,-60}},
              color={28,108,200},
              thickness=1));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Ellipse(lineColor={0,140,72},
                        fillColor={255,255,255},
                        fillPattern=FillPattern.Solid,
                        extent={{-100,-100},{100,100}}),
                Polygon(lineColor={0,140,72},
                        fillColor={0,140,72},
                        pattern=LinePattern.None,
                        fillPattern=FillPattern.Solid,
                        points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
            experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
        end SteamDrum_Astom_1_Test;

        model SteamDrum_Astom_2_Test

          //input Utilities.Units.Pressure P_FW_source(start=121);

          output Utilities.Units.MassFlowRate Q_FW_source(start = 84.06);
          input Real T_FW_source(start = 315.6, min = 130, nominal = 150);
          output Real Q_s(start = 84.06);
          input Real W_evap(start=104.60049e6);

          HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Astom_2
                                              steamDrum_Astom_2_1(   Controlled_level=false)
                                                        annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
          WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
          WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={84,-60})));
          Modelica.Blocks.Sources.Step step(height=2e6,startTime=100)  annotation (Placement(transformation(extent={{60,60},{80,80}})));
          Modelica.Blocks.Sources.Ramp ramp(
            height=10e6,
            duration=60,
            startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
        equation

          FW_source.P_out = 121*1e5;
          FW_source.T_out = 273.15 + T_FW_source;
          FW_source.Q_out = - Q_FW_source;
          Steam_sink.Q_in = Q_s;  //+ ramp.y
          steamDrum_Astom_2_1.W_evap = W_evap + step.y;     //+ step.y
          Q_s =84.06/104.6649e6*steamDrum_Astom_2_1.W_evap;
          //steamDrum_Astom_Simple.P_drum = 121e5/104.6e6*steamDrum_Astom_Simple.W_evap;
          steamDrum_Astom_2_1.Q_b = 80;

          connect(steamDrum_Astom_2_1.steam_out, Steam_sink.C_in) annotation (Line(
              points={{0,14},{0,60},{-79,60}},
              color={238,46,47},
              thickness=1,
              pattern=LinePattern.Dash));
          connect(steamDrum_Astom_2_1.fw_in, FW_source.C_out) annotation (Line(
              points={{14.2,0},{70,0},{70,-60},{79,-60}},
              color={28,108,200},
              thickness=1));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Ellipse(lineColor={0,140,72},
                        fillColor={255,255,255},
                        fillPattern=FillPattern.Solid,
                        extent={{-100,-100},{100,100}}),
                Polygon(lineColor={0,140,72},
                        fillColor={0,140,72},
                        pattern=LinePattern.None,
                        fillPattern=FillPattern.Solid,
                        points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
            experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
        end SteamDrum_Astom_2_Test;

        model SteamDrum_Astom_SimplifiedGeometry_3

          //input Utilities.Units.Pressure P_FW_source(start=121);

          input Utilities.Units.MassFlowRate Q_FW_source(start = 50.44);
          input Real T_FW_source(start = 241.90323, min = 0, nominal = 150);
          output Real Q_s(start = 60);
          input Real W_evap(start=85.94833e6);

          HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Astom_SimplifiedGeometry_3 Evap annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
          WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
          WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={84,-62})));
          Modelica.Blocks.Sources.Step step(height=10e6, startTime=50) annotation (Placement(transformation(extent={{60,60},{80,80}})));
          Modelica.Blocks.Sources.Ramp ramp(
            height=10e6,
            duration=60,
            startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
        equation

          //P_out = 85*1e5;
          FW_source.T_out = 273.15 + T_FW_source;
          FW_source.Q_out = - Q_FW_source;

          Steam_sink.Q_in = Q_s;  //+ ramp.y
          Evap.W_evap = W_evap + step.y;

          //Evap.Q_ct = 10.5; // h_f
          //Evap.Q_dc = 1195; // A_dc
          Evap.A_dc = 0.381601;
         // Evap.V_sd = 4.9; // V_0_sd
          Evap.V_0_sd = 7.662651;
          Q_FW_source = Q_s;
          //Evap.x_r = 0.051;

          connect(Evap.steam_out, Steam_sink.C_in) annotation (Line(
              points={{0,14},{0,60},{-79,60}},
              color={238,46,47},
              thickness=1,
              pattern=LinePattern.Dash));
          connect(Evap.fw_in, FW_source.C_out) annotation (Line(
              points={{14.2,0},{70,0},{70,-62},{79,-62}},
              color={28,108,200},
              thickness=1));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Ellipse(lineColor={0,140,72},
                        fillColor={255,255,255},
                        fillPattern=FillPattern.Solid,
                        extent={{-100,-100},{100,100}}),
                Polygon(lineColor={0,140,72},
                        fillColor={0,140,72},
                        pattern=LinePattern.None,
                        fillPattern=FillPattern.Solid,
                        points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
            experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
        end SteamDrum_Astom_SimplifiedGeometry_3;

        model SteamDrum_Astom_SimplifiedGeometry_4

          //input Utilities.Units.Pressure P_FW_source(start=121);

          input Utilities.Units.MassFlowRate Q_FW_source(start = 70);
          input Real T_FW_source(start = 244.88, min = 0, nominal = 150);
          input Real Q_s(start = 70);
          input Real W_evap(start=118.27767e6);

          HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Astom_SimplifiedGeometry_4 Evap annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
          WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
          WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={84,-62})));
          Modelica.Blocks.Sources.Step step(height=10e6, startTime=50) annotation (Placement(transformation(extent={{60,60},{80,80}})));
          Modelica.Blocks.Sources.Ramp ramp(
            height=10e6,
            duration=60,
            startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
        equation

          //FW_source.P_out = 85*1e5;
          FW_source.T_out = 273.15 + T_FW_source;
          FW_source.Q_out = - Q_FW_source;
          Steam_sink.Q_in = Q_s;  //+ ramp.y
          Evap.W_evap = W_evap + step.y;

          // Calibration
          //Evap.Q_cd = 10.5; // h_f
          //Evap.Q_dc = 1195; // A_dc
          //Evap.V_sd = 4.9; // V_0_sd



          connect(Evap.steam_out, Steam_sink.C_in) annotation (Line(
              points={{0,14},{0,60},{-79,60}},
              color={238,46,47},
              thickness=1,
              pattern=LinePattern.Dash));
          connect(Evap.fw_in, FW_source.C_out) annotation (Line(
              points={{14.2,0},{70,0},{70,-62},{79,-62}},
              color={28,108,200},
              thickness=1));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Ellipse(lineColor={0,140,72},
                        fillColor={255,255,255},
                        fillPattern=FillPattern.Solid,
                        extent={{-100,-100},{100,100}}),
                Polygon(lineColor={0,140,72},
                        fillColor={0,140,72},
                        pattern=LinePattern.None,
                        fillPattern=FillPattern.Solid,
                        points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
            experiment(StopTime=200, __Dymola_Algorithm="Dassl"));
        end SteamDrum_Astom_SimplifiedGeometry_4;

        model SteamDrum_Astom_Sunil_2_Test

          //input Utilities.Units.Pressure P_FW_source(start=121);

          input Utilities.Units.MassFlowRate Q_FW_source(start = 70);
          input Real T_FW_source(start = 244.88, min = 0, nominal = 150);
          input Real Q_s(start = 70);
          input Real W_evap(start=118.27767e6);

          HeatExchangers.TwoPhaseHX.SteamDrumModels.SteamDrum_Sunil_2                    Evap(
            V_D=40,
            M_D=140000,
            Cp_D=472)                                                                         annotation (Placement(transformation(extent={{-4,-4},{16,16}})));
          WaterSteam.BoundaryConditions.Sink Steam_sink annotation (Placement(transformation(extent={{-74,50},{-94,70}})));
          WaterSteam.BoundaryConditions.Source FW_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={84,-62})));
          Modelica.Blocks.Sources.Step step(height=10e6, startTime=50) annotation (Placement(transformation(extent={{60,60},{80,80}})));
          Modelica.Blocks.Sources.Ramp ramp(
            height=10e6,
            duration=60,
            startTime=100) annotation (Placement(transformation(extent={{60,20},{80,40}})));
          WaterSteam.BoundaryConditions.Sink d_sink annotation (Placement(transformation(
                extent={{10,-10},{-10,10}},
                rotation=90,
                origin={6,-44})));
          WaterSteam.BoundaryConditions.Source r_source annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={-20,-44})));
          WaterSteam.Pipes.HeatLoss Heat annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={-20,-16})));
        equation

          FW_source.P_out = 85*1e5;
          FW_source.T_out = 273.15 + T_FW_source;
          FW_source.Q_out = - Q_FW_source;
          Steam_sink.Q_in = Q_s;  //+ ramp.y
          //Evap.W_evap = W_evap + step.y;
          Heat.W = W_evap;
          Heat.h_in = Evap.h_d;
          //Evap.Q_d = - Q_FW_source*18.29;

          // Calibration
          //Evap.Q_cd = 10.5; // h_f
          //Evap.Q_dc = 1195; // A_dc
          //Evap.V_sd = 4.9; // V_0_sd

          connect(Evap.steam_out, Steam_sink.C_in) annotation (Line(
              points={{0,14},{0,60},{-79,60}},
              color={238,46,47},
              thickness=1,
              pattern=LinePattern.Dash));
          connect(Evap.fw_in, FW_source.C_out) annotation (Line(
              points={{14.2,0},{70,0},{70,-62},{79,-62}},
              color={28,108,200},
              thickness=1));
          connect(Evap.downcomers_out, d_sink.C_in) annotation (Line(points={{6,-4},{6,-39}}, color={28,108,200}));
          connect(r_source.C_out, Heat.C_in) annotation (Line(points={{-20,-39},{-20,-26}}, color={28,108,200}));
          connect(Heat.C_out, Evap.risers_in) annotation (Line(points={{-20,-6},{-20,0},{-2,0}}, color={28,108,200}));
          annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                Ellipse(lineColor={0,140,72},
                        fillColor={255,255,255},
                        fillPattern=FillPattern.Solid,
                        extent={{-100,-100},{100,100}}),
                Polygon(lineColor={0,140,72},
                        fillColor={0,140,72},
                        pattern=LinePattern.None,
                        fillPattern=FillPattern.Solid,
                        points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)),
            experiment(StopTime=200, __Dymola_Algorithm="Dassl"));
        end SteamDrum_Astom_Sunil_2_Test;
      end SteamDrumModels;
      annotation (Icon(graphics={
            Rectangle(
              lineColor={200,200,200},
              fillColor={248,248,248},
              fillPattern=FillPattern.HorizontalCylinder,
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Rectangle(
              lineColor={128,128,128},
              extent={{-100,-100},{100,100}},
              radius=25.0),
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={78,138,73},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
    end TwoPhaseHX;
    annotation (Icon(graphics={
          Rectangle(
            lineColor={200,200,200},
            fillColor={248,248,248},
            fillPattern=FillPattern.HorizontalCylinder,
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Rectangle(
            lineColor={128,128,128},
            extent={{-100,-100},{100,100}},
            radius=25.0),
          Polygon(
            origin={8,14},
            lineColor={78,138,73},
            fillColor={78,138,73},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
  end Tests;

  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),      Line(points={{-56,72}}, color={28,108,200}), Line(
          points={{-100,0},{-50,100},{50,-100},{100,0}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier)}));
end DynamicComponents;
