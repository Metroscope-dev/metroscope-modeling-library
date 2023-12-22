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

    //   // ------ Fluids properties ------
    //     // State
    //     WaterSteamMedium.ThermodynamicState state_water[Rows, N+1] "Water side node boundary state";
    //     FlueGasesMedium.ThermodynamicState state_fg[Rows + 1, N] "Flue gas side node boundary states";
    //     // Enthalpy
    //     Units.SpecificEnthalpy h_water[Rows, N+1](each start=h_water_out_0) "Water specific enthalpy";
    //     Units.SpecificEnthalpy h_fg[Rows + 1, N] "Flue gas specific enthalpy";
    //     // Mass flow rate
    //     Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
    //     Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
    //     // Pressure
    //     Units.Pressure P_water(start=P_water_0) "Water Pressure";
    //     Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
    //     // Density
    //     Units.Density rho_water[Rows, N+1] "Node boundary water density";
    //     Units.Density rho_water_node[Rows, N] "Node average water density";
    //     Units.Density rho_fg[Rows + 1, N] "Node boundary flue gas density";
    //     // Mass fraction
    //     Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
    //     Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
    //     // Temperature
    //     Units.Temperature T_water[Rows, N+1] "Node boundary water temperature";
    //     Units.Temperature T_water_node[Rows, N] "Node average water temperature";
    //     Units.Temperature T_fg[Rows + 1, N] "Node boundary flue gas temperature";
    //     Units.Temperature T_fg_node[Rows, N] "Node average flue gas temperature";
    //     // Dynamic viscosities
    //     Modelica.Units.SI.DynamicViscosity Mu_water[Rows, N+1] "Node boundary water dynamic viscosity";
    //     Modelica.Units.SI.DynamicViscosity Mu_water_node[Rows, N] "Node average water dynamic viscosity";
    //     Modelica.Units.SI.DynamicViscosity Mu_fg[Rows + 1, N] "Node boundary flue gas dynamic viscosity";
    //     // Heat capacities Cp
    //     Units.HeatCapacity Cp_water[Rows, N+1] "Node boundary water Cp";
    //     Units.HeatCapacity Cp_water_node[Rows, N] "Node average water Cp";
    //     Units.HeatCapacity Cp_fg[Rows + 1, N] "Node boundary flue gas Cp";
    //     // Thermal conductivity
    //     Modelica.Units.SI.ThermalConductivity k_water[Rows, N+1] "Node boundary water thermal conductivity";
    //     Modelica.Units.SI.ThermalConductivity k_water_node[Rows, N] "Node average water thermal conductivity";
    //     Modelica.Units.SI.ThermalConductivity k_fg[Rows + 1, N] "Node boundary flue gas thermal conductivity";

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
            // State
    //         state_water[i, j+1] = WaterSteamMedium.setState_phX(P_water, h_water[i, j+1], Xi_water);
    //         state_fg[i+1, j] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1, j], Xi_fg);
    //         // Temperature
    //         T_water[i, j+1] = WaterSteamMedium.temperature(state_water[i, j+1]);
    //         T_fg[i+1, j] = FlueGasesMedium.temperature(state_fg[i+1, j]);
    //         T_water_node[i, j] = 0.5*(T_water[i, j] + T_water[i, j+1]);
    //         T_fg_node[i, j] = 0.5*(T_fg[i, j] + T_fg[i+1, j]);
    //         // Density
    //         rho_water[i, j+1] = WaterSteamMedium.density(state_water[i, j+1]);
    //         rho_fg[i+1, j] = FlueGasesMedium.density(state_fg[i+1, j]);
    //         rho_water_node[i, j] = 0.5*(rho_water[i, j] + rho_water[i, j+1]);
    //         // Dynamic viscosity
    //         Mu_water[i, j+1] = WaterSteamMedium.dynamicViscosity(state_water[i, j+1]);
    //         Mu_water_node[i, j] = 0.5*(Mu_water[i, j] + Mu_water[i, j+1]);
    //         Mu_fg[i+1, j] = FlueGasesMedium.dynamicViscosity(state_fg[i+1, j]);
    //         // Specific heat capacities Cp
    //         Cp_water[i, j+1] = WaterSteamMedium.specificHeatCapacityCp(state_water[i, j+1]);
    //         Cp_water_node[i, j] = 0.5*(Cp_water[i, j] + Cp_water[i, j+1]);
    //         Cp_fg[i+1, j] = FlueGasesMedium.specificHeatCapacityCp(state_fg[i+1, j]);
    //         // Thermal conductivity
    //         k_water[i, j+1] = WaterSteamMedium.thermalConductivity(state_water[i, j+1]);
    //         k_water_node[i, j] = 0.5*(k_water[i, j] + k_water[i, j+1]);
    //         k_fg[i+1, j] = FlueGasesMedium.thermalConductivity(state_fg[i+1, j]);

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
  end HeatExchangers;

  package Tests

    model CounterCurrent_MonoPhasicHX_LCM_Test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.CounterCurrent_MonoPhasicHX_LCM HX(
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
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end CounterCurrent_MonoPhasicHX_LCM_Test;

    model CounterCurrent_MonoPhasicHX_2NodesConduction_Test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.CounterCurrent_MonoPhasicHX_2NodesConduction HX(
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
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end CounterCurrent_MonoPhasicHX_2NodesConduction_Test;

    model CounterCurrent_MonoPhasicHX_1DConduction_plate_Test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.CounterCurrent_MonoPhasicHX_1DConduction_plate HX(
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
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end CounterCurrent_MonoPhasicHX_1DConduction_plate_Test;

    model CounterCurrent_MonoPhasicHX_1DConduction_radial_Test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.CounterCurrent_MonoPhasicHX_1DConduction_radial HX(
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
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
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
      HeatExchangers.CrossCurrent_MonoPhasicHX_LCM_ConstantK crossCurrent_MonoPhasicHX_LCM_ConstantK annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
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
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
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
      HeatExchangers.CrossCurrent_MonoPhasicHX_LCM crossCurrent_MonoPhasicHX_2NodesConduction(
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
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
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
      HeatExchangers.CrossCurrent_MonoPhasicHX_2NodesConduction crossCurrent_MonoPhasicHX_LCM(
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
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
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
      HeatExchangers.CrossCurrent_MonoPhasicHX_1DConduction_radial crossCurrent_MonoPhasicHX_LCM(
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
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end CrossCurrent_MonoPhasicHX_1DConduction_Test;

    model CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_Test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK
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
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_Test;

    model CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage_Test
      import MetroscopeModelingLibrary.Utilities.Units;
      import MetroscopeModelingLibrary.Utilities.Units.Inputs;
      HeatExchangers.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage
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
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage_Test;
  end Tests;

  package Examples
    model HX_4_pass
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
            rotation=0,
            origin={-34,-40})));
      WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{10,-10},{-10,10}},
            rotation=180,
            origin={30,-40})));
      FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-44,-10},{-24,10}})));
      FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{20,-10},{40,10}})));
      Modelica.Blocks.Sources.Step step(height=100, startTime=100) annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
      Modelica.Blocks.Sources.Ramp ramp(
        height=-20,
        duration=60,
        startTime=300) annotation (Placement(transformation(extent={{20,20},{40,40}})));
      Multiple_Pass_HX.HX_4_pass hX_4_pass(
        N_tubes_row_dir_1=46,
        N_tubes_row_dir_2=46,
        N_tubes_row_dir_3=46,
        N_tubes_row_dir_4=46)              annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    equation
      hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
      hot_source.P_out = P_hot_source*1e5;
      hot_source.T_out = T_hot_source + 273.15 + ramp.y;
      hot_source.Q_out = - Q_hot_source + step.y;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source;


      connect(hot_source.C_out, hX_4_pass.fg_in) annotation (Line(points={{-29,0},{-10,0}}, color={95,95,95}));
      connect(hX_4_pass.fg_out, hot_sink.C_in) annotation (Line(points={{10,0},{25,0}}, color={95,95,95}));
      connect(cold_source.C_out, hX_4_pass.water_in) annotation (Line(points={{-29,-40},{-6,-40},{-6,-10}}, color={28,108,200}));
      connect(cold_sink.C_in, hX_4_pass.water_out) annotation (Line(points={{25,-40},{6.2,-40},{6.2,-10}}, color={28,108,200}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},{60,60}})),
                                                                     Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},{60,60}})),
        experiment(
          StopTime=500,
          __Dymola_NumberOfIntervals=1000,
          __Dymola_Algorithm="Dassl"));
    end HX_4_pass;
  end Examples;

  package Multiple_Pass_HX
    model HX_4_pass

      parameter Integer N_tubes_row_dir_1 = 40;
      parameter Integer N_tubes_row_dir_2 = 40;
      parameter Integer N_tubes_row_dir_3 = 40;
      parameter Integer N_tubes_row_dir_4 = 40;

      Integer N_tubes_row;
      Real Q_tot;


      HeatExchangers.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_1(
        D_out=0.0381,
        e=0.003048,
        L=18.29,                                                                            N_tubes_row = N_tubes_row_dir_1,
        T_wall_0=745.15,
        N=10)                                                                                                                annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
      HeatExchangers.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_2(
        D_out=0.0381,
        e=0.003048,
        L=18.29,                                                                            N_tubes_row = N_tubes_row_dir_2,
        T_wall_0=745.15,
        N=10)                                                                                                                annotation (Placement(transformation(extent={{-30,10},{-10,-10}})));
      HeatExchangers.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_3(
        D_out=0.0381,
        e=0.003048,
        L=18.29,                                                                            N_tubes_row = N_tubes_row_dir_3,
        T_wall_0=745.15,
        N=10)                                                                                                                annotation (Placement(transformation(extent={{10,-10},{30,10}})));
      HeatExchangers.CrossCurrent_1NodePerRow_MonoPhasicHX_LCM_ConstantK_WaterStorage Dir_4(
        D_out=0.0381,
        e=0.003048,
        L=18.29,                                                                            N_tubes_row = N_tubes_row_dir_4,
        T_wall_0=745.15,
        N=10)                                                                                                                annotation (Placement(transformation(extent={{50,10},{70,-10}})));
      FlueGases.Connectors.Inlet fg_in annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
      FlueGases.Connectors.Outlet fg_out annotation (Placement(transformation(extent={{90,-10},{110,10}})));
      WaterSteam.Connectors.Inlet water_in annotation (Placement(transformation(extent={{-70,-110},{-50,-90}})));
      WaterSteam.Connectors.Outlet water_out annotation (Placement(transformation(extent={{52,-110},{72,-90}})));
      FlueGases.Pipes.PressureCut PC_Dir_1 annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-50,-40})));
      FlueGases.Pipes.PressureCut PC_Dir_2 annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-10,-40})));
      FlueGases.Pipes.PressureCut PC_Dir_3 annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={28,-40})));
    equation

      N_tubes_row = N_tubes_row_dir_1 + N_tubes_row_dir_2 + N_tubes_row_dir_3 + N_tubes_row_dir_4;
      Q_tot = Dir_1.fg_side.Q + Dir_2.fg_side.Q + Dir_3.fg_side.Q + Dir_4.fg_side.Q;

      Dir_1.fg_side.Q = Q_tot*N_tubes_row_dir_1/N_tubes_row;
      Dir_2.fg_side.Q = Q_tot*N_tubes_row_dir_2/N_tubes_row;
      Dir_3.fg_side.Q = Q_tot*N_tubes_row_dir_3/N_tubes_row;


      connect(Dir_1.water_inlet, water_in) annotation (Line(points={{-60,-10},{-60,-100}}, color={28,108,200}));
      connect(Dir_4.water_outlet, water_out) annotation (Line(points={{60,-10},{60,-56},{60,-100},{62,-100}}, color={28,108,200}));
      connect(fg_in, Dir_1.fg_inlet) annotation (Line(points={{-100,0},{-64,0}}, color={95,95,95}));
      connect(Dir_4.fg_outlet, fg_out) annotation (Line(points={{64,0},{100,0}}, color={95,95,95}));
      connect(Dir_1.fg_outlet, PC_Dir_1.C_in) annotation (Line(points={{-56,0},{-50,0},{-50,-30}}, color={95,95,95}));
      connect(PC_Dir_1.C_out, fg_out) annotation (Line(points={{-50,-50},{-50,-60},{80,-60},{80,0},{100,0}}, color={95,95,95}));
      connect(Dir_2.fg_outlet, PC_Dir_2.C_in) annotation (Line(points={{-16,0},{-10,0},{-10,-30}}, color={95,95,95}));
      connect(PC_Dir_2.C_out, fg_out) annotation (Line(points={{-10,-50},{-10,-60},{80,-60},{80,0},{100,0}}, color={95,95,95}));
      connect(Dir_3.fg_outlet, PC_Dir_3.C_in) annotation (Line(points={{24,0},{28,0},{28,-30}}, color={95,95,95}));
      connect(PC_Dir_3.C_out, fg_out) annotation (Line(points={{28,-50},{28,-60},{80,-60},{80,0},{100,0}}, color={95,95,95}));
      connect(Dir_4.fg_inlet, Dir_1.fg_inlet) annotation (Line(points={{56,0},{50,0},{50,60},{-80,60},{-80,0},{-64,0}}, color={95,95,95}));
      connect(Dir_3.fg_inlet, Dir_1.fg_inlet) annotation (Line(points={{16,0},{10,0},{10,60},{-80,60},{-80,0},{-64,0}}, color={95,95,95}));
      connect(Dir_2.fg_inlet, Dir_1.fg_inlet) annotation (Line(points={{-24,0},{-30,0},{-30,60},{-80,60},{-80,0},{-64,0}}, color={95,95,95}));
      connect(Dir_1.water_outlet, Dir_2.water_inlet) annotation (Line(points={{-60,10},{-60,40},{-20,40},{-20,10}}, color={28,108,200}));
      connect(Dir_2.water_outlet, Dir_3.water_inlet) annotation (Line(points={{-20,-10},{-20,-20},{20,-20},{20,-10}}, color={28,108,200}));
      connect(Dir_3.water_outlet, Dir_4.water_inlet) annotation (Line(points={{20,10},{20,40},{60,40},{60,10}}, color={28,108,200}));
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
            Line(points={{-58,72}}, color={28,108,200}),
            Rectangle(
              extent={{-80,-80},{80,-86}},
              lineColor={28,108,200},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Line(
              points={{-70,0},{-70,-80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(
              points={{-60,0},{-60,-80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(
              points={{-50,0},{-50,-80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(
              points={{-30,0},{-30,80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(
              points={{-20,0},{-20,80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(
              points={{-10,0},{-10,80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(
              points={{10,0},{10,-80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(
              points={{20,0},{20,-80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(
              points={{30,0},{30,-80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(
              points={{50,0},{50,80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(
              points={{60,0},{60,80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(
              points={{70,0},{70,80}},
              color={28,108,200},
              arrow={Arrow.Filled,Arrow.None}),
            Line(points={{-70,0},{-70,80}}, color={28,108,200}),
            Line(points={{-60,0},{-60,80}}, color={28,108,200}),
            Line(points={{-50,0},{-50,80}}, color={28,108,200}),
            Line(points={{-10,-80},{-10,0}}, color={28,108,200}),
            Line(points={{-20,-80},{-20,0}}, color={28,108,200}),
            Line(points={{-30,-80},{-30,0}}, color={28,108,200}),
            Line(points={{70,-80},{70,0}}, color={28,108,200}),
            Line(points={{60,-80},{60,0}}, color={28,108,200}),
            Line(points={{50,-80},{50,0}}, color={28,108,200}),
            Line(points={{30,0},{30,80}}, color={28,108,200}),
            Line(points={{20,0},{20,80}}, color={28,108,200}),
            Line(points={{10,0},{10,80}}, color={28,108,200})}),     Diagram(coordinateSystem(preserveAspectRatio=false)));
    end HX_4_pass;
  end Multiple_Pass_HX;
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
