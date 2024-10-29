within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.TwoPhaseHX;
model Evaporator_1
 import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;
  import CorrelationConstants =
         MetroscopeModelingLibrary.DynamicComponents.Correlations;

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  // Constants
    parameter Real pi = Constants.pi;
    parameter Real Mmol = 18.015 "Water molar mass";

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

    // Saturation properties
    Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat "Saturation state for properties calculation";
    WaterSteamMedium.ThermodynamicState state_s "Steam state";
    WaterSteamMedium.ThermodynamicState state_l "Liquid state";
    Units.SpecificEnthalpy h_l "Sat. liquid specific enthalpy";
    Units.SpecificEnthalpy h_s "Sat. vapor specific enthalpy";
    Units.Density rho_l "Sat. liquid density";
    Units.Density rho_s "Sat. vaport density";
    Units.Temperature T_sat "Saturation temperatuere";
    Modelica.Units.SI.DynamicViscosity Mu_l "Sat. liquid dynamic viscosity";
    Modelica.Units.SI.DynamicViscosity Mu_s "Sat. vapor dynamic viscosity";
    Units.HeatCapacity Cp_l "Sat. liquid heat capacity";
    Units.HeatCapacity Cp_s "Sat. vapor heat capacity";
    Modelica.Units.SI.ThermalConductivity k_l "Sat. liquid thermal conductivity";
    Modelica.Units.SI.ThermalConductivity k_s "Sat. vapor thermal conductivity";
    // State
    WaterSteamMedium.ThermodynamicState state_water[Rows, N+1] "Water side node boundary state";
    FlueGasesMedium.ThermodynamicState state_fg[Rows + 1, N] "Flue gas side node boundary states";
    // Enthalpy
    Units.SpecificEnthalpy h_water[Rows, N+1](each start=1.4e6) "Water specific enthalpy";
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
    //Units.Temperature T_water[Rows, N+1] "Node boundary water temperature";
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
    // Steam quality
    Real x[Rows, N+1](each start=0.5);
    Real x_node[Rows, N](each start=0.5);

  // ------ Conduction variables ------
    Units.Temperature T_wall[Rows, N](each start=400+273.15) "Node wall average temperature";

  // ------ Tubes configuration parameters ------
    Units.Velocity U_fg_face "Flue gas face velocity";
    Units.Velocity U_fg_max "Flue gas maximum velocity";
    Real Re_fg_max "Flue gas maximum Reynold's number";
    Real Pr_fg "Flue gas overall average Prandtl number";
    FlueGasesMedium.ThermodynamicState state_fg_s "Flue gas state at surface temperature";
    Real Pr_fg_s "Flue gas overall average Prandtl number at surface temperature";
    Real Nu_fg_avg "Flue gass overall average Nusselt number";
    Units.HeatExchangeCoefficient K_conv_fg(each start=76) "Flue gase convection heat transfer coefficient"; //  = 76.83

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
    Units.HeatExchangeCoefficient K_conv_2phase[Rows, N](each start=20000);
    // Discretized heat transfer power
    Units.Power dW_water[Rows, N](each start = 330e6/N) "Node water heat exchange";
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

    // Two-phase heat transfer coefficient correlation variables
    Real E[Rows, N](each start = 4);
    Real S[Rows, N](each start = 0.0751);
    Units.HeatExchangeCoefficient K_bo[Rows, N](each start = 45509) "Boiling heat transfer coefficient";
    Real P_red;
    Real Bo[Rows, N](each start = 0.0001878);
    Real X_tt[Rows, N](each start = 3);
    Real G;

  WaterSteam.Connectors.Outlet water_outlet annotation (Placement(transformation(extent={{-50,90},{-30,110}}), iconTransformation(extent={{-50,90},{-30,110}})));
  WaterSteam.Connectors.Inlet water_inlet annotation (Placement(transformation(extent={{30,90},{50,110}}), iconTransformation(extent={{30,90},{50,110}})));
  FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-90,-10},{-70,10}}), iconTransformation(extent={{-90,-10},{-70,10}})));
  FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{70,-10},{90,10}}), iconTransformation(extent={{70,-10},{90,10}})));
  WaterSteam.BaseClasses.IsoPFlowModel water_side annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,40})));
  FlueGases.BaseClasses.IsoPFlowModel fg_side annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation

    // ------ Boundaries ------

    // Outlet
    water_side.W = sum(dW_water);
    fg_side.W = sum(dW_fg);

    // Set saturation state
    sat.psat =  P_water;
    sat.Tsat =  Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_water);

    // Saturation properties
    state_l = WaterSteamMedium.setBubbleState(sat);
    state_s = WaterSteamMedium.setDewState(sat);
    h_l = WaterSteamMedium.specificEnthalpy(state_l);
    h_s = WaterSteamMedium.specificEnthalpy(state_s);
    rho_l = WaterSteamMedium.density(state_l);
    rho_s = WaterSteamMedium.density(state_s);
    T_sat = WaterSteamMedium.temperature(state_l);
    Mu_l = WaterSteamMedium.dynamicViscosity(state_l);
    Mu_s = WaterSteamMedium.dynamicViscosity(state_s);
    Cp_l = WaterSteamMedium.specificHeatCapacityCp(state_l);
    Cp_s = WaterSteamMedium.specificHeatCapacityCp(state_s);
    k_l = WaterSteamMedium.thermalConductivity(state_l);
    k_s = WaterSteamMedium.thermalConductivity(state_s);

    // Inlet
    for i in 1:Rows loop
      h_water[i, 1] = water_side.h_in;
      state_water[i, 1] = state_l;
      rho_water[i, 1] = rho_l;
      Mu_water[i, 1] = Mu_l;
      Cp_water[i, 1] = Cp_l;
      k_water[i, 1] = k_l;
      x[i,  1] = (h_water[i, 1] - h_l)/(h_s - h_l);
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

    G = Q_water/Ac_water;
    P_red = P_water/(220.64e5);

  // ------ Discretization computation loop ------
    for i in 1:Rows loop
        for j in 1:N loop
      // Fluids Properties
        // State
        state_water[i, j+1] = WaterSteamMedium.setState_phX(P_water, h_water[i, j+1], Xi_water);
        state_fg[i+1, j] = FlueGasesMedium.setState_phX(P_fg, h_fg[i+1, j], Xi_fg);
        // Temperature
        //T_water[i, j+1] = T_sat;
        T_fg[i+1, j] = FlueGasesMedium.temperature(state_fg[i+1, j]);
        T_water_node[i, j] = T_sat;
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
        //Steam quality
        if ((h_water[i, j+1] - h_l)/(h_s - h_l) < 1) then
          x[i, j+1] = (h_water[i, j+1] - h_l)/(h_s - h_l);
        else
          x[i, j+1] = 1;
        end if;

        if 0.5*(x[i, j] + x [i, j+1]) < 1 then
          x_node[i, j] = 0.5*(x[i, j] + x [i, j+1]);
        else
          x_node[i, j] = 0.99;
        end if;

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

        // Two phase convection heat transfer coefficient
        K_conv_2phase[i, j] = CorrelationConstants.Kandlikar_2phase(P_water, Q_water, dW_water[i, j], K_conv_water[i, j], x_node[i, j], Ac_water, dA_water);
        //K_conv_2phase[i, j] = K_conv_water[i, j];
        //K_conv_2phase[i, j] = 10000;
        E[i, j] = 1 + 24000*Bo[i, j]^1.16 + 1.37*(1/X_tt[i, j])^0.86;
        S[i, j] = 1/(1 + 1.15e-6*E[i, j]^2*Re_water[i, j]^1.17);
        Bo[i, j] = abs(dW_water[i, j])/(dA_water*(h_s - h_l)*G);

        K_bo[i, j] = 55*P_red^0.12*(-Modelica.Math.log10(P_red))^(-0.55)*Mmol^(-0.5)*(abs(dW_water[i, j]/dA_water))^0.67;

        X_tt[i, j] = 1;
//         if (x_node[i, j] < 0.01 or x_node[i, j] > 0.85) then
//           X_tt[i, j] = 1;
//           K_conv_2phase[i, j] = K_conv_water[i, j];
//         else
//           X_tt[i, j] = ((1 - x_node[i, j])/x_node[i, j])^0.9*(rho_s/rho_l)^0.5*(Mu_l/Mu_s)^0.1;
//           K_conv_2phase[i, j] = E[i, j]*K_conv_water[i, j] + S[i, j]*K_bo[i, j];
//         end if;

      // Convection heat transfer equations
        // Water side
        dW_water[i, j] = K_conv_2phase[i, j]*dA_water*(T_wall[i, j] - T_water_node[i, j]);
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
  connect(water_side.C_out,water_outlet)  annotation (Line(points={{40,30},{40,-40},{-40,-40},{-40,100}}, color={28,108,200}));
  connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-10,0},{-80,0}}, color={95,95,95}));
  connect(fg_side.C_out, fg_outlet) annotation (Line(points={{10,0},{80,0}}, color={95,95,95}));
  connect(water_side.C_in, water_inlet) annotation (Line(points={{40,50},{40,100}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-80,100},{80,-100}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(
          points={{8,-80}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Ellipse(
          extent={{-50,70},{-30,50}},
          lineColor={0,0,0},
          lineThickness=1),
        Ellipse(
          extent={{-50,-70},{-30,-90}},
          lineColor={0,0,0},
          lineThickness=1),
        Line(
          points={{-40,50},{-40,-70}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier,
          pattern=LinePattern.Dash),
        Line(
          points={{-34,52},{-30,10},{-30,-30},{-34,-72}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier,
          pattern=LinePattern.Dash),
        Line(
          points={{-46,52},{-50,10},{-50,-30},{-46,-72}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier,
          pattern=LinePattern.Dash),
        Line(
          points={{-30,58},{-20,10},{-20,-30},{-30,-78}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier,
          pattern=LinePattern.Dash),
        Line(
          points={{-50,58},{-60,10},{-60,-30},{-50,-78}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier,
          pattern=LinePattern.Dash),
        Line(
          points={{40,90},{40,-80},{-30,-80}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{-40,100},{-40,70}},
          color={28,108,200},
          pattern=LinePattern.Dash,
          thickness=1)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Evaporator_1;