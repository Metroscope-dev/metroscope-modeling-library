within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.One_pass_HX;
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
