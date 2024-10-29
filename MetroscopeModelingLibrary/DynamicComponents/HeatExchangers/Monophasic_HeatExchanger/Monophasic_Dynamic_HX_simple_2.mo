within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.Monophasic_HeatExchanger;
model Monophasic_Dynamic_HX_simple_2
 import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;
  import CorrelationConstants =
         MetroscopeModelingLibrary.DynamicComponents.Correlations;

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  // Constants
    parameter Real pi = Constants.pi;

  parameter Boolean steady_state = false;

  // Geometry
  parameter Units.Area A_i = 680;
  parameter Units.Area A_o = 721;
  parameter Units.Area A_f = 2500;
  parameter Real eta_f = 0.8;
  parameter Integer N_r = 2;
  parameter Units.Length L = 18;
  parameter Integer N_tubes = 300;
  parameter Units.Length D_out = 0.03 "Pipe outer diameter";
  parameter Units.Length e = 0.003 "Pipe wall thickness";
  parameter Units.Length D_in = D_out - 2*e "Pipe inner diameter";
  parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
  parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";
  parameter Units.Volume V_water = N_tubes*0.25*pi*D_in^2*L "Total volume of water inside the tubes";
  parameter Units.HeatCapacity Cp_fg = 1068;
  parameter Units.HeatCapacity Cp_water = 4333;

  // Heat transfer
  parameter Units.HeatExchangeCoefficient K_conv_water = 2500;
  parameter Units.HeatExchangeCoefficient K_conv_fg = 70;
  parameter Units.ThermalConductivity K_cond_wall = 27 "Wall thermal conductivity";

  // Discretization
  parameter Integer N = 10;
  parameter Units.Length dz = L/N;
  parameter Units.Mass dM_wall = M_wall/N/N_r "Tube mass of a single node";
  parameter Units.Area dA_i = A_i/N/N_r "Water side heat exchange surface of a single node";
  parameter Units.Area dA_o = A_o/N/N_r "Flue gas side heat exchange surface of a single node";
  parameter Units.Area dA_fg_fin = A_f/N/N_r;
  parameter Units.Volume dV_water = V_water/N/N_r "WAter volume of 1 element";

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

  // ------ Fluids properties ------
    // State
    //FlueGasesMedium.ThermodynamicState state_fg[N_r + 1, N] "Flue gas side node boundary states";
    // Enthalpy
    Units.SpecificEnthalpy h_water[N_r, N+1](each start=h_water_out_0) "Water specific enthalpy";
    Units.SpecificEnthalpy h_water_node[N_r, N](each start=h_water_out_0) "Water specific enthalpy";
    Units.SpecificEnthalpy h_fg[N_r + 1, N] "Flue gas specific enthalpy";
    // Mass flow rate
    Units.PositiveMassFlowRate Q_water[N_r, N+1] "Node boundary water mass flow rate";
    Units.PositiveMassFlowRate Q_fg "Node boundary fg mass flow rate";
    // Pressure
    Units.Pressure P_water(start=P_water_0) "Water Pressure";
    Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
    // Mass fraction
    Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
    Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
    // Temperature
    Units.Temperature T_water[N_r, N+1] "Node boundary water temperature";
    Units.Temperature T_water_node[N_r, N] "Node average water temperature";
    Units.Temperature T_fg[N_r + 1, N] "Node boundary flue gas temperature";
  // ------ Conduction variables ------
    Units.Temperature T_wall_water[N_r, N] "Wall temperature from the water side";
    Units.Temperature T_wall[N_r, N] "Node wall average temperature";
    Units.Temperature T_wall_fg[N_r, N] "Wall temperature from the flue gas side";

    Units.Power dW_water[N_r, N] "Node water heat exchange";
    Units.Power dW_fg[N_r, N] "Node flue gas heat exchange";

  // Parameters of interest
    Units.Temperature T_water_in "Water inlet temperature";
    Units.Temperature T_water_out "Water outlet temperature";
    //Units.Temperature T_fg_in "Flue gas inlet temperature";
    //Units.Temperature T_fg_out "Flue gas outlet temperature";
    Units.Temperature T_water_avg "Water overall average temperature";
    Units.Temperature T_wall_avg "Wall overall average temperature";

  // Density derivatives
  Real drhodh_water[N_r, N+1];
  Real drhodp_water[N_r, N+1];

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
    for i in 1:N_r loop
      h_water[i, 1] = water_side.h_in;
      T_water[i, 1] = h_water[i, 1]/Cp_water;
      Q_water[i, 1] = water_side.Q/N_r;
      drhodh_water[i, 1] = 0.001;
      drhodp_water[i, 1] = 0.001;
    end for;

    for j in 1:N loop
     h_fg[1,  j] = fg_side.h_in;
     T_fg[1,  j] = h_fg[1,  j]/Cp_fg;
    end for;

    // Pressure
    P_water = water_side.P_in;
    P_fg = fg_side.P_in;
    // Mass flow rate
    Q_fg = fg_side.Q;
    // Mass Fraction
    Xi_water = water_side.Xi;
    Xi_fg = fg_side.Xi;

  // ------ Parameters of interest ------
    // IN/OUT temperatures
    T_water_in = water_side.T_in;
    T_water_out = water_side.T_out;
    //T_fg_in = fg_side.T_in;
    //T_fg_out = fg_side.T_out;
    // Average Temperatures
    T_water_avg = sum(T_water_node)/(N_r*N);
    T_wall_avg =  sum(T_wall)/(N_r*N);

  // ------ Discretization computation loop ------
    for i in 1:N_r loop
        for j in 1:N loop
      // Fluids Properties
        // State
        // Temperature
        T_water[i, j+1] = h_water[i, j+1]/Cp_water;
        T_fg[i+1, j] = h_fg[i+1, j]/Cp_fg;
        T_water_node[i, j] = 0.5*(T_water[i, j] + T_water[i, j+1]);
        // Mean enthalpy
        h_water_node[i, j] = 0.5*(h_water[i, j] + h_water[i, j+1]);
        // Density derivatives
        drhodh_water[i, j+1] = 0.001;
        drhodp_water[i, j+1] = 0.001;

      // Mass balance
        Q_water[i, j+1] = Q_water[i, j] + dV_water*drhodh_water[i, j]*der(h_water[i, j+1]);

      // Conduction heat transfer
        dW_water[i, j] = 2*pi*K_cond_wall*dz*N_tubes*(T_wall[i, j] - T_wall_water[i, j])/(Modelica.Math.log(1 + e/D_in));
        dW_fg[i, j] = 2*pi*K_cond_wall*dz*N_tubes*(T_wall[i, j] - T_wall_fg[i, j])/(Modelica.Math.log(1 + e/(e + D_in)));

      // Node energy balance
        // Water side
        dW_water[i, j] = Q_water[i, j]*(h_water[i, j+1] - h_water[i, j]);
        // Flue gas side
        dW_fg[i, j] = Q_fg*(h_fg[i+1, j] - h_fg[i, j]);
        // Global with wall storage
        dW_water[i, j] + dW_fg[i, j] + dM_wall*Cp_wall*der(T_wall[i, j]) = 0;

      // Convection heat transfer equations
        // Water side
        dW_water[i, j] = K_conv_water*dA_i*(T_wall[i, j] - T_water_node[i, j]);
        // Flue gas side
        dW_fg[i, j] = K_conv_fg*(T_wall[i, j] - T_fg[i, j])*(dA_o + eta_f*dA_fg_fin);

    end for;
    end for;

initial equation
      for i in 1:N_r loop
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
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
                                                                graphics={
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
        Line(points={{-40,0},{40,0}},    color={0,0,0}),
        Text(
          extent={{-49,14},{49,-14}},
          textColor={28,108,200},
          textString="%name",
          origin={-69,50},
          rotation=90),
        Text(
          extent={{48,-102},{188,-160}},
          textColor={28,108,200},
          textString="T_in = " + DynamicSelect("?", String(T_water_in-273.15))),
        Text(
          extent={{28,176},{168,118}},
          textColor={28,108,200},
          textString="T_out = " + DynamicSelect("?", String(T_water_out-273.15)))}),                           Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
end Monophasic_Dynamic_HX_simple_2;
