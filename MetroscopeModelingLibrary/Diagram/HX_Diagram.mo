within MetroscopeModelingLibrary.Diagram;
model HX_Diagram "Added more info on fins and used ESCOA correlation"
 import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;
  import CorrelationConstants = MetroscopeModelingLibrary.DynamicComponents.Correlations;

  parameter Boolean steady_state = false;

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

    Modelica.Units.SI.ThermalConductance UA_fg(start=171696.64);
    Modelica.Units.SI.ThermalConductance UA_water(start=1476005.2);
    Modelica.Units.SI.ThermalConductance UA(start=155746);
    parameter Real r_UA = 39.677;
    parameter Units.Area A = 3500;
    Units.HeatExchangeCoefficient U "Overall heat transfer coefficient";

    // Wall
    parameter Units.Mass M_wall = 25379 "Tubes + fins total mass";
    parameter Units.HeatCapacity Cp_wall = 420 "Tubes specific heat capacity";

    // Discretization
    parameter Integer N = 1;

    // Temperatures
    Units.Temperature T_water_in "Water inlet temperature";
    Units.Temperature T_water_out(start=T_water_out_0) "Water outlet temperature";
    Units.Temperature T_water[N + 1] "Water temperature";
    Units.Temperature T_water_node[N] "Mean water temperature";
    Units.Temperature T_fg_in "Flue gas inlet temperature";
    Units.Temperature T_fg_out(start=T_fg_out_0) "Flue gas outlet temperature";
    Units.Temperature T_fg[2, N] "Flue gas temperature";
    Units.Temperature T_fg_node[N] "Mean flue gas temperature";
    Units.Temperature T_wall[N](each start=T_wall_0) "Tubes wall temperature";
    // Enthalpy
    Units.SpecificEnthalpy h_water[N+1] "Water specific enthalpy";
    Units.SpecificEnthalpy h_fg[2, N] "Flue gas specific enthalpy";
    // Pressures
    Units.Pressure P_water(start=P_water_0) "Water Pressure";
    Units.Pressure P_fg(start=P_fg_0) "Flue gas Pressure";
    // Mass flow rates
    Units.PositiveMassFlowRate Q_water(start=Q_water_0) "Water Mass flow rate";
    Units.PositiveMassFlowRate Q_fg(start=Q_fg_0) "Flue gas Mass flow rate";
    // Heat exchanged
    Units.Power dW_water[N];
    Units.Power dW_fg[N];
    // States
    WaterSteamMedium.ThermodynamicState state_water[N+1] "Water side node boundary state";
    FlueGasesMedium.ThermodynamicState state_fg[2, N] "Flue gas side node boundary states";
    // Mass fraction
    Units.MassFraction Xi_water[WaterSteamMedium.nXi] "Species mass fraction";
    Units.MassFraction Xi_fg[FlueGasesMedium.nXi] "Species mass fraction";
    Real DT_node[N];
    Real pinch;

    // Discretization

    //Units.Temperature DT_LMTD_water[N](each start=DT_LMTD_water_0);
    //Units.Temperature DT_LMTD_fg[N](each start=DT_LMTD_fg_0);

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
  UA = U*A;

  // ------ Boundaries ------
    // Enthalpy
    h_water[1] = water_side.h_in;
    h_water[N+1] = water_side.h_out;
    fg_side.W = sum(dW_fg);
    // Pressure
    P_water = water_side.P_in;
    P_fg = fg_side.P_in;
    // Mass flow rate
    Q_water = water_side.Q;
    Q_fg = fg_side.Q;
    // Mass Fraction
    Xi_fg = fg_side.Xi;
    Xi_water = water_side.Xi;
    // Water properties at the first node
    state_water[1] = WaterSteamMedium.setState_phX(P_water, h_water[1], Xi_water);
    T_water[1] = WaterSteamMedium.temperature(state_water[1]);
    // Quantities of interest
    T_water_in = water_side.T_in;
    T_water_out = water_side.T_out;
    T_fg_in = fg_side.T_in;
    T_fg_out = fg_side.T_out;

  for i in 1:N loop
     h_fg[1, i] = fg_side.h_in;
     T_fg[1, i] = FlueGasesMedium.temperature(state_fg[1, i]);
     state_fg[1, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[1, i], Xi_fg);
  end for;

  for i in 1:N loop
    // State
    state_water[i+1] = WaterSteamMedium.setState_phX(P_water, h_water[i+1], Xi_water);
    state_fg[2, i] = FlueGasesMedium.setState_phX(P_fg, h_fg[2, i], Xi_fg);
    // Temperature
    T_water[i+1] = WaterSteamMedium.temperature(state_water[i+1]);
    T_fg[2, i] = FlueGasesMedium.temperature(state_fg[2, i]);

    // Mean temperatures
    T_water_node[i] = 0.5*(T_water[i] + T_water[i+1]);
    T_fg_node[i] = 0.5*(T_fg[1, i] + T_fg[2, i]);
    DT_node[i] =  T_fg_node[i] - T_water_node[i];
    assert(T_water_node[i] < T_fg_node[i], "Negative pinch is reached", AssertionLevel.warning);
    // DT LMTD
    //DT_LMTD_water[i] = ((T_wall[i] - T_water[i]) - (T_wall[i] - T_water[i+1]))/log((T_wall[i] - T_water[i])/(T_wall[i] - T_water[i+1]));
    //DT_LMTD_fg[i] = ((T_fg[1, i] - T_wall[i]) - (T_fg[2, i] - T_wall[i]))/log((T_fg[1, i] - T_wall[i])/(T_fg[2, i] - T_wall[i]));
    // Heat transfer equations
    dW_water[i] = UA_water/N*(T_wall[i] - T_water_node[i]);
    dW_fg[i] = UA_fg/N*(T_wall[i] - T_fg_node[i]);
    dW_water[i] = Q_water*(h_water[i+1] - h_water[i]);
    dW_fg[i] = Q_fg/N*(h_fg[2, i] - h_fg[1, i]);
    // Global energy balance
    if steady_state then
      dW_water[i] + dW_fg[i] = 0;
      else
      dW_water[i] + dW_fg[i] + M_wall*Cp_wall*der(T_wall[i]) = 0;
    end if;

  end for;

  pinch = min(DT_node);

initial equation
  if not steady_state then
    for i in 1:N loop
       der(T_wall[i]) = 0;
    end for;
  end if;

equation
  connect(water_side.C_in, water_inlet) annotation (Line(points={{-1.83187e-15,10},{-1.83187e-15,-6},{0,-6},{0,-100}},
                                                                                   color={28,108,200}));
  connect(water_side.C_out, water_outlet) annotation (Line(points={{1.77636e-15,30},{0,30},{0,100}},
                                                                                       color={28,108,200}));
  connect(fg_side.C_out, fg_outlet) annotation (Line(points={{-10,-40},{40,-40},{40,0}},
                                                                                 color={95,95,95}));
  connect(fg_side.C_in, fg_inlet) annotation (Line(points={{-30,-40},{-40,-40},{-40,0}},
                                                                                 color={95,95,95}));
  annotation (Icon(coordinateSystem(extent={{-20,-20},{20,20}}),graphics={
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
        Line(points={{-40,0},{40,0}},    color={0,0,0})}), Diagram(coordinateSystem(extent={{-20,-20},{20,20}})));
end HX_Diagram;
