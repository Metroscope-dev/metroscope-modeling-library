within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.TwoPhaseHX;
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
