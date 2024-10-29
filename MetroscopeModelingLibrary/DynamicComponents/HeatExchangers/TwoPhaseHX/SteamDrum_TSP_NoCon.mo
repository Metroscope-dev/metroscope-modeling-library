within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.TwoPhaseHX;
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
