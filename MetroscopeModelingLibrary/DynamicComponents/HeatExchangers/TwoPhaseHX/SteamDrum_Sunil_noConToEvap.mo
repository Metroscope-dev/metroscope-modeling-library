within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.TwoPhaseHX;
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
