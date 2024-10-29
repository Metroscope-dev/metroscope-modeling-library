within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Astom_SimplifiedGeometry_5_debug
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
    Units.Area A_dc  "Downcomers cross sectional surface"; // 0.371
    parameter Units.Area A_d = 20 "Drum wet area at normal operating level";
    parameter Modelica.Units.SI.Volume V_dc = 11 "Downcomers volume";
    parameter Real k_friction = 25 "Friction coefficient in the downcomers"; // 25
    parameter Real beta = 0.3 "Constant";
    parameter Modelica.Units.SI.Volume V_t = V_D + V_r + V_dc;
    Modelica.Units.SI.Volume V_0_sd "Hypothetical volume of bubbles";
    //Modelica.Units.SI.Volume V_0_sd_calc "Calculated V_sd_0";
    parameter Real T_d = 12 "Steam residence time in the drum";

  // Initialization parameters
    parameter Units.Pressure p_0 = 85e5;
    parameter Modelica.Units.SI.Volume V_wt_0 = 57.1;

    parameter Real K_tuning = 1;

    parameter Units.Height l_0 = 0;
    parameter Units.Height l_w_0 = 0;
    parameter Units.Height l_s_0 = 0;

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
    Units.PositiveMassFlowRate Q_ct "Total condensation mass flow rate";
    Units.PositiveMassFlowRate Q_sd_calc;
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
    Real drho_w;
    Real drho_s;
    Real dhdp_w;
    Real dhdp_s;
    Real dTp;
    // Water surface tension
    Modelica.Units.SI.SurfaceTension sigma_s;
    //Interface Area
    Units.Area A_ma;
    // Escape velocity
    Units.Velocity u_s;

  // Evaporation heat
  Inputs.InputPower W_evap "Heat rate to the risers";
  // Drum water level
  Units.Height l "Water level";
  Units.Height l_w "Water level";
  Units.Height l_s "Water level";

  Real a, b;

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
  sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(p));
  // Densities
  rho_s = WaterSteamMedium.density(state_s);
  rho_w = WaterSteamMedium.density(state_w);
  //rho_b = WaterSteamMedium.density(state_s);

  // Enthalpies
  h_s = WaterSteamMedium.specificEnthalpy(state_s);
  h_w = WaterSteamMedium.specificEnthalpy(state_w);
  h_c = h_s - h_w; //*

//   sigma_s = WaterSteamMedium.surfaceTension(sat);
   // Escape velocity
   u_s = 1.41*((g*sigma_s*(rho_w - rho_s))/rho_w^2)^(1/4); // Verified eq 7

    // Masses
    //M_s = rho_s*V_st;
    //M_w = rho_w*V_wt;

    // Volume
    V_t = V_st + V_wt; //*
    V_wd = V_wt - V_dc - (1 - x_vr_mean)*V_r; //*

    // Steam mean volume fraction in risers
    x_vr_mean = rho_w/(rho_w - rho_s)*(1 - rho_s/((rho_w - rho_s)*x_r)*log(1 + (rho_w - rho_s)*x_r/rho_s)); //*

    // Global mass balance*
    Q_f - Q_s = V_st*(drho_s) + rho_s*der(V_st)
              + V_wt*(drho_w) + rho_w*der(V_wt);

    // Energy Balance*
    W_evap + Q_f*h_f - Q_s*h_s = h_s*V_st*drho_s + rho_s*V_st*der(h_s) + rho_s*h_s*der(V_st)
                               + h_w*V_wt*drho_w + rho_w*V_wt*der(h_w) + rho_w*h_w*der(V_wt)
                               - V_t*der(p)
                               + M_t*Cp*der(T_wall);

    // Mass balance for riser section*
    Q_dc - Q_r = x_vr_mean*V_r*drho_s + rho_s*V_r*der(x_vr_mean)
               + V_r*drho_w
               - x_vr_mean*V_r*drho_w - rho_w*V_r*der(x_vr_mean);

    // Energy balance for the riser section*
    W_evap + Q_dc*h_w - (x_r*h_c + h_w)*Q_r = h_s*x_vr_mean*V_r*drho_s + rho_s*h_s*V_r*der(x_vr_mean) + rho_s*x_vr_mean*V_r*der(h_s)
                                            + h_w*V_r*drho_w + rho_w*V_r*der(h_w)
                                            - h_w*x_vr_mean*V_r*drho_w - rho_w*h_w*V_r*der(x_vr_mean) - rho_w*x_vr_mean*V_r*der(h_w)
                                            - V_r*der(p)
                                            + M_r*Cp*der(T_wall);

    // Mass balance for the steam under the liquid level
    x_r*Q_r - Q_sd - Q_cd = drho_s*V_sd + rho_s*der(V_sd); //*
    Q_cd = (h_w - h_f)/h_c*Q_f + 1/h_c*(rho_s*V_sd*der(h_s) + rho_w*V_wd*der(h_w) - (V_sd + V_wd)*der(p) + M_d*Cp*der(T_wall));//*

    // Mass flow rate on steam through the surface*
    Q_sd = rho_s/T_d*(V_sd - V_0_sd) + x_r*Q_dc + x_r*beta*(Q_dc - Q_r);
    Q_sd_calc = K_tuning*A_ma*u_s*rho_s;
    //Q_sd_calc = Q_sd;

    // Total condensation mass flow rate
    Q_ct = (h_w - h_f)/h_c*Q_f + 1/h_c*(rho_s*V_st*der(h_s) + rho_w*V_wt*der(h_w) - V_t*der(p) + M_t*Cp*der(T_wall)); //*
    a = (h_w - h_f)/h_c*Q_f;
    b = 1/h_c*(rho_s*V_st*der(h_s) + rho_w*V_wt*der(h_w) - V_t*der(p) + M_t*Cp*der(T_wall));
    // Mass flow rate in the downcomers*
    0.5*k_friction*Q_dc^2 = rho_w*A_dc*(rho_w - rho_s)*g*x_vr_mean*V_r;

    // Level equation*
    l = (V_wd + V_sd)/A_d - l_0;
    l_w = V_wd/A_d - l_w_0;
    l_s = V_sd/A_d - l_s_0;

//     // Derrivatives
//     ddph_w = WaterSteamMedium.density_derp_h(state_w);
//     ddph_s = WaterSteamMedium.density_derp_h(state_s);
     ddhp_w = WaterSteamMedium.density_derh_p(state_w);
     ddhp_s = WaterSteamMedium.density_derh_p(state_s);

    // Derrivatives
    ddph_w = Modelica.Media.Water.WaterIF97_base.dBubbleDensity_dPressure(sat);
    ddph_s = Modelica.Media.Water.WaterIF97_base.dDewDensity_dPressure(sat);
    dhdp_w = WaterSteamMedium.dBubbleEnthalpy_dPressure(sat);
    dhdp_s = WaterSteamMedium.dDewEnthalpy_dPressure(sat);
    dTp = Modelica.Media.Water.WaterIF97_base.saturationTemperature_derp(p);

    drho_w = ddph_w*der(p);
    drho_s = ddph_s*der(p);

    //ddhp_w = 0;
    //ddhp_s = 0;

    // Calculated V_sd_0_calc
    //     V_0_sd = x_r*Q_r*T_d/rho_s;
    //V_0_sd = Q_sd*T_d/rho_s;

initial equation
  der(x_r) = 0;
  der(V_sd) = 0;
  der(p) = 0;

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
end SteamDrum_Astom_SimplifiedGeometry_5_debug;
