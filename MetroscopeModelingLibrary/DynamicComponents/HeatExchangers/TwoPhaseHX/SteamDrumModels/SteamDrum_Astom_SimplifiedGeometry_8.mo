within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Astom_SimplifiedGeometry_8

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;
  import CorrelationConstants =
         MetroscopeModelingLibrary.DynamicComponents.Correlations;

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  replaceable package Medium = Modelica.Media.Water.StandardWater constrainedby
    Modelica.Media.Interfaces.PartialTwoPhaseMedium "Medium model";

  // Constants
    parameter Real pi = Constants.pi;
    parameter Real g = Constants.g;

  // Parameters
    // Volumes
    parameter Modelica.Units.SI.Volume V_D = 40 "Drum volume";
    parameter Modelica.Units.SI.Volume V_r = 37 "Riser volume";
    parameter Modelica.Units.SI.Volume V_dc = 11 "Downcomers volume";
    parameter Modelica.Units.SI.Volume V_t = V_D + V_r + V_dc;

    // Masses
    parameter Units.Mass M_d = 140000 "Drum mass";
    parameter Units.Mass M_r = 160000 "Riser mass";
    parameter Units.Mass M_t = 300000 "Total mass of the system";
    // Surfaces
    Units.Area A_dc  "Downcomers cross sectional surface";
    parameter Units.Area A_d = 20 "Drum wet area at normal operating level";
    // Metal heat capacity
    parameter Units.HeatCapacity Cp = 550 "Heat capacity of the metal";
    // Empirical relation of Q_sd
    Modelica.Units.SI.Volume V_0_sd "Hypothetical volume of bubbles";
    parameter Real beta = 0.3 "Constant";
    parameter Real T_d = 12 "Steam residence time in the drum";
    // Friction coefficient
    parameter Real k = 25 "Friction coefficient in the downcomers";
    // Levels at nominal operating conditions
    parameter Units.Height l_0 = 0;
    parameter Units.Height l_w_0 = 0;
    parameter Units.Height l_s_0 = 0;

  // Initialization parameters
    parameter Units.Pressure p_0 = 85e5;
    parameter Modelica.Units.SI.Volume V_wt_0 = 57.1;

  // Variables
    // Drum pressure
    Units.Pressure p(start=p_0) "Drum saturation pressure";
    // Mass flow rates
    Units.PositiveMassFlowRate Q_f "Feed water mass flow rate";
    Units.PositiveMassFlowRate Q_s "Steam mass flow rate";
    Units.PositiveMassFlowRate Q_dc "Downcomers mass flow rate";
    Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
    Units.PositiveMassFlowRate Q_sd "Steam mass flow rate across the surface";
    Units.PositiveMassFlowRate Q_cd "Drum condensation mass flow rate";
    Units.PositiveMassFlowRate Q_ct "Total condensation mass flow rate";
    // Volumes
    Modelica.Units.SI.Volume V_st "Total steam volume";
    Modelica.Units.SI.Volume V_sd(start=4.8) "Drum steam volume";
    Modelica.Units.SI.Volume V_wt(start=V_wt_0, fixed = true) "Total water volume";
    Modelica.Units.SI.Volume V_wd "Drum water volume";
    // Enthalpies
    Units.SpecificEnthalpy h_s "Steam enthalpy";
    Units.SpecificEnthalpy h_w "Liquid enthalpy";
    Units.SpecificEnthalpy h_f "Feedwater enthalpy";
    Units.SpecificEnthalpy h_c "Condensation enthalpy";
    // Densities
    Units.Density rho_s "Steam density";
    Units.Density rho_w "Liquid density";
    // Saturation temperature
    Units.Temperature T_s "Metal wall temperature";
    // Set saturation state
    Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat "Saturation state for properties calculation";
    // Riser mass and volume fractions
    Real x_r(start=0.051) "Steam mass fraction of the riser";
    Real x_vr "Steam mean volume fraction in the riser";
    // Evaporation heat
    Inputs.InputPower W_evap "Heat rate to the risers";
    // Drum water level
    Units.Height l "Water level";
    Units.Height l_w "Water level";
    Units.Height l_s "Water level";

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
  sat.Tsat = Medium.saturationTemperature(p);
  // Assume that metal temperature is equal to the saturation pressure
  T_s = sat.Tsat;

  // Densities
  rho_s = Medium.dewDensity(sat);
  rho_w = Medium.bubbleDensity(sat);
  h_s = Medium.dewEnthalpy(sat);

  // Enthalpies
  h_w = Medium.bubbleEnthalpy(sat);
  h_c = h_s - h_w; //*

    // Masses
    //M_s = rho_s*V_st;
    //M_w = rho_w*V_wt;

    // Volume
    V_t = V_st + V_wt; //*
    V_wd = V_wt - V_dc - (1 - x_vr)*V_r; //*

    // Steam mean volume fraction in risers
    x_vr = rho_w/(rho_w - rho_s)*(1 - rho_s/((rho_w - rho_s)*x_r)*ln(1 + (rho_w - rho_s)*x_r/rho_s)); //*

    // Global mass balance*
    Q_f - Q_s = der(V_st*rho_s + V_wt*rho_w);

    // Energy Balance*
    W_evap + Q_f*h_f - Q_s*h_s = der(h_s*V_st*rho_s + h_w*V_wt*rho_w - p*V_t + M_t*Cp*T_s);

    // Mass balance for riser section*
    Q_dc - Q_r = der(rho_s*x_vr*V_r + rho_w*(1 - x_vr)*V_r);

    // Energy balance for the riser section*
    W_evap + Q_dc*h_w - (x_r*h_c + h_w)*Q_r = der(rho_s*h_s*x_vr*V_r + rho_w*h_w*(1 - x_vr)*V_r - p*V_r + M_r*Cp*T_s);

    // Mass balance for the steam under the liquid level
    x_r*Q_r - Q_sd - Q_cd = der(rho_s*V_sd);

    //*drho_s*V_sd + rho_s*der(V_sd)
    Q_cd = (h_w - h_f)/h_c*Q_f + 1/h_c*(rho_s*V_sd*der(h_s) + rho_w*V_wd*der(h_w) - (V_sd + V_wd)*der(p) + M_d*Cp*der(T_s));

    // Mass flow rate on steam through the surface*
    Q_sd = rho_s/T_d*(V_sd - V_0_sd) + x_r*Q_dc + x_r*beta*(Q_dc - Q_r);

    // Total condensation mass flow rate
    Q_ct = (h_w - h_f)/h_c*Q_f + 1/h_c*(rho_s*V_st*der(h_s) + rho_w*V_wt*der(h_w) - V_t*der(p) + M_t*Cp*der(T_s)); //*

    // Mass flow rate in the downcomers*
    0.5*k*Q_dc^2 = rho_w*A_dc*(rho_w - rho_s)*g*x_vr*V_r;

    // Level equation*
    l = (V_wd + V_sd)/A_d - l_0;
    l_w = V_wd/A_d - l_w_0;
    l_s = V_sd/A_d - l_s_0;

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
end SteamDrum_Astom_SimplifiedGeometry_8;
