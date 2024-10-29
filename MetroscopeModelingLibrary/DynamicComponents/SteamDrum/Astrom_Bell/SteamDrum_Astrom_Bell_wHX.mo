within MetroscopeModelingLibrary.DynamicComponents.SteamDrum.Astrom_Bell;
model SteamDrum_Astrom_Bell_wHX "A simple heat exchange with the flue gas as integrated, as well as a water extraction"

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  replaceable package Medium = Modelica.Media.Water.StandardWater constrainedby Modelica.Media.Interfaces.PartialTwoPhaseMedium
                                                    "Medium model";

  // Steady-state boolean
    parameter Boolean steady_state = false;

  // Constants
    parameter Real pi = Constants.pi;
    parameter Real g = Constants.g;

  // Parameters
    // Volumes
    parameter Units.Volume V_D = 40 "Drum volume";
    parameter Units.Volume V_r = 37 "Riser volume";
    parameter Units.Volume V_dc = 11 "Downcomers volume";
    parameter Units.Volume V_t = V_D + V_r + V_dc "Total system volume";

    // Masses
    parameter Units.Mass M_d = 0 "Drum mass";
    parameter Units.Mass M_r = 160000 "Riser mass";
    parameter Units.Mass M_t = 300000 "Total mass of the system";
    // Surfaces
    parameter Units.Area A_dc "Downcomers cross sectional surface";
    parameter Units.Area A_d = 20 "Drum wet area at normal operating level";
    // Metal heat capacity
    parameter Units.HeatCapacity Cp = 550 "Heat capacity of the metal";
    // Empirical relation of Q_sd
    Inputs.InputVolume V_0_sd "Hypothetical volume of bubbles";
    parameter Real beta = 0.3 "Constant";
    parameter Real T_d = 12 "Steam residence time in the drum";
    // Friction coefficient
    Inputs.InputReal k "Friction coefficient in the downcomers";
    // Levels at nominal operating conditions
    parameter Units.Height l_0 = 0;
    parameter Units.Height l_w_0 = 0;
    parameter Units.Height l_s_0 = 0;
    // Heat transfer
    parameter Units.HeatExchangeCoefficient K_conv_fg = 46 "FG heat transfer coefficient";
    parameter Units.Area A_fg_tubes = 3283.7;
    parameter Units.Area A_fg_fins = 58635.5;
    Inputs.InputReal eta_fins(start=0.77);

  // Initialization parameters
    parameter Units.Pressure p_0 = 85e5;
    parameter Units.Volume V_wt_0 = 57.2;

  // Variables
    // Drum pressure
    Units.Pressure p "Drum saturation pressure"; // (start=p_0)
    // Mass flow rates
    Units.PositiveMassFlowRate Q_f(start=49.9195) "Feed water mass flow rate";
    Units.PositiveMassFlowRate Q_s(start=49.9195) "Steam mass flow rate";
    Units.PositiveMassFlowRate Q_dc "Downcomers mass flow rate";
    Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
    Units.PositiveMassFlowRate Q_sd "Steam mass flow rate across the surface";
    Units.PositiveMassFlowRate Q_cd "Drum condensation mass flow rate";
    Units.PositiveMassFlowRate Q_ct "Total condensation mass flow rate";
    Units.PositiveMassFlowRate Q_w_out "Water extraction from steam drum";
    // Volumes
    Units.Volume V_st "Total steam volume";
    Units.Volume V_sd(start=4.8) "Drum steam volume";
    Units.Volume V_wt(start=V_wt_0, fixed = true) "Total water volume";
    Units.Volume V_wd "Drum water volume";
    // Masses
    Units.Mass M_wt "Total water mass";
    Units.Mass M_st "Total steam mass";
    Units.Mass M_wr "Riser water mass";
    Units.Mass M_sr "Riser steam mass";
    Units.Mass M_sd "Drum steam mass under water level";
    Units.Mass M_wd "Drum steam mass";
    // Enthalpies
    Units.SpecificEnthalpy h_s "Steam enthalpy";
    Units.SpecificEnthalpy h_w "Liquid enthalpy";
    Units.SpecificEnthalpy h_f "Feedwater enthalpy";
    Units.SpecificEnthalpy h_c "Condensation enthalpy";
    // Flue gas temperature
    Units.Temperature T_fg_in "Flue gas inlet temperature";
    Units.Temperature T_fg_out "Flue gas outlet temperature";
    // Densities
    Units.Density rho_s "Steam density";
    Units.Density rho_w "Liquid density";
    // Saturation temperature
    Units.Temperature T_s "Metal wall temperature";
    // Set saturation state
    Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat "Saturation state for properties calculation";
    // Riser mass and volume fractions
    Real x_r(start=0.051) "Steam mass fraction of the riser"; // (start=0.051)
    Real x_vr "Steam mean volume fraction in the riser";
    // Evaporation heat
    Units.Power W_evap(start=85.91238e6) "Heat rate to the risers";
    // Drum water level
    Units.Height l "Water level";
    Units.Height l_w "Water level";
    Units.Height l_s "Water level";
    // Circulation ratio
    Real CR "Circulation ratio";

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
  FlueGases.Pipes.HeatLoss fg_cooling annotation (Placement(transformation(extent={{-20,-50},{0,-30}})));
  WaterSteam.BoundaryConditions.Source Water_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={4,-80})));
  WaterSteam.BaseClasses.IsoPHFlowModel Water_extraction annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,-80})));
  FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-90,-10},{-70,10}}), iconTransformation(extent={{-90,-230},{-70,-210}})));
  FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{70,-10},{90,10}}), iconTransformation(extent={{70,-230},{90,-210}})));
  WaterSteam.Connectors.Outlet steam_out annotation (Placement(transformation(extent={{-70,70},{-50,90}}), iconTransformation(extent={{-70,70},{-50,90}})));
  WaterSteam.Connectors.Inlet fw_in annotation (Placement(transformation(extent={{72,-70},{92,-50}}), iconTransformation(extent={{72,-70},{92,-50}})));
  WaterSteam.Connectors.Outlet water_out annotation (Placement(transformation(extent={{-90,-90},{-70,-70}}), iconTransformation(extent={{-90,-70},{-70,-50}})));
equation

  // Connectors
    // Enthalpies:
    h_s = Steam_source.h_out;
    h_f = FW_supply.h_in;
    h_w = Water_source.h_out;
    // Pressures
    p = FW_sink.P_in;
    p = Steam_extraction.P_in;
    p = Water_extraction.P_in;
    // Mass flow rates
    Q_f = FW_supply.Q;
    Q_s = Steam_extraction.Q;
    Q_w_out = Water_extraction.Q;
    // Flue gas properties
    T_fg_in = fg_cooling.T_in;
    T_fg_out = fg_cooling.T_out;

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
    h_c = h_s - h_w;

  // Volumes
    // Total volume of the system
    V_t = V_st + V_wt;
    // Drum water volume
    V_wd = V_wt - V_dc - (1 - x_vr)*V_r;
  // Masses
    // Total
    M_wt = rho_w*V_wt;
    M_st = rho_s*V_st;
    // Masses in the riser
    M_sr = rho_s*x_vr*V_r;
    M_wr = rho_w*(1 - x_vr)*V_r;
    // Masses in drum under water level
    M_sd = rho_s*V_sd;
    M_wd = rho_w*V_wd;

  // Balance equations
    if steady_state then
      // Global mass balance
      Q_f - Q_s - Q_w_out = 0;
      // Global energy Balance
      W_evap + Q_f*h_f - Q_s*h_s - Q_w_out*h_w = 0;
      // Mass balance for riser section
      Q_dc - Q_r = 0; // computes Q_r
      // Energy balance for the riser section
      W_evap + Q_dc*h_w - (x_r*h_c + h_w)*Q_r = 0;
      // Mass balance for the steam under the liquid level
      x_r*Q_r - Q_sd - Q_cd = 0;
    else
      // Global mass balance
      Q_f - Q_s - Q_w_out = der(M_wt + M_st);
      // Global energy Balance
      W_evap + Q_f*h_f - Q_s*h_s - Q_w_out*h_w = der(M_st*h_s + M_wt*h_w - p*V_t + M_t*Cp*T_s);
      // Mass balance for riser section
      Q_dc - Q_r = der(M_sr + M_wr);
      // Energy balance for the riser section
      W_evap + Q_dc*h_w - (x_r*h_c + h_w)*Q_r = der(M_sr*h_s + M_wr*h_w - p*V_r + M_r*Cp*T_s);
      // Mass balance for the steam under the liquid level
      x_r*Q_r - Q_sd - Q_cd = der(M_sd);

    end if;

  // Condensation mass flow rate
  if steady_state then
    Q_cd = (h_w - h_f)/h_c*Q_f;
    Q_ct = (h_w - h_f)/h_c*Q_f;
  else
    Q_cd = (h_w - h_f)/h_c*Q_f + 1/h_c*(M_sd*der(h_s) + M_wd*der(h_w) - (V_sd + V_wd)*der(p) + M_d*Cp*der(T_s));
    Q_ct = (h_w - h_f)/h_c*Q_f + 1/h_c*(M_st*der(h_s) + M_wt*der(h_w) - V_t*der(p) + M_t*Cp*der(T_s));
  end if;

  // Mass flow rate on steam through the surface
  Q_sd = rho_s/T_d*(V_sd - V_0_sd) + x_r*Q_dc + x_r*beta*(Q_dc - Q_r);
  // Residence time
  //T_d = rho_s*V_0_sd/Q_sd;
  // Mass flow rate in the downcomers*
  0.5*k*Q_dc^2 = rho_w*A_dc*(rho_w - rho_s)*g*x_vr*V_r;
  // Steam mean volume fraction in risers
  x_vr = rho_w/(rho_w - rho_s)*(1 - rho_s/((rho_w - rho_s)*x_r)*log(1 + (rho_w - rho_s)*x_r/rho_s));
  // Level equation*
  l = (V_wd + V_sd)/A_d - l_0;
  l_w = V_wd/A_d - l_w_0;
  l_s = V_sd/A_d - l_s_0;
  // Circulation ratio
  CR = Q_dc/Q_s;

  // Heat exchange with flue gas
  W_evap = - fg_cooling.W;
  W_evap = K_conv_fg*(0.5*(T_fg_in + T_fg_out) - T_s)*(A_fg_tubes + eta_fins*A_fg_fins);

initial equation
  if not steady_state then
    der(x_r) = 0;
    der(V_sd) = 0;
    der(V_wt) = 0;
  end if;
equation
  connect(Steam_extraction.C_out,steam_out)  annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
  connect(Steam_extraction.C_in,Steam_source. C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
  connect(FW_supply.C_in,fw_in)  annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
  connect(FW_supply.C_out,FW_sink. C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
  connect(fg_cooling.C_in,fg_inlet)  annotation (Line(points={{-20,-40},{-80,-40},{-80,0}}, color={95,95,95}));
  connect(fg_cooling.C_out,fg_outlet)  annotation (Line(points={{0,-40},{60,-40},{60,0},{80,0}}, color={95,95,95}));
  connect(Water_source.C_out,Water_extraction. C_in) annotation (Line(points={{-1,-80},{-20,-80}}, color={28,108,200}));
  connect(water_out,water_out)  annotation (Line(points={{-80,-80},{-80,-80}}, color={28,108,200}));
  connect(Water_extraction.C_out,water_out)  annotation (Line(points={{-40,-80},{-80,-80}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-320},{100,100}}), graphics={
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
          fillPattern=FillPattern.Solid),
          Rectangle(
          extent={{-80,-120},{80,-320}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(
          points={{8,-300}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Ellipse(
          extent={{-50,-150},{-30,-170}},
          lineColor={0,0,0},
          lineThickness=1),
        Ellipse(
          extent={{-50,-290},{-30,-310}},
          lineColor={0,0,0},
          lineThickness=1),
        Line(
          points={{-40,-170},{-40,-290}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier,
          pattern=LinePattern.Dash),
        Line(
          points={{-34,-168},{-30,-210},{-30,-250},{-34,-292}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier,
          pattern=LinePattern.Dash),
        Line(
          points={{-46,-168},{-50,-210},{-50,-250},{-46,-292}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier,
          pattern=LinePattern.Dash),
        Line(
          points={{-30,-162},{-20,-210},{-20,-250},{-30,-298}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier,
          pattern=LinePattern.Dash),
        Line(
          points={{-50,-162},{-60,-210},{-60,-250},{-50,-298}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier,
          pattern=LinePattern.Dash),
        Line(
          points={{40,-92},{40,-300},{-30,-300}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{-40,-92},{-40,-150}},
          color={28,108,200},
          pattern=LinePattern.Dash,
          thickness=1),
        Text(
          extent={{-98,-330},{98,-366}},
          textColor={28,108,200},
          textString="%name")}),            Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-320},{100,100}})));
end SteamDrum_Astrom_Bell_wHX;
