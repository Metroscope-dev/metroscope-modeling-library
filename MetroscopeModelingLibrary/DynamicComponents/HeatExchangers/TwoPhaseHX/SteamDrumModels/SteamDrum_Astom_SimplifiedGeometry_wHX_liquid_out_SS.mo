within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Astom_SimplifiedGeometry_wHX_liquid_out_SS
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;
  import CorrelationConstants = MetroscopeModelingLibrary.DynamicComponents.Correlations;

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  // Constants
    parameter Real pi = Constants.pi;
    parameter Real g = Constants.g;
    parameter Boolean steady_state = false;

  // Dimensions and properties
    parameter Modelica.Units.SI.Volume V_D = 40 "Drum volume";
    parameter Units.Mass M_d = M_t - M_r "Drum mass";
    parameter Units.Mass M_t = 300000 "Total mass of the system";
    parameter Modelica.Units.SI.Volume V_r = 37 "Riser volume";
    parameter Units.Mass M_r = 160000 "Riser mass";
    parameter Units.HeatCapacity Cp = 472 "Heat capacity of the metal";
    parameter Units.Area A_dc = 0.371 "Downcomers cross sectional surface";
    parameter Units.Area A_d = 20 "Drum wet area at normal operating level";
    parameter Modelica.Units.SI.Volume V_dc = 11 "Downcomers volume";
    parameter Real k_friction = 25 "Friction coefficient in the downcomers";
    parameter Real beta = 0.3 "Constant";
    parameter Modelica.Units.SI.Volume V_t = V_D + V_r + V_dc;
    parameter Modelica.Units.SI.Volume V_0_sd = 7.662651 "Hypothetical volume of bubbles";
    parameter Real T_d = 12 "Steam residence time in the drum";

  // Initialization parameters
    parameter Units.Pressure p_0 = 85e5;
    parameter Modelica.Units.SI.Volume V_wt_0 = 57.1;

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
    Units.PositiveMassFlowRate Q_w_out "Water extraction from steam drum";
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

  // Evaporation heat
  parameter Units.HeatExchangeCoefficient K_conv_fg = 46 "FG heat transfer coefficient";
  parameter Units.Area A_fg_tubes = 3283.7;
  parameter Units.Area A_fg_fins = 58635.5;
  Inputs.InputReal eff_fins(start=0.77);
  Units.Temperature T_fg_in;
  Units.Temperature T_fg_out;

  Units.Power W_evap(start=104e6) "Heat rate to the risers";
  // Drum water level
  Units.Height l "Water level";

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
  FlueGases.Connectors.Inlet fg_inlet annotation (Placement(transformation(extent={{-90,-10},{-70,10}}), iconTransformation(extent={{-90,-230},{-70,-210}})));
  FlueGases.Connectors.Outlet fg_outlet annotation (Placement(transformation(extent={{70,-10},{90,10}}), iconTransformation(extent={{70,-230},{90,-210}})));
  FlueGases.Pipes.HeatLoss fg_cooling annotation (Placement(transformation(extent={{-20,-50},{0,-30}})));
  WaterSteam.Connectors.Outlet water_out annotation (Placement(transformation(extent={{-90,-90},{-70,-70}}), iconTransformation(extent={{-90,-70},{-70,-50}})));
  WaterSteam.BoundaryConditions.Source Water_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={4,-80})));
  WaterSteam.BaseClasses.IsoPHFlowModel Water_extraction annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,-80})));
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
    //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(p));
    // Densities
    rho_s = WaterSteamMedium.density(state_s);
    rho_w = WaterSteamMedium.density(state_w);
    //rho_b = WaterSteamMedium.density(state_s);

    // Enthalpies
    h_s = WaterSteamMedium.specificEnthalpy(state_s);
    h_w = WaterSteamMedium.specificEnthalpy(state_w);
    h_c = h_s - h_w;

  // Flue gas properties
    T_fg_in = fg_cooling.T_in;
    T_fg_out = fg_cooling.T_out;

    // Volume
    V_t = V_st + V_wt;
    V_wd = V_wt - V_dc - (1 - x_vr_mean)*V_r;

    // Steam mean volume fraction in risers
    x_vr_mean = rho_w/(rho_w - rho_s)*(1 - rho_s/((rho_w - rho_s)*x_r)*log(1 + (rho_w - rho_s)*x_r/rho_s));

    if steady_state then
      // Global mass balance
      Q_f - Q_s - Q_w_out = 0;
      // Energy Balance
      W_evap + Q_f*h_f - Q_s*h_s - Q_w_out*h_w = 0;
      // Mass balance for riser section
      Q_dc - Q_r = 0;
      // Energy balance for the riser section
      W_evap + Q_dc*h_w - (x_r*h_c + h_w)*Q_r = 0;
      // Mass balance for the steam under the liquid level
      x_r*Q_r - Q_sd - Q_cd = 0;
      Q_cd = (h_w - h_f)/h_c*Q_f;
    else
      // Global mass balance
      Q_f - Q_s - Q_w_out = V_st*((ddph_s*der(p) + ddhp_s*der(h_s))) + rho_s*der(V_st)
                + V_wt*((ddph_w*der(p) + ddhp_w*der(h_w))) + rho_w*der(V_wt);
      // Energy Balance
      W_evap + Q_f*h_f - Q_s*h_s - Q_w_out*h_w = h_s*V_st*(ddph_s*der(p) + ddhp_s*der(h_s)) + rho_s*V_st*der(h_s) + rho_s*h_s*der(V_st)
                                 + h_w*V_wt*(ddph_w*der(p) + ddhp_w*der(h_w)) + rho_w*V_wt*der(h_w) + rho_w*h_w*der(V_wt)
                                 - V_t*der(p)
                                 + M_t*Cp*der(T_wall);
      // Mass balance for riser section
      Q_dc - Q_r = x_vr_mean*V_r*(ddph_s*der(p) + ddhp_s*der(h_s)) + rho_s*V_r*der(x_vr_mean)
                 + V_r*(ddph_w*der(p) + ddhp_w*der(h_w))
                 - x_vr_mean*V_r*(ddph_w*der(p) + ddhp_w*der(h_w)) - rho_w*V_r*der(x_vr_mean);
      // Energy balance for the riser section
      W_evap + Q_dc*h_w - (x_r*h_c + h_w)*Q_r = h_s*x_vr_mean*V_r*(ddph_s*der(p) + ddhp_s*der(h_s)) + rho_s*h_s*V_r*der(x_vr_mean) + rho_s*x_vr_mean*V_r*der(h_s)
                                              + h_w*V_r*(ddph_w*der(p) + ddhp_w*der(h_w)) + rho_w*V_r*der(h_w)
                                              - h_w*x_vr_mean*V_r*(ddph_w*der(p) + ddhp_w*der(h_w)) - rho_w*h_w*V_r*der(x_vr_mean) - rho_w*x_vr_mean*V_r*der(h_w)
                                              - V_r*der(p)
                                              + M_r*Cp*der(T_wall);
      // Mass balance for the steam under the liquid level
      x_r*Q_r - Q_sd - Q_cd = (ddph_s*der(p) + ddhp_s*der(h_s))*V_sd + rho_s*der(V_sd);
      Q_cd = (h_w - h_f)/h_c*Q_f + 1/h_c*(rho_s*V_sd*der(h_s) + rho_w*V_wd*der(h_w) - (V_sd + V_wd)*der(p) + M_d*Cp*der(T_wall));
    end if;

    // Mass flow rate on steam through the surface
    Q_sd = rho_s/T_d*(V_sd - V_0_sd) + x_r*Q_dc + x_r*beta*(Q_dc - Q_r);

    // Mass flow rate in the downcomers
    0.5*k_friction*Q_dc^2 = rho_w*A_dc*(rho_w - rho_s)*g*x_vr_mean*V_r;

    // Level equation
    l = (V_wd + V_sd)/A_d;

    // Heat exchange with flue gas
    W_evap = - fg_cooling.W;
    W_evap = K_conv_fg*(0.5*(T_fg_in + T_fg_out) - T_wall)*(A_fg_tubes + eff_fins*A_fg_fins);

    // Derrivatives
    ddph_w = WaterSteamMedium.density_derp_h(state_w);
    ddph_s = WaterSteamMedium.density_derp_h(state_s);
    ddhp_w = WaterSteamMedium.density_derh_p(state_w);
    ddhp_s = WaterSteamMedium.density_derh_p(state_s);

initial equation
  if (steady_state == false) then
    der(x_r) = 0;
    der(V_sd) = 0;
    der(T_wall) = 0;
  end if;

equation
  connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
  connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
  connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
  connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
  connect(fg_cooling.C_in, fg_inlet) annotation (Line(points={{-20,-40},{-80,-40},{-80,0}}, color={95,95,95}));
  connect(fg_cooling.C_out, fg_outlet) annotation (Line(points={{0,-40},{60,-40},{60,0},{80,0}}, color={95,95,95}));
  connect(Water_source.C_out, Water_extraction.C_in) annotation (Line(points={{-1,-80},{-20,-80}}, color={28,108,200}));
  connect(water_out, water_out) annotation (Line(points={{-80,-80},{-80,-80}}, color={28,108,200}));
  connect(Water_extraction.C_out, water_out) annotation (Line(points={{-40,-80},{-80,-80}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-320},{100,100}}),
                                                                graphics={
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
          thickness=1)}),                   Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
end SteamDrum_Astom_SimplifiedGeometry_wHX_liquid_out_SS;
