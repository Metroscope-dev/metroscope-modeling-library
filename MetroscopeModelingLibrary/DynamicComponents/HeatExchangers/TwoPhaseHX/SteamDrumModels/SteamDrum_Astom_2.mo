within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.TwoPhaseHX.SteamDrumModels;
model SteamDrum_Astom_2
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
    //parameter Units.HeatCapacity Cp_D = 232;
    parameter Units.Length D_D = 2.6;
    parameter Units.Length R_D = D_D/2;
    parameter Units.Length L_D = 11.160;

  // Evaporator
    parameter Units.Length D_r = 38.1e-3;
    parameter Units.Area A_r = 0.25*pi*D_r*N_tubes_r;
    parameter Units.Length L_r = 18.29;
    parameter Integer N_tubes_r = 1472; // P&ID 1560
    parameter Modelica.Units.SI.Volume V_r = A_r*L_r;
    parameter Units.Mass M_r = 159376;
    parameter Units.HeatCapacity Cp = 472;

  // Downcomers
    parameter Units.Length D_o_dc = 505e-3;
    parameter Units.Length e_dc = 38.1e-3;
    parameter Units.Length L_dc = 18.29;
    parameter Units.Density rho_steel = 7850;
    parameter Units.Area A_dc = N_dc*0.25*pi*(D_o_dc - 2*e_dc)^2;
    parameter Integer N_dc = 3;
    parameter Modelica.Units.SI.Volume V_dc = A_dc*L_dc;
    parameter Units.Mass M_dc = N_dc*0.25*pi*(D_o_dc^2 - (D_o_dc - 2*e_dc)^2)*L_dc*rho_steel;
    parameter Real k_friction = 25;

  // Total mass and volume
    parameter Units.Mass M_t = M_D + M_r + M_dc;
    parameter Modelica.Units.SI.Volume V_t = V_D + V_r + V_dc;

  // Initialization parameters
  parameter Units.Pressure P_drum_0 = 121.631e5;
  parameter Modelica.Units.SI.Volume V_wt_0 = 20;

  // Water/Steam properties
    // Drum pressure
    Units.Pressure P_drum(start=P_drum_0) "Drum saturation pressure";
    // Mass flow rates
    Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
    Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
    Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
    Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
    Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
    // Volumes
    Modelica.Units.SI.Volume V_st "Steam volume";
    Modelica.Units.SI.Volume V_b "Bubble volume";
    Modelica.Units.SI.Volume V_wt(start=V_wt_0, fixed=true) "Total water volume under the level"; //
    Modelica.Units.SI.Volume V_wd "Water volume in the drum";
    //Modelica.Units.SI.Volume V_l "Liquid volume";
    // Enthalpies
    Units.SpecificEnthalpy h_s "Steam enthalpy";
    Units.SpecificEnthalpy h_l "Liquid enthalpy";
    Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
    Units.SpecificEnthalpy h_c "Condensation enthalpy";
    //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
    //Inputs.InputSpecificEnthalpy h_r "Risers enthalpy";
    //Units.SpecificEnthalpy h_b "Bubble enthalpy";
    // Densities
    Units.Density rho_s "Steam density";
    Units.Density rho_l "Liquid density";
    //Units.Density rho_b "Bubble density";
    // States
    WaterSteamMedium.ThermodynamicState state_s "Steam state";
    WaterSteamMedium.ThermodynamicState state_l "Liquid state";
    // Wall temperature
    Units.Temperature T_wall;
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
    //Units.Mass M_l;
    //Units.Mass M_s;

    // Evaporation heat
    Inputs.InputPower W_evap(start=104e6);

    // Set saturation state
    Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

    // Derrivatives
    Real ddph_l;
    Real ddph_s;

    // Riser mass and volume fractions
    Real x_r; // Steam mass fraction of the riser
    Real x_vr_mean; // Steam mean volume fraction in the riser

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
  // Variables definition
    // Enthalpies:
    h_s = Steam_source.h_out;
    h_fw = FW_supply.h_in;
    // Pressures
    P_drum = FW_sink.P_in;
    P_drum = Steam_extraction.P_in;
    // Mass flow rates
    Q_fw = FW_supply.Q;
    //Q_r = Risers_supply.Q;
    Q_s = Steam_extraction.Q;
    //Q_d = Downcomers_extraction.Q;
    // Set saturation state
    sat.psat = P_drum;
    sat.Tsat = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

    T_wall = Modelica.Media.Water.WaterIF97_ph.saturationTemperature(P_drum);

    // States
    state_l = WaterSteamMedium.setBubbleState(sat);
    state_s = WaterSteamMedium.setDewState(sat);
    // Surface tension
    //sigma_s = WaterSteamMedium.surfaceTension(WaterSteamMedium.setSat_p(P_drum));
    // Densities
    rho_s = WaterSteamMedium.density(state_s);
    rho_l = WaterSteamMedium.density(state_l);
    //rho_b = WaterSteamMedium.density(state_s);

    // Enthalpies
    h_s = WaterSteamMedium.specificEnthalpy(state_s);
    h_l = WaterSteamMedium.specificEnthalpy(state_l);
    h_c = h_s - h_l;

    // Masses
    //M_s = rho_s*V_st;
    //M_l = rho_l*V_wt;

    // Volume
    V_t = V_st + V_wt;
    V_wd = V_wt - V_dc - (1 - x_vr_mean)*V_r;
    //V_l = V_wt - V_b;

    // Steam mean volume fraction in risers
    x_vr_mean = rho_l/(rho_l - rho_s)*(1 - rho_s/((rho_l - rho_s)*x_r)*log(1 + (rho_l - rho_s)*x_r/rho_s));

    // Mass balance
    Q_fw - Q_s = V_st*(ddph_s*der(P_drum)) + rho_s*der(V_st)
               + V_wt*(ddph_l*der(P_drum)) + rho_l*der(V_wt);

    // Energy Balance
    W_evap + Q_fw*h_fw - Q_s*h_s = h_s*V_st*ddph_s*der(P_drum) + rho_s*V_st*der(h_s) + rho_s*h_s*der(V_st)
                                 + h_l*V_wt*ddph_l*der(P_drum) + rho_l*V_wt*der(h_l) + rho_l*h_l*der(V_wt)
                                 + V_t*der(P_drum)
                                 + M_t*Cp*der(T_wall);

    // Mass balance in riser section
    Q_d - Q_r = x_vr_mean*V_r*ddph_s*der(P_drum) + rho_s*V_r*der(x_vr_mean)
              + V_r*ddph_l*der(P_drum)
              - x_vr_mean*V_r*ddph_l*der(P_drum) - rho_l*V_r*der(x_vr_mean);

    // Energy balance in the riser section
    W_evap + Q_d*h_l - (x_r*h_c + h_l)*Q_r = h_s*x_vr_mean*V_r*ddph_s*der(P_drum) + rho_s*h_s*V_r*der(x_vr_mean) + rho_s*x_vr_mean*V_r*der(h_s)
                                           + h_l*V_r*ddph_l*der(P_drum) + rho_l*V_r*der(h_l)
                                           - h_l*x_vr_mean*V_r*ddph_l*der(P_drum) - rho_l*h_l*V_r*der(x_vr_mean) - rho_l*x_vr_mean*V_r*der(h_l)
                                           - V_r*der(P_drum)
                                           + M_r*Cp*der(T_wall);

    // Mass balance for the steam under the liquid level
    x_r*Q_r - Q_b = ddph_s*der(P_drum)*V_b + rho_s*der(V_b);

    // Mass flow rate in the downcomers
    0.5*k_friction*Q_d^2 = rho_l*A_dc*(rho_l - rho_s)*g*x_vr_mean*V_r;

    // Derrivatives
    ddph_l = WaterSteamMedium.density_derp_h(state_l);
    ddph_s = WaterSteamMedium.density_derp_h(state_s);

    if Controlled_level then
      der(V_wt) = 0;
    end if;

initial equation
  der(P_drum) = 0;

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
end SteamDrum_Astom_2;
