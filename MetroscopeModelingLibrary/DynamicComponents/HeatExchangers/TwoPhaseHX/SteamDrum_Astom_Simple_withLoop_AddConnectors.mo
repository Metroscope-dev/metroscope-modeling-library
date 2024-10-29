within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers.TwoPhaseHX;
model SteamDrum_Astom_Simple_withLoop_AddConnectors
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
  parameter Units.Pressure P_drum_0 = 121.631e5;
  parameter Modelica.Units.SI.Volume V_l_0 = 20;

  // Water/Steam properties
    // Drum pressure
    Units.Pressure P_drum(start=P_drum_0, fixed=false) "Drum saturation pressure";
    // Mass flow rates
    Units.PositiveMassFlowRate Q_fw(start=80) "Feed water mass flow rate";
    //Units.PositiveMassFlowRate Q_b "Bubbles mass flow rate";
    Units.PositiveMassFlowRate Q_s(start=80) "Steam mass flow rate";
    //Units.PositiveMassFlowRate Q_d "Downcomers mass flow rate";
    //Units.PositiveMassFlowRate Q_r "Risers mass flow rate";
    Units.PositiveMassFlowRate Q_loop(start=80) "Evaporator loop mass flow rate";
    // Volumes
    Modelica.Units.SI.Volume V_s "Steam volume";
    //Modelica.Units.SI.Volume V_b "Bubble volume";
    Modelica.Units.SI.Volume V_l(start=V_l_0, fixed=true) "Liquid volume"; //
    // Enthalpies
    Units.SpecificEnthalpy h_s "Steam enthalpy";
    Units.SpecificEnthalpy h_l "Liquid enthalpy";
    Units.SpecificEnthalpy h_fw "Feedwater enthalpy";
    //Units.SpecificEnthalpy h_d "Downcomers enthalpy";
    Units.SpecificEnthalpy h_r "Risers enthalpy";
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
    Units.Mass M_l;
    Units.Mass M_s;

    // Evaporation heat
    Units.Power W_evap(start=104e6);

    // Set saturation state
    Modelica.Media.Water.WaterIF97_ph.SaturationProperties sat;

    // Derrivatives
    Real ddph_l;
    Real ddph_s;

    // Riser steam quality
    Inputs.InputReal x_r;

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
  WaterSteam.BaseClasses.IsoPHFlowModel Downcomers_extraction annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-60})));
  WaterSteam.BoundaryConditions.Sink Risers_sink annotation (Placement(transformation(extent={{-26,-70},{-6,-50}})));
  WaterSteam.BaseClasses.IsoPHFlowModel Risers_supply annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
  WaterSteam.BoundaryConditions.Source Downcomers_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-28})));
  WaterSteam.Connectors.Outlet downcomers_out annotation (Placement(transformation(extent={{-10,-110},{10,-90}}), iconTransformation(extent={{-10,-110},{10,-90}})));
  WaterSteam.Connectors.Inlet risers_in annotation (Placement(transformation(extent={{-90,-70},{-70,-50}}), iconTransformation(extent={{-90,-70},{-70,-50}})));
equation
  // Variables definition
    // Enthalpies:
    h_s = Steam_source.h_out;
    h_fw = FW_supply.h_in;
    h_l = Downcomers_extraction.h_out;
    h_r = Risers_supply.h_in;
    // Pressures
    P_drum = FW_sink.P_in;
    P_drum = Steam_extraction.P_in;
    // P_drum = Risers_supply.P_in;
    P_drum = Downcomers_extraction.P_in;
    // Mass flow rates
    Q_fw = FW_supply.Q;
    Q_loop = Risers_supply.Q;
    Q_s = Steam_extraction.Q;
    //Q_loop = Downcomers_extraction.Q;
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

    // Masses
    M_s = rho_s*V_s;
    M_l = rho_l*V_l;

    // Volume
    V_s = V_D - V_l;

    // Mass balance
    Q_fw - Q_s = V_s*(ddph_s*der(P_drum)) + rho_s*der(V_s)
               + V_l*(ddph_l*der(P_drum)) + rho_l*der(V_l);

    // Energy Balance
    Q_loop*(h_r - h_l)  + Q_fw*h_fw - Q_s*h_s = h_s*V_s*ddph_s*der(P_drum) + rho_s*V_s*der(h_s) + rho_s*h_s*der(V_s)
                                 + h_l*V_l*ddph_l*der(P_drum) + rho_l*V_l*der(h_l) + rho_l*h_l*der(V_l)
                                 + V_D*der(P_drum)
                                 + M_D*Cp_D*der(T_wall);

    W_evap = Q_loop*(h_r - h_l);
    x_r = (h_r - h_l)/(h_s - h_l);

    // Derrivatives
    ddph_l = WaterSteamMedium.density_derp_h(state_l);
    ddph_s = WaterSteamMedium.density_derp_h(state_s);

initial equation
  //der(P_drum) = 0;
  //der(V_l) = 0;
  //W_evap = Q_s*h_s - Q_fw*h_fw;

equation
  connect(Steam_extraction.C_out, steam_out) annotation (Line(points={{0,48},{0,80},{-60,80}}, color={28,108,200}));
  connect(Steam_extraction.C_in, Steam_source.C_out) annotation (Line(points={{0,28},{0,1}}, color={28,108,200}));
  connect(FW_supply.C_in, fw_in) annotation (Line(points={{60,-60},{82,-60}}, color={28,108,200}));
  connect(FW_supply.C_out, FW_sink.C_in) annotation (Line(points={{40,-60},{23,-60}}, color={28,108,200}));
  connect(Downcomers_extraction.C_out,downcomers_out)  annotation (Line(points={{0,-70},{0,-100}},         color={28,108,200}));
  connect(Downcomers_extraction.C_in,Downcomers_source. C_out) annotation (Line(points={{0,-50},{0,-33}}, color={28,108,200}));
  connect(Risers_sink.C_in,Risers_supply. C_out) annotation (Line(points={{-21,-60},{-40,-60}}, color={28,108,200}));
  connect(Risers_supply.C_in,risers_in)  annotation (Line(points={{-60,-60},{-80,-60}}, color={28,108,200}));
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
end SteamDrum_Astom_Simple_withLoop_AddConnectors;
