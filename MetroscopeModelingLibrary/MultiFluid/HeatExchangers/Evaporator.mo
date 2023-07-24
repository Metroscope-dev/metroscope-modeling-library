within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Evaporator
    extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
    package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
    import MetroscopeModelingLibrary.Utilities.Units;
    import MetroscopeModelingLibrary.Utilities.Units.Inputs;

    // Pressure Losses
    Inputs.InputFrictionCoefficient Kfr_cold;
    Inputs.InputFrictionCoefficient Kfr_hot(start=0);
    Inputs.InputArea S;

    // Heating
    Units.Power W_heating;

    // Vaporisation
    Inputs.InputHeatExchangeCoefficient Kth;
    parameter String HX_config="evaporator";

    Units.Power W_vap;
    Units.MassFraction x_steam_out(start=1); // Steam mass fraction at water outlet
    Units.SpecificEnthalpy h_vap_sat(start=h_vap_sat_0);
    Units.SpecificEnthalpy h_liq_sat(start=h_liq_sat_0);
    Units.Temperature Tsat(start=T_cold_out_0);

    // Definitions
    Units.MassFlowRate Q_cold(start=Q_cold_0);
    Units.MassFlowRate Q_hot(start=Q_hot_0);
    Units.Temperature T_cold_in(start=T_cold_in_0);
    Units.Temperature T_hot_in(start=T_hot_in_0);
    Units.Temperature T_cold_out(start=T_cold_out_0);
    Units.Temperature T_hot_out(start=T_hot_out_0);

    // Indicators
    Units.Temperature T_approach(start=T_cold_out_0-T_cold_in_0);

    // Failure modes
    parameter Boolean faulty = false;
    Units.Percentage fouling; // Fouling percentage

    // Initialization parameters
      // Flow Rates
      parameter Units.MassFlowRate Q_cold_0 = 85;
      parameter Units.MassFlowRate Q_hot_0 = 640;
      // Temperatures
      parameter Units.Temperature T_cold_in_0 = 326 + 273.15;
      parameter Units.Temperature T_cold_out_0 = WaterSteamMedium.saturationTemperature(P_cold_out_0);
      parameter Units.Temperature T_hot_in_0 = 475 + 273.15;
      parameter Units.Temperature T_hot_out_0 = 345 + 273.15;
      // Pressures
      parameter Units.Pressure P_cold_in_0 = 130e5;
      parameter Units.Pressure P_cold_out_0 = 130e5;
      parameter Units.Pressure P_hot_in_0 = 1.1e5;
      parameter Units.Pressure P_hot_out_0 = 1.05e5;
      // Enthalpies
      parameter Units.SpecificEnthalpy h_cold_in_0 = 1.5e6;
      parameter Units.SpecificEnthalpy h_vap_sat_0 = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_cold_out_0));
      parameter Units.SpecificEnthalpy h_liq_sat_0 = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_cold_out_0));
      parameter Units.SpecificEnthalpy h_hot_in_0 = 8.05e5;
      parameter Units.SpecificEnthalpy h_hot_out_0 = 6.5e5;

  FlueGases.Pipes.Pipe hot_side_pipe(Q_0=Q_hot_0, h_0=h_hot_in_0, P_in_0=P_hot_in_0, P_out_0=P_hot_out_0) annotation (Placement(transformation(extent={{-58,-30},{-38,-10}})));
  Power.HeatExchange.NTUHeatExchange HX_vaporising(config=HX_config, T_cold_in_0=T_cold_in_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-20,0})));
  FlueGases.BaseClasses.IsoPFlowModel hot_side_vaporising(Q_0=Q_hot_0, P_0=P_hot_out_0, h_in_0=h_hot_in_0) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-20,-20})));
  WaterSteam.BaseClasses.IsoPFlowModel cold_side_vaporising(Q_0=Q_cold_0, P_in_0=P_cold_out_0, h_in_0=h_liq_sat_0, h_out_0=h_vap_sat_0) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={-20,20})));
  WaterSteam.Pipes.Pipe cold_side_pipe(Q_0=Q_cold_0, h_0=h_cold_in_0, P_in_0=P_cold_in_0, P_out_0=P_cold_out_0) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={42,34})));
  FlueGases.BaseClasses.IsoPFlowModel hot_side_heating(Q_0=Q_hot_0, P_0=P_hot_out_0, h_out_0=h_hot_out_0) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={20,-20})));
  WaterSteam.BaseClasses.IsoPFlowModel cold_side_heating(Q_0=Q_cold_0, h_in_0=h_cold_in_0, P_0=P_cold_out_0, h_out_0=h_liq_sat_0) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={20,20})));
  FlueGases.Connectors.Inlet C_hot_in(Q(start=Q_hot_0), P(start=P_hot_in_0)) annotation (Placement(transformation(
          extent={{-110,-10},{-90,10}}),iconTransformation(extent={{-110,-10},{-90,10}})));
  FlueGases.Connectors.Outlet C_hot_out(Q(start=-Q_hot_0), P(start=P_hot_out_0), h_outflow(start = h_hot_out_0)) annotation (Placement(transformation(
          extent={{90,-10},{110,10}}),iconTransformation(extent={{90,-10},{110,10}})));
  WaterSteam.Connectors.Inlet C_cold_in(Q(start=Q_cold_0), P(start=P_cold_in_0)) annotation (Placement(transformation(
          extent={{30,70},{50,90}}),   iconTransformation(extent={{30,70},{50,90}})));
  WaterSteam.Connectors.Outlet C_cold_out(Q(start=-Q_cold_0), P(start=P_cold_out_0), h_outflow(start = h_vap_sat_0)) annotation (Placement(transformation(extent={{-50,70},{-30,90}}), iconTransformation(extent={{-50,70},{-30,90}})));

equation
  // Failure modes
  if not faulty then
    fouling = 0;
  end if;

  // Definitions
  Q_cold = cold_side_heating.Q;
  Q_hot = hot_side_vaporising.Q;
  T_cold_in = cold_side_heating.T_in;
  T_cold_out = cold_side_vaporising.T_out;
  T_hot_in = hot_side_vaporising.T_in;
  T_hot_out = hot_side_heating.T_out;
  Tsat = cold_side_heating.T_out;
  h_vap_sat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(cold_side_heating.P_in));
  h_liq_sat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(cold_side_heating.P_in));

  // Indicators
  T_approach = cold_side_heating.DT;

    // Pressure losses
  cold_side_pipe.delta_z=0;
  cold_side_pipe.Kfr = Kfr_cold;
  hot_side_pipe.delta_z=0;
  hot_side_pipe.Kfr = Kfr_hot;

  /* heating*/
  // Energy balance
  hot_side_heating.W + cold_side_heating.W = 0;
  cold_side_heating.W = W_heating;

  // Power Exchange
  if cold_side_heating.h_in < h_liq_sat then
      cold_side_heating.h_out = h_liq_sat; // if water is not yet saturated, it is first heated
  else
      cold_side_heating.h_out = cold_side_heating.h_in; // case when water enters the evaporator and is already in gas phase.
  end if;

  /* Vaporising */
  // Energy balance
  hot_side_vaporising.W + cold_side_vaporising.W = 0;
  cold_side_vaporising.W =W_vap;

  // Power Exchange
  cold_side_vaporising.h_out = x_steam_out * h_vap_sat + (1-x_steam_out)*h_liq_sat;

  HX_vaporising.W =W_vap;
  HX_vaporising.Kth = Kth*(1-fouling/100);
  HX_vaporising.S =S;
  HX_vaporising.Q_cold = Q_cold;
  HX_vaporising.Q_hot = Q_hot;
  HX_vaporising.T_cold_in = Tsat;
  HX_vaporising.T_hot_in = hot_side_vaporising.T_in;
  HX_vaporising.Cp_cold = 0; // Not used by NTU method in evaporator mode
  HX_vaporising.Cp_hot =MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium.specificHeatCapacityCp(hot_side_vaporising.state_in);

  connect(hot_side_pipe.C_out,hot_side_vaporising. C_in) annotation (Line(points={{-38,-20},{-30,-20}}, color={95,95,95}));
  connect(C_cold_in,cold_side_pipe. C_in) annotation (Line(points={{40,80},{40,58},{42,58},{42,44}},
                                                                                     color={28,108,200}));
  connect(hot_side_pipe.C_in,C_hot_in)  annotation (Line(points={{-58,-20},{-100,-20},{-100,0}},color={95,95,95}));
  connect(cold_side_vaporising.C_in,cold_side_heating. C_out) annotation (Line(points={{-10,20},{10,20}}, color={28,108,200}));
  connect(cold_side_pipe.C_out,cold_side_heating. C_in) annotation (Line(points={{42,24},{42,20},{30,20}}, color={28,108,200}));
  connect(hot_side_vaporising.C_out,hot_side_heating. C_in) annotation (Line(points={{-10,-20},{10,-20}}, color={95,95,95}));
  connect(hot_side_heating.C_out,C_hot_out)  annotation (Line(points={{30,-20},{100,-20},{100,0}},color={95,95,95}));
  connect(cold_side_vaporising.C_out, C_cold_out) annotation (Line(points={{-30,20},{-30,80},{-40,80}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-44,80},{-44,48},{-42,18},{-38,-8},{-24,-32},{-6,-46}},
          color={28,108,200},
          thickness=1,
          pattern=LinePattern.Dash,
          smooth=Smooth.Bezier),
        Line(
          points={{-40,80},{-40,48},{-38,18},{-34,-8},{-20,-32},{-2,-46}},
          color={28,108,200},
          thickness=1,
          pattern=LinePattern.Dash,
          smooth=Smooth.Bezier),
        Line(
          points={{-36,80},{-36,48},{-34,18},{-30,-8},{-16,-32},{2,-46}},
          color={28,108,200},
          thickness=1,
          pattern=LinePattern.Dash,
          smooth=Smooth.Bezier),
        Line(
          points={{36,82},{36,50},{34,20},{30,-6},{16,-30},{-6,-46}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Line(
          points={{40,82},{40,50},{38,20},{34,-6},{20,-30},{-2,-46}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Line(
          points={{44,82},{44,50},{42,20},{38,-6},{24,-30},{2,-46}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Evaporator;
