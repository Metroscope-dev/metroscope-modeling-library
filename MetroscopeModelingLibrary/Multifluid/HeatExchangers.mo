within MetroscopeModelingLibrary.MultiFluid;
package HeatExchangers
  extends MetroscopeModelingLibrary.Icons.HeatExchangePackage;

  model Economiser
    extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
    extends Partial.HeatExchangers.hrsg_monophasic_HX(QCp_max_side = "cold",T_cold_in_0=76 + 273.15,P_cold_in_0 = 18 *1e5,Q_cold_0=178)
   annotation(IconMap(primitivesVisible=false));

    annotation (Icon(graphics={
            Rectangle(
            extent={{-70,50},{70,-50}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid), Line(
            points={{30,66},{30,-60},{10,-60},{10,64},{-10,64},{-10,-60},{-30,-60},{-30,66}},
            color={28,108,200},
            smooth=Smooth.Bezier,
            thickness=1)}));
  end Economiser;

  model Superheater
    extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
    extends Partial.HeatExchangers.hrsg_monophasic_HX(QCp_max_side = "hot",T_cold_in_0=140 + 273.15,P_cold_in_0 = 3.5 *1e5, Q_cold_0= 11)
   annotation(IconMap(primitivesVisible=false));
    import MetroscopeModelingLibrary.Units.Inputs;


    annotation (Icon(graphics={
            Rectangle(
            extent={{-70,50},{70,-50}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Line(
            points={{30,66},{30,-60},{10,-60},{10,64},{-10,64},{-10,-60},{-30,-60},{-30,66}},
            color={170,213,255},
            smooth=Smooth.Bezier,
            thickness=1),
          Line(
            points={{28,68},{30,-56},{10,-60},{12,66},{-12,66},{-10,-60},{-30,-56},{-28,68}},
            color={28,108,200},
            smooth=Smooth.Bezier),
          Line(
            points={{32,66},{32,-60},{8,-62},{10,62},{-10,62},{-8,-62},{-32,-60},{-32,66}},
            color={28,108,200},
            smooth=Smooth.Bezier)}));
  end Superheater;

  model Evaporator
    extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
     package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
      import MetroscopeModelingLibrary.Units;
      import MetroscopeModelingLibrary.Units.Inputs;

      // Pressure Losses
      Inputs.InputFrictionCoefficient Kfr_cold;
      Inputs.InputFrictionCoefficient Kfr_hot;
      Inputs.InputArea S_vaporising;

      // Heating
      Units.Power W_heating;

      // Vaporisation
      Inputs.InputHeatExchangeCoefficient Kth;
      parameter String HX_config="evaporator";

      Units.Power W_vaporising;
      Units.MassFraction x_steam_out(start=0.7); // Steam mass fraction at water outlet
      Units.SpecificEnthalpy h_vap_sat(start=2e6);
      Units.SpecificEnthalpy h_liq_sat(start=1e5);
      Units.Temperature Tsat;

      // Definitions
      Units.MassFlowRate Q_cold;
      Units.MassFlowRate Q_hot;
      Units.Temperature T_cold_in;
      Units.Temperature T_hot_in;
      Units.Temperature T_cold_out;
      Units.Temperature T_hot_out;

      // Initialization parameters
      parameter Units.MassFlowRate Q_cold_0 = 500;
      parameter Units.MassFlowRate Q_hot_0 = 50;
      parameter Units.Temperature T_cold_in_0 = 76 + 273.15;
      parameter Units.Pressure P_cold_in_0 = 18 *1e5;

    FlueGases.Pipes.Pipe hot_side_pipe(Q_0=Q_hot_0) annotation (Placement(transformation(extent={{-58,-30},{-38,-10}})));
    Power.HeatExchange.NTUHeatExchange HX_vaporising(config=HX_config, T_cold_in_0=T_cold_in_0) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-20,0})));
    FlueGases.BaseClasses.IsoPFlowModel hot_side_vaporising(Q_0=Q_hot_0) annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=180,
          origin={-20,-20})));
    WaterSteam.BaseClasses.IsoPFlowModel cold_side_vaporising(
      Q_0=Q_cold_0,
      T_in_0=T_cold_in_0,
      P_in_0=P_cold_in_0) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=0,
          origin={-20,20})));
    WaterSteam.Pipes.Pipe cold_side_pipe(Q_0=Q_cold_0, T_in_0=T_cold_in_0) annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={42,34})));
    FlueGases.BaseClasses.IsoPFlowModel hot_side_heating(Q_0=Q_hot_0) annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=180,
          origin={20,-20})));
    WaterSteam.BaseClasses.IsoPFlowModel cold_side_heating(
      Q_0=Q_cold_0,
      T_in_0=T_cold_in_0,
      P_in_0=P_cold_in_0) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=0,
          origin={20,20})));
    FlueGases.Connectors.Inlet C_hot_in(Q(start=Q_hot_0)) annotation (Placement(transformation(
            extent={{-80,-12},{-60,8}}),  iconTransformation(extent={{-80,-12},{-60,8}})));
    FlueGases.Connectors.Outlet C_hot_out(Q(start=Q_hot_0)) annotation (Placement(transformation(
            extent={{60,-12},{80,8}}),  iconTransformation(extent={{60,-12},{80,8}})));
    WaterSteam.Connectors.Inlet C_cold_in(Q(start=Q_cold_0)) annotation (Placement(transformation(
            extent={{20,60},{40,80}}),   iconTransformation(extent={{20,60},{40,80}})));
    WaterSteam.Connectors.Outlet C_cold_out annotation (Placement(transformation(extent={{-40,60},{-20,80}}), iconTransformation(extent={{-40,60},{-20,80}})));

  equation
    // Definitions
    Q_cold = cold_side_heating.Q_in;
    Q_hot = hot_side_vaporising.Q_in;
    T_cold_in = cold_side_heating.T_in;
    T_cold_out = cold_side_heating.T_out;
    T_hot_in = hot_side_vaporising.T_in;
    T_hot_out = hot_side_vaporising.T_out;
    Tsat = cold_side_heating.T_out;
    h_vap_sat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(cold_side_heating.P_in));
    h_liq_sat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(cold_side_heating.P_in));

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
    cold_side_vaporising.W = W_vaporising;

    // Power Exchange
    cold_side_vaporising.h_out = x_steam_out * h_vap_sat + (1-x_steam_out)*h_liq_sat;

    HX_vaporising.W = W_vaporising;
    HX_vaporising.Kth = Kth;
    HX_vaporising.S = S_vaporising;
    HX_vaporising.Q_cold = Q_cold;
    HX_vaporising.Q_hot = Q_hot;
    HX_vaporising.T_cold_in = Tsat;
    HX_vaporising.T_hot_in = hot_side_vaporising.T_in;
    HX_vaporising.Cp_cold = 0; // Not used by NTU method in evaporator mode
    HX_vaporising.Cp_hot =MetroscopeModelingLibrary.Media.FlueGasesMedium.specificHeatCapacityCp(hot_side_vaporising.state_in);

    connect(hot_side_pipe.C_out,hot_side_vaporising. C_in) annotation (Line(points={{-38,-20},{-30,-20}}, color={95,95,95}));
    connect(C_cold_in,cold_side_pipe. C_in) annotation (Line(points={{30,70},{30,58},{42,58},{42,44}},
                                                                                       color={28,108,200}));
    connect(hot_side_pipe.C_in,C_hot_in)  annotation (Line(points={{-58,-20},{-70,-20},{-70,-2}}, color={95,95,95}));
    connect(cold_side_vaporising.C_in,cold_side_heating. C_out) annotation (Line(points={{-10,20},{10,20}}, color={28,108,200}));
    connect(cold_side_pipe.C_out,cold_side_heating. C_in) annotation (Line(points={{42,24},{42,20},{30,20}}, color={28,108,200}));
    connect(hot_side_vaporising.C_out,hot_side_heating. C_in) annotation (Line(points={{-10,-20},{10,-20}}, color={95,95,95}));
    connect(hot_side_heating.C_out,C_hot_out)  annotation (Line(points={{30,-20},{70,-20},{70,-2}}, color={95,95,95}));
    connect(cold_side_vaporising.C_out, C_cold_out) annotation (Line(points={{-30,20},{-30,20},{-30,70}}, color={28,108,200}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
            extent={{-68,50},{70,-50}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Line(
            points={{32,64},{32,-2},{2,-52},{-30,0},{-30,64}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{28,64},{28,-2},{-2,-52},{-34,0},{-34,64}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{36,64},{36,-2},{6,-52},{-26,0},{-26,64}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{-26,60},{-26,18.2871},{-20,-6},{-14,-20},{-4.14648,-33.7949},{4,-40}},
            color={170,213,255},
            thickness=1,
            pattern=LinePattern.Dot,
            smooth=Smooth.Bezier),
          Line(
            points={{-30,60},{-30,18.2871},{-24,-6},{-18,-20},{-8.1465,-33.7949},{0,-40}},
            color={170,213,255},
            thickness=1,
            pattern=LinePattern.Dot,
            smooth=Smooth.Bezier),
          Line(
            points={{-34,60},{-34,18.2871},{-28,-6},{-22,-20},{-12.1465,-33.795},{-4,-40}},
            color={170,213,255},
            thickness=1,
            pattern=LinePattern.Dot,
            smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Evaporator;

  model FuelHeater
    extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
    package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
    package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
    import MetroscopeModelingLibrary.Units;
    import MetroscopeModelingLibrary.Units.Inputs;

    // Pressure Losses
    Inputs.InputFrictionCoefficient Kfr_cold;
    Inputs.InputFrictionCoefficient Kfr_hot;

    // Heating
    parameter String QCp_max_side = "hot";
    parameter String HX_config = "monophasic_cross_current";
    Inputs.InputArea S;
    Inputs.InputHeatExchangeCoefficient Kth;
    Units.Power W;

    // Definitions
    Units.MassFlowRate Q_cold;
    Units.MassFlowRate Q_hot;
    Units.Temperature T_cold_in;
    Units.Temperature T_cold_out;
    Units.Temperature T_hot_in;
    Units.Temperature T_hot_out;

    // Initialization parameters
    parameter Units.MassFlowRate Q_cold_0 = 500;
    parameter Units.MassFlowRate Q_hot_0 = 50;
    parameter Units.Temperature T_cold_in_0 = 76 + 273.15;
    parameter Units.Pressure P_cold_in_0 = 18 *1e5;

    Fuel.Connectors.Inlet C_cold_in annotation (Placement(transformation(extent={{-80,-10},{-60,10}}), iconTransformation(extent={{-80,-10},{-60,10}})));
    Fuel.Connectors.Outlet C_cold_out annotation (Placement(transformation(extent={{60,-10},{80,10}}), iconTransformation(extent={{60,-10},{80,10}})));
    WaterSteam.Connectors.Inlet C_hot_in annotation (Placement(transformation(extent={{30,60},{50,80}}), iconTransformation(extent={{30,60},{50,80}})));
    WaterSteam.Connectors.Outlet C_hot_out annotation (Placement(transformation(extent={{-50,-80},{-30,-60}}),
                                                                                                             iconTransformation(extent={{-50,-80},{-30,-60}})));
    Power.HeatExchange.NTUHeatExchange HX(
      config=HX_config,
      QCp_max_side=QCp_max_side,
      T_cold_in_0=T_cold_in_0)                                                                                                  annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=0,
          origin={10,14})));
    WaterSteam.BaseClasses.IsoPFlowModel hot_side(
      Q_0=Q_cold_0,
      T_in_0=T_cold_in_0,
      P_in_0=P_cold_in_0) annotation (Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=0,
          origin={10,28})));
    WaterSteam.Pipes.Pipe hot_side_pipe(Q_0=Q_cold_0, T_in_0=T_cold_in_0) annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-14,-24})));
    Fuel.Pipes.Pipe cold_side_pipe annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
    Fuel.BaseClasses.IsoPFlowModel cold_side annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  equation
      // Definitions
    Q_cold = cold_side.Q_in;
    Q_hot = hot_side.Q_in;
    T_cold_in = cold_side.T_in;
    T_cold_out = cold_side.T_out;
    T_hot_in = hot_side.T_in;
    T_hot_out = hot_side.T_out;
    cold_side.W = W;

    // Energy balance
    hot_side.W + cold_side.W = 0;

    // Pressure losses
    cold_side_pipe.delta_z = 0;
    cold_side_pipe.Kfr = Kfr_cold;
    hot_side_pipe.delta_z = 0;
    hot_side_pipe.Kfr = Kfr_hot;

    // Power Exchange
    HX.W = W;
    HX.S = S;
    HX.Kth = Kth;
    HX.Q_cold = Q_cold;
    HX.Q_hot = Q_hot;
    HX.T_cold_in = T_cold_in;
    HX.T_hot_in = T_hot_in;
    HX.Cp_cold = FuelMedium.specificHeatCapacityCp(cold_side.state_in);
    HX.Cp_hot = WaterSteamMedium.specificHeatCapacityCp(hot_side.state_in);
    connect(hot_side_pipe.C_out, C_hot_out) annotation (Line(points={{-14,-34},{-14,-70},{-40,-70}},       color={28,108,200}));
    connect(hot_side_pipe.C_in, hot_side.C_out) annotation (Line(points={{-14,-14},{-14,28},{0,28}},color={28,108,200}));
    connect(hot_side.C_in, C_hot_in) annotation (Line(points={{20,28},{40,28},{40,70}}, color={28,108,200}));
    connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(points={{-52,0},{-70,0}}, color={213,213,0}));
    connect(cold_side_pipe.C_out, cold_side.C_in) annotation (Line(points={{-32,0},{0,0}}, color={213,213,0}));
    connect(cold_side.C_out, C_cold_out) annotation (Line(points={{20,0},{70,0}}, color={213,213,0}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
            extent={{-70,50},{70,-50}},
            lineColor={0,0,0},
            fillColor={226,230,140},
            fillPattern=FillPattern.Solid), Line(
            points={{40,66},{40,-60},{20,-60},{20,64},{0,64},{0,-60},{-20,-60},{-20,65.6309},{-40,66},{-40,-66}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(points={{122,-56}}, color={102,44,145})}),
                            Diagram(coordinateSystem(preserveAspectRatio=false)));
  end FuelHeater;
end HeatExchangers;
