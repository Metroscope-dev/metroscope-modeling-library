within MetroscopeModelingLibrary.MultiFluid;
package HeatExchangers
  extends MetroscopeModelingLibrary.Icons.HeatExchangePackage;

  model Economiser
    extends Partial.HeatExchangers.hrsg_monophasic_HX(QCp_max_side = "cold",T_cold_in_0=76 + 273.15,P_cold_in_0 = 18 *1e5,Q_cold_0=178);

  end Economiser;

  model Superheater
    extends Partial.HeatExchangers.hrsg_monophasic_HX(QCp_max_side = "hot",T_cold_in_0=140 + 273.15,P_cold_in_0 = 3.5 *1e5, Q_cold_0= 11);
    import MetroscopeModelingLibrary.Units.Inputs;


  end Superheater;

  model Evaporator
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
      Units.Power W_vaporising;
      Inputs.InputHeatExchangeCoefficient Kth;
      parameter String HX_config="phase_change";
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

    FlueGases.Connectors.Inlet C_hot_in(Q(start=Q_hot_0)) annotation (Placement(transformation(
            extent={{-80,-12},{-60,8}}),  iconTransformation(extent={{-80,-12},{-60,8}})));
    FlueGases.Connectors.Outlet C_hot_out(Q(start=Q_hot_0)) annotation (Placement(transformation(
            extent={{60,-12},{80,8}}),  iconTransformation(extent={{60,-12},{80,8}})));
    WaterSteam.Connectors.Inlet C_cold_in(Q(start=Q_cold_0)) annotation (Placement(transformation(
            extent={{40,60},{60,80}}),   iconTransformation(extent={{40,60},{60,80}})));
    WaterSteam.Connectors.Outlet C_cold_vap_out(Q(start=Q_cold_0)) annotation (Placement(transformation(extent={{-80,80},{-60,100}}), iconTransformation(extent={{-80,80},{-60,100}})));
    WaterSteam.Connectors.Outlet C_cold_liq_out annotation (Placement(transformation(extent={{-80,-100},{-60,-80}}), iconTransformation(extent={{-80,-100},{-60,-80}})));
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
          origin={50,34})));
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
    WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-52,12},{-72,32}})));

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
    HX_vaporising.T_cold_in = Tsat;//cold_side_vaporising.T_in;
    HX_vaporising.T_hot_in = hot_side_vaporising.T_in;
    HX_vaporising.Cp_cold = 10000000;// not supposed to be used because Cp(fluid changing phase) = infinite
    HX_vaporising.Cp_hot = MetroscopeModelingLibrary.Media.FlueGasesMedium.specificHeatCapacityCp(hot_side_vaporising.state_in);



    connect(hot_side_pipe.C_out, hot_side_vaporising.C_in) annotation (Line(points={{-38,-20},{-30,-20}}, color={95,95,95}));
    connect(C_cold_in, cold_side_pipe.C_in) annotation (Line(points={{50,70},{50,44}}, color={28,108,200}));
    connect(hot_side_pipe.C_in, C_hot_in) annotation (Line(points={{-58,-20},{-70,-20},{-70,-2}}, color={95,95,95}));
    connect(cold_side_vaporising.C_in, cold_side_heating.C_out) annotation (Line(points={{-10,20},{10,20}}, color={28,108,200}));
    connect(cold_side_pipe.C_out, cold_side_heating.C_in) annotation (Line(points={{50,24},{50,20},{30,20}}, color={28,108,200}));
    connect(hot_side_vaporising.C_out, hot_side_heating.C_in) annotation (Line(points={{-10,-20},{10,-20}}, color={95,95,95}));
    connect(hot_side_heating.C_out, C_hot_out) annotation (Line(points={{30,-20},{70,-20},{70,-2}}, color={95,95,95}));
    connect(cold_side_vaporising.C_out, flashTank.C_in) annotation (Line(points={{-30,20},{-46,20},{-46,26},{-52,26}}, color={28,108,200}));
    connect(flashTank.C_hot_steam, C_cold_vap_out) annotation (Line(points={{-72,26},{-76,26},{-76,76},{-70,76},{-70,90}}, color={28,108,200}));
    connect(flashTank.C_hot_liquid, C_cold_liq_out) annotation (Line(points={{-72,18},{-84,18},{-84,-90},{-70,-90}}, color={28,108,200}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
            extent={{-68,48},{70,-52}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-18,84},{18,48}},
            lineColor={0,0,0},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{2,52},{24,0},{2,-56},{-32,0},{-16,64}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Polygon(
            points={{-18,62},{-8,50},{8,50},{14,58},{16,64},{16,66},{-12,68},{-16,66},{-16,62},{-18,62}},
            lineColor={28,108,200},
            lineThickness=1,
            smooth=Smooth.Bezier,
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-14,64},{-12,62}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-10,66},{-6,62}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Line(
            points={{6,54},{28,0},{6,-56},{-28,0},{-14,62}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{10,54},{32,0},{10,-56},{-24,0},{-10,62}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Ellipse(
            extent={{-10,60},{-8,58}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Line(
            points={{46,64},{12,64}},
            color={28,108,200},
            thickness=1),
          Line(
            points={{-70,90},{0,90},{0,78}},
            color={170,213,255},
            thickness=1),
          Line(
            points={{0,-52},{0,-90},{-68,-90}},
            color={28,108,200},
            thickness=1)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Evaporator;

  model Evaporator_without_flashTank
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
      Units.Power W_vaporising;
      Inputs.InputHeatExchangeCoefficient Kth;
      parameter String HX_config="phase_change";
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
    HX_vaporising.T_cold_in = Tsat;//cold_side_vaporising.T_in;
    HX_vaporising.T_hot_in = hot_side_vaporising.T_in;
    HX_vaporising.Cp_cold = 10000000;// not supposed to be used because Cp(fluid changing phase) = infinite
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
            smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Evaporator_without_flashTank;
end HeatExchangers;
