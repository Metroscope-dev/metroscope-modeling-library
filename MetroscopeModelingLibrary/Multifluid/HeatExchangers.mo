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

      Inputs.InputArea S_vaporising;
      Inputs.InputHeatExchangeCoefficient Kth;
      Inputs.InputFrictionCoefficient Kfr_cold;
      Inputs.InputFrictionCoefficient Kfr_hot;
      Inputs.InputTemperature T_cold_in;
      Inputs.InputTemperature T_hot_in;

      Units.Temperature nominal_hot_side_temperature_rise; // flue gases reference temperature rise based on H&MB diagramm values
      Units.Power W;
      Units.MassFlowRate Q_cold;
      Units.MassFlowRate Q_hot;

      Units.SpecificEnthalpy h_vap_sat(start=2e6);
      Units.SpecificEnthalpy h_liq_sat(start=1e5);
      Units.Temperature Tsat;

      Units.Power W_heating;
      Units.Power W_vaporising;

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
    Power.HeatExchange.NTUHeatExchange HX_vaporising(config="evaporator", T_cold_in_0=T_cold_in_0) annotation (Placement(transformation(
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
  protected
    MetroscopeModelingLibrary.Units.HeatCapacity Cp_hot_min;
    MetroscopeModelingLibrary.Units.HeatCapacity Cp_hot_max;
    MetroscopeModelingLibrary.Media.FlueGasesMedium.ThermodynamicState state_hot_out; // estimation of the flue gases outlet thermodynamic state

  equation
    // Definitions
    Q_cold =cold_side_heating.Q_in;
    Q_hot =hot_side_vaporising.Q_in;
    T_cold_in =cold_side_heating.T_in;
    T_hot_in =hot_side_vaporising.T_in;
    //cold_side.W + liquid_cold_side.W = W;
    h_vap_sat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(hot_side_heating.P_in));
    h_liq_sat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(hot_side_heating.P_in));

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
    if hot_side_heating.h_in < h_liq_sat then
        hot_side_heating.h_out = h_liq_sat; // if water is not yet saturated, it is first heated
    else
        hot_side_heating.h_out = hot_side_heating.h_in;
    end if;


    /* Vaporising */
    // Energy balance
    hot_side_vaporising.W + cold_side_vaporising.W = 0;
    cold_side_vaporising.W = W_vaporising;
    // Power Exchange
    hot_side_vaporising.h_out = h_liq_sat;

    HX_vaporising.W = W_vaporising;
    HX_vaporising.Kth = Kth;
    HX_vaporising.S = S_vaporising;
    HX_vaporising.Q_cold = Q_cold;
    HX_vaporising.Q_hot = Q_hot;
    HX_vaporising.T_cold_in = cold_side_vaporising.T_in;
    HX_vaporising.T_hot_in = Tsat;
    HX_vaporising.Cp_cold = 1000000;// not supposed to be used because Cp(fluid changing phase) = infinite
    HX_vaporising.Cp_hot = (Cp_hot_min + Cp_hot_max)/2;


    Cp_hot_max=MetroscopeModelingLibrary.Media.FlueGasesMedium.specificHeatCapacityCp(hot_side_vaporising.state_in);// fg inlet Cp
    state_hot_out = MetroscopeModelingLibrary.Media.FlueGasesMedium.setState_pTX(hot_side_vaporising.P_in, hot_side_vaporising.T_in + nominal_hot_side_temperature_rise,hot_side_vaporising.Xi_in);
    Cp_hot_min =MetroscopeModelingLibrary.Media.FlueGasesMedium.specificHeatCapacityCp(state_hot_out);  // fg outlet Cp



    connect(hot_side_pipe.C_out, hot_side_vaporising.C_in) annotation (Line(points={{-38,-20},{-30,-20}}, color={95,95,95}));
    connect(C_cold_in, cold_side_pipe.C_in) annotation (Line(points={{50,70},{50,44}}, color={28,108,200}));
    connect(cold_side_vaporising.C_out, C_cold_vap_out) annotation (Line(points={{-30,20},{-70,20},{-70,90}}, color={28,108,200}));
    connect(cold_side_vaporising.C_out, C_cold_liq_out) annotation (Line(points={{-30,20},{-86,20},{-86,-90},{-70,-90}}, color={28,108,200}));
    connect(hot_side_pipe.C_in, C_hot_in) annotation (Line(points={{-58,-20},{-70,-20},{-70,-2}}, color={95,95,95}));
    connect(cold_side_vaporising.C_in, cold_side_heating.C_out) annotation (Line(points={{-10,20},{10,20}}, color={28,108,200}));
    connect(cold_side_pipe.C_out, cold_side_heating.C_in) annotation (Line(points={{50,24},{50,20},{30,20}}, color={28,108,200}));
    connect(hot_side_vaporising.C_out, hot_side_heating.C_in) annotation (Line(points={{-10,-20},{10,-20}}, color={95,95,95}));
    connect(hot_side_heating.C_out, C_hot_out) annotation (Line(points={{30,-20},{70,-20},{70,-2}}, color={95,95,95}));
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
end HeatExchangers;
