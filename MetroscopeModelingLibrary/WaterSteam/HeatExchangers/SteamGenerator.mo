within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model SteamGenerator

  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Units;

  Inputs.InputMassFraction vapor_fraction;
  Inputs.InputPressure steam_pressure;
  Units.PositiveMassFlowRate Q_purge;
  Units.Power thermal_power;
  Units.Pressure P_purge;

  Units.SpecificEnthalpy h_vap_sat;
  Units.SpecificEnthalpy h_liq_sat;

  Units.PositiveMassFlowRate Q_feedwater_measured(start=1000, nominal=1000); // Measured feedwater mass flow rate
  Units.PositiveMassFlowRate Q_steam(start=1000, nominal=1000);

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage feed_water_flow_rate_measurement_bias(min = 0, max=20, nominal=1); // Flow rate measurement bias


  Connectors.Inlet feedwater_inlet annotation (Placement(transformation(extent={{20,-10},{40,10}}), iconTransformation(extent={{20,-10},{40,10}})));
  Connectors.Outlet purge_outlet annotation (Placement(transformation(extent={{-10,-128},{10,-108}}), iconTransformation(extent={{-10,-128},{10,-108}})));
  Connectors.Outlet steam_outlet annotation (Placement(transformation(extent={{-10,110},{10,130}}), iconTransformation(extent={{-10,110},{10,130}})));

  BoundaryConditions.Source steam_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,82})));
  BoundaryConditions.Sink feedwater_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={10,0})));
  BoundaryConditions.Source purge_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-82})));
  Power.Connectors.Inlet C_thermal_power annotation (Placement(transformation(extent={{-40,-10},{-20,10}}), iconTransformation(extent={{-40,-10},{-20,10}})));
  Power.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
equation

  // Fault modes
  if not faulty then
    feed_water_flow_rate_measurement_bias = 0; // a bias of +5 kg/s means that 1005 kg/s is measured, while only 1000 kg/s actually go into the SG
  end if;

  // Steam mass fraction
  steam_source.P_out = steam_pressure;
  h_liq_sat= Water.bubbleEnthalpy(Water.setSat_p(steam_pressure));
  h_vap_sat= Water.dewEnthalpy(Water.setSat_p(steam_pressure));
  steam_source.h_out = vapor_fraction*h_vap_sat + (1 - vapor_fraction)*h_liq_sat;

  // Mass flow rates definitions
  Q_feedwater_measured = feedwater_sink.Q_in + feed_water_flow_rate_measurement_bias; // Note : feedwater_sink.Q_in is the actual flow going into the steam generator
  Q_steam = - steam_source.Q_out;
  Q_purge = -purge_source.Q_out;

  // Mass balance
  Q_steam = Q_feedwater_measured - Q_purge;

  // Power
  thermal_power = Q_steam*steam_source.h_out - Q_feedwater_measured*feedwater_sink.h_in + Q_purge*purge_source.h_out; // thermal power here refers to the estimation of thermal power so it corresponds to the measured feedwater flow rate
  thermal_power = C_thermal_power.W;

  // Purge
  purge_source.h_out = Water.bubbleEnthalpy(Water.setSat_p(P_purge));

  purge_source.P_out = P_purge;



  connect(steam_source.C_out, steam_outlet) annotation (Line(points={{2.77556e-16,87},{2.77556e-16,106.5},{0,106.5},{0,120}},
                                                      color={28,108,200}));
  connect(feedwater_sink.C_in, feedwater_inlet) annotation (Line(points={{15,-1.11022e-15},{18.5,-1.11022e-15},{18.5,0},{30,0}},
                                                color={28,108,200}));
  connect(purge_source.C_out, purge_outlet) annotation (Line(points={{-8.88178e-16,
          -87},{-8.88178e-16,-102.5},{0,-102.5},{0,-118}}, color={28,108,200}));
  connect(sink.C_in, C_thermal_power) annotation (Line(points={{-15,0},{-30,0}}, color={244,125,35}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-120},
            {60,120}}), graphics={
        Rectangle(
          extent={{-32,26},{30,-80}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-32,-42},{30,-116}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{-32,26},{-32,26},{-44,48},{44,48},{30,26},{10,26},{-32,26}},
          pattern=LinePattern.None,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-44,120},{44,48}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-44,48},{44,82}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid)}),
                         Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-60,-120},{60,120}})),
            Icon(coordinateSystem(preserveAspectRatio=false)),
                                   Diagram(coordinateSystem(preserveAspectRatio=
           false)));
end SteamGenerator;
