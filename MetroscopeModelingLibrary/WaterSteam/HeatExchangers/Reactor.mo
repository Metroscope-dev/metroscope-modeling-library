within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model Reactor

  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Units;

  Inputs.InputMassFraction vapor_fraction;
  Inputs.InputPressure steam_pressure;

  Units.SpecificEnthalpy h_vap_sat;
  Units.SpecificEnthalpy h_liq_sat;

  Units.PositiveMassFlowRate Q_feedwater_measured(start=1000, nominal=1000); // Measured feedwater mass flow rate
  Units.PositiveMassFlowRate Q_steam(start=1000, nominal=1000);

  Inputs.InputPower thermal_power;

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage feed_water_flow_rate_measurement_bias(min = 0, max=20, nominal=1); // Flow rate measurement bias

  Connectors.Inlet feedwater_inlet annotation (Placement(transformation(extent={{20,-10},{40,10}}), iconTransformation(extent={{20,-10},{40,10}})));
  Connectors.Outlet steam_outlet annotation (Placement(transformation(extent={{-10,90},{10,110}}),  iconTransformation(extent={{-10,90},{10,110}})));

  BoundaryConditions.Source steam_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,66})));
  BoundaryConditions.Sink feedwater_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={2,0})));
  Power.Connectors.Inlet C_thermal_power annotation (Placement(transformation(extent={{-40,-10},{-20,10}}), iconTransformation(extent={{-40,-10},{-20,10}})));
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

  // Mass balance
  Q_steam = Q_feedwater_measured;

  // Power
  thermal_power = Q_steam*steam_source.h_out - Q_feedwater_measured*feedwater_sink.h_in; // thermal power here refers to the estimation of thermal power so it corresponds to the measured feedwater flow rate
  thermal_power = C_thermal_power.W;

  connect(steam_source.C_out, steam_outlet) annotation (Line(points={{2.77556e-16,71},{2.77556e-16,104.5},{0,104.5},{0,100}},
                                                      color={28,108,200}));
  connect(feedwater_sink.C_in, feedwater_inlet) annotation (Line(points={{7,-1.11022e-15},{18.5,-1.11022e-15},{18.5,0},{30,0}},
                                                color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-100},{60,100}}),
                        graphics={
        Rectangle(
          extent={{-34,44},{34,-62}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-34,-22},{34,-96}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{-32,18},{-32,18},{-44,48},{44,48},{30,18},{10,18},{-32,18}},
          pattern=LinePattern.None,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-34,100},{34,40}},
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
          extent={{-60,-100},{60,100}})),
            Icon(coordinateSystem(preserveAspectRatio=false)),
                                   Diagram(coordinateSystem(preserveAspectRatio=
           false)));
end Reactor;
