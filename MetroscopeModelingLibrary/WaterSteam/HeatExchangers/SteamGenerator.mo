within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model SteamGenerator

  package Water = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  import MetroscopeModelingLibrary.Units.Inputs;
  import MetroscopeModelingLibrary.Units;

  Inputs.InputMassFraction vapor_fraction;
  Inputs.InputPressure steam_pressure;
  Inputs.InputInletMassFlowRate Q_purge;
  Units.Pressure P_purge;

  Units.SpecificEnthalpy h_vap_sat;
  Units.SpecificEnthalpy h_liq_sat;

  Units.Power thermal_power;


  Connectors.Inlet feedwater_inlet annotation (Placement(transformation(extent={{20,-10},{40,10}}), iconTransformation(extent={{20,-10},{40,10}})));
  Connectors.Outlet purge_outlet annotation (Placement(transformation(extent={{-10,-128},{10,-108}}), iconTransformation(extent={{-10,-128},{10,-108}})));
  Connectors.Outlet steam_outlet annotation (Placement(transformation(extent={{-10,110},{10,130}}), iconTransformation(extent={{-10,110},{10,130}})));

  BoundaryConditions.Source steam_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,88})));
  BoundaryConditions.Sink feedwater_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={2,0})));
  BoundaryConditions.Source purge_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-82})));
equation

  // Steam mass fraction
  steam_source.P_out = steam_pressure;
  h_liq_sat= Water.bubbleEnthalpy(Water.setSat_p(steam_pressure));
  h_vap_sat= Water.dewEnthalpy(Water.setSat_p(steam_pressure));
  steam_source.h_out = vapor_fraction*h_vap_sat + (1 - vapor_fraction)*h_liq_sat;

  // Mass balance
  steam_source.Q_out + purge_source.Q_out + feedwater_sink.Q_in = 0;

  // Power
  thermal_power = -steam_source.Q_out*steam_source.h_out - feedwater_sink.Q_in*feedwater_sink.h_in - purge_source.Q_out*purge_source.h_out;

  // Purge
  purge_source.h_out = Water.bubbleEnthalpy(Water.setSat_p(purge_source.P_out));
  purge_source.Q_out = - Q_purge;
  purge_source.P_out = P_purge;



  connect(steam_source.C_out, steam_outlet) annotation (Line(points={{2.77556e-16,
          93},{2.77556e-16,106.5},{0,106.5},{0,120}}, color={28,108,200}));
  connect(feedwater_sink.C_in, feedwater_inlet) annotation (Line(points={{7,-1.11022e-15},
          {18.5,-1.11022e-15},{18.5,0},{30,0}}, color={28,108,200}));
  connect(purge_source.C_out, purge_outlet) annotation (Line(points={{-8.88178e-16,
          -87},{-8.88178e-16,-102.5},{0,-102.5},{0,-118}}, color={28,108,200}));
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
