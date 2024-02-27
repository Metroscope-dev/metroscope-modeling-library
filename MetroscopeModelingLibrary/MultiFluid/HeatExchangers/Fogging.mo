within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model Fogging
  package FlueGasesMedium =
      MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  package WaterSteamMedium =
      MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // Boundary Conditions
  Inputs.InputMassFlowRate Q_fg_in(start=Q_fg_in_0) "Inlet fg mass flow rate at the inlet";

  // Parameters
  Inputs.InputMassFraction x_vapor(start=1); // Vapor mass fraction

  // Definitions
  Units.SpecificEnthalpy h_vap_sat;
  Units.SpecificEnthalpy h_liq_sat;
  Units.MassFlowRate Q_w_in(start=Q_w_in_0) "Inlet water mass flow rate at the inlet";

  // Initialization parameters
  // Flow Rates
  parameter Units.MassFlowRate Q_fg_in_0 = 500;
  parameter Units.MassFlowRate Q_w_in_0 = 1;


  WaterSteam.Connectors.Inlet C_water_in annotation (Placement(transformation(extent={{-10,50},{10,70}}), iconTransformation(extent={{-10,50},{10,70}})));
  WaterSteam.Pipes.HeatLoss water_evaporation annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-14})));
  WaterSteam.BoundaryConditions.Sink sink_w annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-44})));
  FlueGases.Connectors.Inlet C_fg_inlet annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
  FlueGases.Pipes.HeatLoss evaporative_cooling annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  FlueGases.BoundaryConditions.Source source_fg annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,44})));
  FlueGases.Connectors.Outlet C_fg_out annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{90,-10},{110,10}})));
  WaterSteam.Pipes.PressureCut fogging_nozzle_w annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,14})));
  FlueGases.Pipes.PressureCut pressureCut annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,20})));
equation

  // Boundary Conditions
  evaporative_cooling.Q = Q_fg_in;

  // Definitions
  h_vap_sat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(water_evaporation.P_in));
  h_liq_sat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(water_evaporation.P_in));
  fogging_nozzle_w.Q = Q_w_in;

  // Parameters
  x_vapor = (water_evaporation.h_out - h_liq_sat)/(h_vap_sat - h_liq_sat);

  // Energy balance
  water_evaporation.W = - evaporative_cooling.W;

  // Mixing
  source_fg.P_out = sink_w.P_in;
  source_fg.Q_out = - sink_w.Q_in;
  source_fg.T_out = evaporative_cooling.T_out;
  source_fg.Xi_out[1] = 0;
  source_fg.Xi_out[2] = 0;
  source_fg.Xi_out[3] = 1;
  source_fg.Xi_out[4] = 0;
  source_fg.Xi_out[5] = 0;
  water_evaporation.P_in = evaporative_cooling.P_in;


  connect(evaporative_cooling.C_out, C_fg_out) annotation (Line(points={{-40,0},{100,0}},
                                                                                        color={95,95,95}));
  connect(water_evaporation.C_out, sink_w.C_in) annotation (Line(points={{-1.77636e-15,-24},{-1.77636e-15,-27.5},{8.88178e-16,-27.5},{8.88178e-16,-39}},
                                                                                                                                                     color={28,108,200}));
  connect(fogging_nozzle_w.C_out, water_evaporation.C_in) annotation (Line(points={{-1.77636e-15,4},{-1.77636e-15,-1},{1.77636e-15,-1},{1.77636e-15,-4}}, color={28,108,200}));
  connect(fogging_nozzle_w.C_in, C_water_in) annotation (Line(points={{1.77636e-15,24},{1.77636e-15,41},{0,41},{0,60}}, color={28,108,200}));
  connect(source_fg.C_out, pressureCut.C_in) annotation (Line(points={{40,39},{40,30}}, color={95,95,95}));
  connect(pressureCut.C_out, C_fg_out) annotation (Line(points={{40,10},{40,0},{100,0}}, color={95,95,95}));
  connect(evaporative_cooling.C_in, C_fg_inlet) annotation (Line(points={{-60,0},{-100,0}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={95,95,95},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-2,60},{2,-52}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(points={{2,40},{12,46}}, color={28,108,200}),
        Line(points={{2,40},{12,40}}, color={28,108,200}),
        Line(points={{2,40},{12,34}}, color={28,108,200}),
        Line(points={{2,0},{12,6}}, color={28,108,200}),
        Line(points={{2,0},{12,0}}, color={28,108,200}),
        Line(points={{2,0},{12,-6}}, color={28,108,200}),
        Line(points={{2,20},{12,26}}, color={28,108,200}),
        Line(points={{2,20},{12,20}}, color={28,108,200}),
        Line(points={{2,20},{12,14}}, color={28,108,200}),
        Line(points={{2,-20},{12,-14}}, color={28,108,200}),
        Line(points={{2,-20},{12,-20}}, color={28,108,200}),
        Line(points={{2,-20},{12,-26}}, color={28,108,200}),
        Ellipse(
          extent={{20,40},{22,38}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{32,34},{34,32}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{42,26},{44,24}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{48,24},{50,22}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{24,18},{26,16}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{26,6},{28,4}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{36,8},{38,6}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{26,-4},{28,-6}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{44,8},{46,6}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{56,2},{58,0}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{66,-6},{68,-8}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{72,-8},{74,-10}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{48,-14},{50,-16}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{36,0},{38,-2}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{46,-8},{48,-10}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{52,-10},{54,-12}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{28,-16},{30,-18}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{66,8},{68,6}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{64,14},{66,12}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{70,12},{72,10}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{50,12},{52,10}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{36,18},{38,16}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{58,24},{60,22}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{68,30},{70,28}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{66,36},{68,34}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{72,34},{74,32}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(points={{2,-40},{12,-34}}, color={28,108,200}),
        Line(points={{2,-40},{12,-40}}, color={28,108,200}),
        Line(points={{2,-40},{12,-46}}, color={28,108,200}),
        Ellipse(
          extent={{30,-32},{32,-34}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{38,-32},{40,-34}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{50,-38},{52,-40}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{30,-40},{32,-42}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{44,-28},{46,-30}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{30,-22},{32,-24}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{58,-30},{60,-32}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{56,-24},{58,-26}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{62,-26},{64,-28}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{74,-26},{76,-28}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{84,-34},{86,-36}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{90,-36},{92,-38}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{66,-42},{68,-44}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{70,-38},{72,-40}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{84,-20},{86,-22}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{76,8},{78,6}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{74,14},{76,12}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{80,12},{82,10}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{42,40},{44,38}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{40,46},{42,44}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{46,44},{48,42}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{52,40},{54,38}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{50,46},{52,44}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{56,44},{58,42}},
          lineColor={95,95,95},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end Fogging;
