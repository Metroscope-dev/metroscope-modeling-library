within MetroscopeModelingLibrary.Examples.CCGT;
model Evaporator_noRecirculation_reverse
    input Real P_hot_source(start=1*1e5, min=1*1e5, nominal=1*1e5);
  input Utilities.Units.MassFlowRate Q_hot_source(start=586);
  input Real hot_source_h(start=494000);

  input Real P_cold_source(start=3.5*1e5, min=1.5*1e5, nominal=3.5*1e5);
  input Utilities.Units.MassFlowRate Q_cold_source(start=96);
  input Real T_cold_source(start = 132+273.15, min = 130+273.15, nominal = 150+273.15);

  // Parameters
  parameter Utilities.Units.Area S=10;

  // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth;
  output Utilities.Units.FrictionCoefficient Kfr_hot;
  output Utilities.Units.FrictionCoefficient Kfr_cold;

  // Calibration inputs
  input Real P_cold_out(start = 3.5, min=1.5, nominal=3.5) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real P_hot_out(start=1, min=1, nominal=1) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
  input Real Q_cold_liq_out(start = 97, min = 80, nominal = 97) "kg/s"; // Outlet temperature on cold side, to calibrate Kth
  MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-38,-36},{40,36}})));
  WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-26,38},{-46,58}})));
  WaterSteam.BoundaryConditions.Sink cold_liquid_sink annotation (Placement(transformation(extent={{-78,-62},{-98,-42}})));
  WaterSteam.BoundaryConditions.Sink cold_steam_sink annotation (Placement(transformation(extent={{-80,42},{-100,62}})));
  FlueGases.BoundaryConditions.Source                           hot_source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{72,30},{52,50}})));
  FlueGases.BoundaryConditions.Sink                           hot_sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  Sensors.WaterSteam.PressureSensor                                P_cold_out_sensor annotation (Placement(transformation(extent={{-58,60},{-74,44}})));
  Sensors.WaterSteam.FlowSensor                                Q_cold_liquid_out annotation (Placement(transformation(extent={{8,-8},{-8,8}},
        rotation=90,
        origin={-50,-36})));
  Sensors.FlueGases.PressureSensor                                    P_hot_out_sensor  annotation (Placement(transformation(extent={{44,-4},{52,4}})));
equation

    // Boundary conditions
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source;
  hot_source.h_out = hot_source_h;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source;
  cold_source.T_out =  T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  // Parameters
  evaporator.S = S;

  // Inputs for calibration
  Q_cold_liquid_out.Q = Q_cold_liq_out;
  P_cold_out_sensor.P_barA = P_cold_out;
  P_hot_out_sensor.P_barA = P_hot_out;

  // Calibrated parameters
  evaporator.Kth = Kth;
  evaporator.Kfr_hot = Kfr_hot;
  evaporator.Kfr_cold = Kfr_cold;

  connect(flashTank.C_in, evaporator.C_cold_out) annotation (Line(points={{-26,52},{-10.7,52},{-10.7,25.2}}, color={28,108,200}));
  connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-26.3,-0.72},{-55.65,-0.72},{-55.65,0},{-85,0}}, color={95,95,95}));
  connect(evaporator.C_cold_in, cold_source.C_out) annotation (Line(points={{12.7,25.2},{12.7,40},{57,40}}, color={28,108,200}));
  connect(evaporator.C_hot_out, P_hot_out_sensor.C_in) annotation (Line(points={{28.3,-0.72},{36.15,-0.72},{36.15,0},{44,0}}, color={95,95,95}));
  connect(hot_sink.C_in, P_hot_out_sensor.C_out) annotation (Line(points={{59,0},{52,0}}, color={95,95,95}));
  connect(cold_liquid_sink.C_in, Q_cold_liquid_out.C_out) annotation (Line(points={{-83,-52},{-50,-52},{-50,-44}}, color={28,108,200}));
  connect(flashTank.C_hot_liquid, Q_cold_liquid_out.C_in) annotation (Line(points={{-46,44},{-50,44},{-50,-28}}, color={28,108,200}));
  connect(flashTank.C_hot_steam, P_cold_out_sensor.C_in) annotation (Line(points={{-46,52},{-58,52}}, color={28,108,200}));
  connect(cold_steam_sink.C_in, P_cold_out_sensor.C_out) annotation (Line(points={{-85,52},{-74,52}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-40,76},{-4,40}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-40,54},{-30,42},{-14,42},{-8,50},{-6,56},{-6,58},{-34,60},{-38,58},{-38,54},{-40,54}},
          lineColor={28,108,200},
          lineThickness=1,
          smooth=Smooth.Bezier,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-36,56},{-34,54}},
          lineThickness=1,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-32,58},{-28,54}},
          lineThickness=1,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-32,52},{-30,50}},
          lineThickness=1,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
          Rectangle(
          extent={{-60,40},{78,-60}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Line(
          points={{40,40},{40,-12},{10,-62},{-22,-10},{-22,54}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Line(
          points={{36,40},{36,-12},{6,-62},{-26,-10},{-26,54}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Line(
          points={{44,40},{44,-12},{14,-62},{-18,-10},{-18,54}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Evaporator_noRecirculation_reverse;
