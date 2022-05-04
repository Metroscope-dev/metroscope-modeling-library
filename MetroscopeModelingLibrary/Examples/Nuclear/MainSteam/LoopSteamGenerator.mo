within MetroscopeModelingLibrary.Examples.Nuclear.MainSteam;
model LoopSteamGenerator

  // Boundary conditions
  input Real steam_pressure(start=70) "barA";
  input Units.PositiveMassFlowRate feedwater_flow_rate(start = 500) "kg/s";
  input Real feedwater_pressure(start=80) "barA";
  input Real feedwater_temperature(start=225) "degC";
  input Units.PositiveMassFlowRate blowdown_flow_rate(start=10) "kg/s";

  // Parameters
  parameter Real vapor_fraction = 0.99;

  WaterSteam.HeatExchangers.SteamGenerator steamGenerator annotation (Placement(transformation(extent={{-100,-38},{-60,38}})));
  Sensors.WaterSteam.PressureSensor steam_pressure_sensor annotation (Placement(transformation(extent={{-68,50},{-48,70}})));
  Sensors.WaterSteam.FlowSensor feedwater_flow_rate_sensor annotation (Placement(transformation(extent={{10,-10},{-10,10}})));
  Sensors.WaterSteam.TemperatureSensor feedwater_temperature_sensor annotation (Placement(transformation(extent={{40,-10},{20,10}})));
  Sensors.WaterSteam.PressureSensor feedwater_pressure_sensor annotation (Placement(transformation(extent={{70,-10},{50,10}})));
  Sensors.WaterSteam.FlowSensor blowdown_flow_rate_sensor annotation (Placement(transformation(extent={{-68,-70},{-48,-50}})));
  WaterSteam.Pipes.PressureCut pressureCut annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
  WaterSteam.Pipes.HeatLoss heatLoss annotation (Placement(transformation(extent={{-4,50},{16,70}})));
  WaterSteam.Pipes.PressureCut pressureCut1 annotation (Placement(transformation(extent={{34,50},{54,70}})));
  WaterSteam.Pipes.LoopBreaker loopBreaker annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-38,0})));
equation

  // Quantities definitions
  steam_pressure_sensor.P_barA = steam_pressure;
  feedwater_flow_rate_sensor.Q = feedwater_flow_rate;
  feedwater_pressure_sensor.P_barA = feedwater_pressure;
  feedwater_temperature_sensor.T_degC = feedwater_temperature;
  blowdown_flow_rate_sensor.Q = blowdown_flow_rate;

  // Parameters
  steamGenerator.vapor_fraction = vapor_fraction;

  // Hypothesis
  steamGenerator.P_purge = steamGenerator.steam_pressure;

  connect(pressureCut.C_out, feedwater_pressure_sensor.C_in) annotation (Line(points={{-10,-60},{80,-60},{80,0},{70,0}}, color={28,108,200}));
  connect(steamGenerator.steam_outlet, steam_pressure_sensor.C_in) annotation (Line(points={{-80,38},{-80,60},{-68,60}}, color={28,108,200},
      thickness=1));
  connect(feedwater_temperature_sensor.C_out, feedwater_flow_rate_sensor.C_in) annotation (Line(points={{20,0},{10,0}}, color={28,108,200},
      thickness=1));
  connect(feedwater_pressure_sensor.C_out, feedwater_temperature_sensor.C_in) annotation (Line(points={{50,0},{40,0}}, color={28,108,200},
      thickness=1));
  connect(steamGenerator.purge_outlet, blowdown_flow_rate_sensor.C_in) annotation (Line(points={{-80,-37.3667},{-80,-60},{-68,-60}}, color={28,108,200}));
  connect(blowdown_flow_rate_sensor.C_out, pressureCut.C_in) annotation (Line(points={{-48,-60},{-30,-60}}, color={28,108,200}));
  connect(steam_pressure_sensor.C_out, heatLoss.C_in) annotation (Line(points={{-48,60},{-4,60}}, color={28,108,200},
      thickness=1));
  connect(heatLoss.C_out, pressureCut1.C_in) annotation (Line(points={{16,60},{34,60}}, color={28,108,200},
      thickness=1));
  connect(pressureCut1.C_out, feedwater_pressure_sensor.C_in) annotation (Line(points={{54,60},{80,60},{80,0},{70,0}},   color={28,108,200},
      thickness=1));
  connect(steamGenerator.feedwater_inlet, loopBreaker.C_out) annotation (Line(points={{-70,0},{-59,0},{-59,8.88178e-16},{-48,8.88178e-16}}, color={28,108,200},
      thickness=1));
  connect(loopBreaker.C_in, feedwater_flow_rate_sensor.C_out) annotation (Line(points={{-28,-1.77636e-15},{-19,-1.77636e-15},{-19,0},{-10,0}}, color={28,108,200},
      thickness=1));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{100,100}}), graphics={
        Rectangle(
          extent={{-32,24},{30,-82}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-32,-44},{30,-118}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{-32,24},{-32,24},{-44,46},{44,46},{30,24},{10,24},{-32,24}},
          pattern=LinePattern.None,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-44,118},{44,46}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-44,46},{44,80}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{100,100}}),
                                                                                                                      graphics={Rectangle(
          extent={{-18,80},{70,50}},
          lineThickness=1,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None), Text(
          extent={{-16,78},{66,68}},
          textString="Secondary circuit",
          textColor={28,108,200})}));
end LoopSteamGenerator;
