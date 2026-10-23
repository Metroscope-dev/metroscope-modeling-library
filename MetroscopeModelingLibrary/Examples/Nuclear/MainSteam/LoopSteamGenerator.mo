within MetroscopeModelingLibrary.Examples.Nuclear.MainSteam;
model LoopSteamGenerator

  // Parameters

  WaterSteam.HeatExchangers.SteamGenerator steamGenerator annotation (Placement(transformation(extent={{-100,-38},{-60,38}})));
  SensorsControl.WaterSteam.PressureSensor steam_pressure_sensor(sensor_function="BC",
    causality="",
    P_start=70,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(70),
    Q_0=490,
    h_0=1e6) annotation (Placement(transformation(extent={{-68,50},{-48,70}})));
  SensorsControl.WaterSteam.FlowSensor feedwater_flow_rate_sensor(sensor_function="BC",
    causality="",
    Q_start=500,
    signal_unit="kg/s",
    display_unit="kg/s",
    Q_0=500,
    h_0=1e6,
    P_0=8000000) annotation (Placement(transformation(extent={{10,-10},{-10,10}})));
  SensorsControl.WaterSteam.TemperatureSensor feedwater_temperature_sensor(sensor_function="BC",
    causality="",
    T_start=225,
    signal_unit="degC",
    display_unit="degC",
    T_0=273.15 + (225),
    Q_0=500,
    h_0=1e6,
    P_0=8000000) annotation (Placement(transformation(extent={{40,-10},{20,10}})));
  SensorsControl.WaterSteam.PressureSensor feedwater_pressure_sensor(sensor_function="BC",
    causality="",
    P_start=80,
    signal_unit="barA",
    display_unit="barA",
    Q_0=500,
    h_0=1e6,
    P_0=8000000) annotation (Placement(transformation(extent={{70,-10},{50,10}})));
  SensorsControl.WaterSteam.FlowSensor blowdown_flow_rate_sensor(sensor_function="BC",
    causality="",
    Q_start=10,
    signal_unit="kg/s",
    display_unit="kg/s",
    Q_0=10,
    h_0=1e6,
    P_0=7000000) annotation (Placement(transformation(extent={{-68,-70},{-48,-50}})));
  WaterSteam.Pipes.PressureCut pressureCut annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
  WaterSteam.Pipes.HeatLoss heatLoss annotation (Placement(transformation(extent={{-4,50},{16,70}})));
  WaterSteam.Pipes.PressureCut pressureCut1 annotation (Placement(transformation(extent={{34,50},{54,70}})));
  WaterSteam.Pipes.LoopBreaker loopBreaker annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-38,0})));
  Power.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-126,-10},{-106,10}})));
  // MML4 instrument interfaces and component characteristics.
  Utilities.Interfaces.BoundaryCondition steam_pressure annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-58,90})));
  Utilities.Interfaces.BoundaryCondition feedwater_flow_rate annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={0,30})));
  Utilities.Interfaces.BoundaryCondition feedwater_pressure annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={60,30})));
  Utilities.Interfaces.BoundaryCondition feedwater_temperature annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={30,30})));
  Utilities.Interfaces.BoundaryCondition blowdown_flow_rate annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-58,-30})));
  Utilities.Interfaces.RealOutput vapor_fraction annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-113.333,11.3333})));
  output Utilities.Units.Power loop_heat_loss;
  output Utilities.Units.MassFlowRate loop_flow_error;

equation

  connect(pressureCut.C_out, feedwater_pressure_sensor.C_in) annotation (Line(points={{-10,-60},{80,-60},{80,0},{70,0}}, color={28,108,200}));
  connect(steamGenerator.steam_outlet, steam_pressure_sensor.C_in) annotation (Line(points={{-80,38},{-80,60},{-68,60}}, color={28,108,200},
      thickness=1));
  connect(feedwater_temperature_sensor.C_out, feedwater_flow_rate_sensor.C_in) annotation (Line(points={{20,0},{10,0}}, color={28,108,200},
      thickness=1));
  connect(feedwater_pressure_sensor.C_out, feedwater_temperature_sensor.C_in) annotation (Line(points={{50,0},{40,0}}, color={28,108,200},
      thickness=1));
  connect(steamGenerator.purge_outlet, blowdown_flow_rate_sensor.C_in) annotation (Line(points={{-80,-37.3666667},{-80,-60},{-68,-60}}, color={28,108,200}));
  connect(blowdown_flow_rate_sensor.C_out, pressureCut.C_in) annotation (Line(points={{-48,-60},{-30,-60}}, color={28,108,200}));
  connect(steam_pressure_sensor.C_out, heatLoss.C_in) annotation (Line(points={{-48,60},{-4,60}}, color={28,108,200},
      thickness=1));
  connect(heatLoss.C_out, pressureCut1.C_in) annotation (Line(points={{16,60},{34,60}}, color={28,108,200},
      thickness=1));
  connect(pressureCut1.C_out, feedwater_pressure_sensor.C_in) annotation (Line(points={{54,60},{80,60},{80,0},{70,0}},   color={28,108,200},
      thickness=1));
  connect(steamGenerator.feedwater_inlet, loopBreaker.C_out) annotation (Line(points={{-70,0},{-59,0},{-48,0}}, color={28,108,200},
      thickness=1));
  connect(loopBreaker.C_in, feedwater_flow_rate_sensor.C_out) annotation (Line(points={{-28,0},{-19,0},{-10,0}}, color={28,108,200},
      thickness=1));
  connect(steamGenerator.C_thermal_power, source.C_out) annotation (Line(points={{-90,0},{-111.2,0}}, color={244,125,35}));

  // MML4 signal connections and fixed characteristics.
  connect(steam_pressure_sensor.P_sensor, steam_pressure) annotation (Line(points={{-58,70},{-58,90}}, color={0,0,127}));
  connect(feedwater_flow_rate_sensor.Q_sensor, feedwater_flow_rate) annotation (Line(points={{0,10},{0,30}}, color={0,0,127}));
  connect(feedwater_pressure_sensor.P_sensor, feedwater_pressure) annotation (Line(points={{60,10},{60,30}}, color={0,0,127}));
  connect(feedwater_temperature_sensor.T_sensor, feedwater_temperature) annotation (Line(points={{30,10},{30,30}}, color={0,0,127}));
  connect(blowdown_flow_rate_sensor.Q_sensor, blowdown_flow_rate) annotation (Line(points={{-58,-50},{-58,-30}}, color={0,0,127}));
  connect(vapor_fraction, steamGenerator.vapor_fraction) annotation (Line(points={{-113.333,11.3333},{-95.3333,11.3333},{-95.3333333,11.3333},{-95.3333333,25.3333333}}, color={0,0,127}));
  vapor_fraction = 0.99;
  loop_heat_loss = heatLoss.W_input;
  loop_flow_error = loopBreaker.loop_flow_error;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-80},{100,100}}), graphics={
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
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-80},{100,100}}),
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
