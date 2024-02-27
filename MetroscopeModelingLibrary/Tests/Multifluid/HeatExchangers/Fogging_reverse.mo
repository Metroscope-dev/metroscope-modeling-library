within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Fogging_reverse

  // Boundary conditions
  input Real P_fg_in(start = 1.1, min = 0, nominal = 10) "barA";
  input Utilities.Units.MassFlowRate Q_fg_in(start = 660) "kg/s";
  input Real T_fg_in(start = 28.8)  "degC";

  input Real P_water(start = 5, min = 0, nominal = 10) "barA";
  input Real Q_water(start = 2.5) "kg/s";
  input Real T_water(start = 19.86, min = 0, nominal = 50) "degC";

  // Calibrated parameters
  input Utilities.Units.MassFraction x_vapor(start=1);

  // Calibration inputs
  output Real T_fg_out(start=23.5) "degC";

  MultiFluid.HeatExchangers.Fogging fogging annotation (Placement(transformation(extent={{10,-30},{30,-10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_w annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,84})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source_fg annotation (Placement(transformation(extent={{-94,-30},{-74,-10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{74,-30},{94,-10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_water_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,62})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_water_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,36})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_water_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={20,10})));
  MetroscopeModelingLibrary.Sensors.FlueGases.FlowSensor Q_fg_in_sensor annotation (Placement(transformation(extent={{-72,-30},{-52,-10}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor P_fg_in_sensor annotation (Placement(transformation(extent={{-46,-30},{-26,-10}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor T_fg_in_sensor annotation (Placement(transformation(extent={{-20,-30},{0,-10}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.TemperatureSensor T_fg_out_sensor annotation (Placement(transformation(extent={{54,-30},{74,-10}})));
equation

  // Boundary conditions
  P_fg_in_sensor.P_barA = P_fg_in;
  Q_fg_in_sensor.Q = Q_fg_in;
  T_fg_in_sensor.T_degC = T_fg_in;
  source_fg.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  P_water_sensor.P_barA = P_water;
  Q_water_sensor.Q = Q_water;
  T_water_sensor.T_degC = T_water;


  // Calibrated parameters
  fogging.x_vapor = x_vapor;

  // Calibration inputs
  T_fg_out_sensor.T_degC = T_fg_out;

  connect(T_water_sensor.C_in, source_w.C_out) annotation (Line(points={{20,72},{20,79}}, color={28,108,200}));
  connect(Q_water_sensor.C_in, T_water_sensor.C_out) annotation (Line(points={{20,46},{20,52}}, color={28,108,200}));
  connect(fogging.C_water_in, P_water_sensor.C_out) annotation (Line(points={{20,-14},{20,0}}, color={28,108,200}));
  connect(P_water_sensor.C_in, Q_water_sensor.C_out) annotation (Line(points={{20,20},{20,26}}, color={28,108,200}));
  connect(Q_fg_in_sensor.C_in, source_fg.C_out) annotation (Line(points={{-72,-20},{-79,-20}}, color={95,95,95}));
  connect(P_fg_in_sensor.C_in, Q_fg_in_sensor.C_out) annotation (Line(points={{-46,-20},{-52,-20}}, color={95,95,95}));
  connect(fogging.C_fg_inlet, T_fg_in_sensor.C_out) annotation (Line(points={{10,-20},{0,-20}}, color={95,95,95}));
  connect(T_fg_in_sensor.C_in, P_fg_in_sensor.C_out) annotation (Line(points={{-20,-20},{-26,-20}}, color={95,95,95}));
  connect(fogging.C_fg_out, T_fg_out_sensor.C_in) annotation (Line(points={{30,-20},{54,-20}}, color={95,95,95}));
  connect(T_fg_out_sensor.C_out, sink.C_in) annotation (Line(points={{74,-20},{79,-20}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(
          origin={20,14},
          lineColor={78,138,73},
          fillColor={95,95,95},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}}),
        Polygon(
          origin={20,14},
          lineColor={78,138,73},
          fillColor={213,213,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58,46},{-4,14},{-58,-14},{-58,46}}),
        Polygon(
          origin={20,14},
          lineColor={78,138,73},
          fillColor={28,108,200},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Fogging_reverse;
