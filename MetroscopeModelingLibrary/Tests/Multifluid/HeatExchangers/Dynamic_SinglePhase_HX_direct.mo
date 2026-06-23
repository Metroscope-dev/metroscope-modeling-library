within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Dynamic_SinglePhase_HX_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent = {{10,-10},{-10,10}},
        rotation=270,
        origin={0,-166})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent = {{10,-10},{-10,10}},
        rotation=270,
        origin={0,122})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source(h_0=991711.2)
                                                                           annotation (Placement(transformation(extent={{-150,-10},{-130,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  MultiFluid.HeatExchangers.Dynamic_SinglePhase_HX HX annotation (Placement(transformation(extent={{-50,-50},{50,50}})));
  Modelica.Blocks.Sources.Ramp T_fg_in(
    height=-20,
    duration=60,
    offset=635,
    startTime=300) annotation (Placement(transformation(extent={{-140,30},{-120,50}})));
  Sensors_Control.FlueGases.TemperatureSensor temperatureSensor(h_0=991711.2, sensor_function="BC")
                                                                annotation (Placement(transformation(extent={{-120,-10},{-100,10}})));
  Sensors_Control.FlueGases.PressureSensor pressureSensor(
    h_0=991711.2,                                         sensor_function="BC", P_start=1) annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  Utilities.Interfaces.BoundaryCondition P_fg annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-80,40}), iconTransformation(extent={{-282,16},{-242,56}})));
  Sensors_Control.FlueGases.FlowSensor flowSensor(sensor_function="BC") annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Utilities.Interfaces.BoundaryCondition Q_fg annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-50,40}), iconTransformation(extent={{-282,16},{-242,56}})));
  Sensors_Control.WaterSteam.TemperatureSensor temperatureSensor1(sensor_function="BC", T_start=500) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-140})));
  Sensors_Control.WaterSteam.PressureSensor pressureSensor1(sensor_function="BC", P_start=120) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-110})));
  Sensors_Control.WaterSteam.FlowSensor flowSensor1(sensor_function="BC", Q_start=85) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-80})));
  Utilities.Interfaces.BoundaryCondition Q_water annotation (Placement(transformation(extent={{-44,-84},{-36,-76}}), iconTransformation(extent={{-346,-96},{-306,-56}})));
  Utilities.Interfaces.BoundaryCondition P_water annotation (Placement(transformation(extent={{-44,-114},{-36,-106}}), iconTransformation(extent={{-346,-96},{-306,-56}})));
  Utilities.Interfaces.BoundaryCondition T_water_in annotation (Placement(transformation(extent={{-44,-144},{-36,-136}}), iconTransformation(extent={{-346,-96},{-306,-56}})));
  Sensors_Control.WaterSteam.TemperatureSensor temperatureSensor2(
    sensor_function="Calibration",
    causality="U",
    T_start=500)                                                                                     annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,90})));
  Utilities.Interfaces.Observable        T_water_out
                                                    annotation (Placement(transformation(extent={{-44,86},{-36,94}}),     iconTransformation(extent={{-346,-96},{-306,-56}})));
  Utilities.Interfaces.RealInput U(start=58) annotation (Placement(transformation(extent={{-64,-34},{-56,-26}}), iconTransformation(extent={{-346,-96},{-306,-56}})));
equation

  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};

  // HX.U = 28;

  connect(temperatureSensor.C_in, hot_source.C_out) annotation (Line(points={{-120,0},{-135,0}}, color={95,95,95},
      thickness=1));
  connect(pressureSensor.C_in, temperatureSensor.C_out) annotation (Line(points={{-90,0},{-100,0}}, color={95,95,95},
      thickness=1));
  connect(pressureSensor.P_sensor, P_fg) annotation (Line(points={{-80,10},{-80,40}}, color={0,0,127}));
  connect(temperatureSensor.T_sensor, T_fg_in.y) annotation (Line(points={{-110,10},{-110,40},{-119,40}}, color={0,0,127}));
  connect(pressureSensor.C_out, flowSensor.C_in) annotation (Line(
      points={{-70,0},{-60,0}},
      color={95,95,95},
      thickness=1));
  connect(HX.fg_inlet, flowSensor.C_out) annotation (Line(
      points={{-20,0},{-40,0}},
      color={95,95,95},
      thickness=1));
  connect(Q_fg, flowSensor.Q_sensor) annotation (Line(points={{-50,40},{-50,10}}, color={28,108,200}));
  connect(HX.fg_outlet, hot_sink.C_in) annotation (Line(
      points={{20,0},{53,0}},
      color={95,95,95},
      thickness=1));
  connect(temperatureSensor1.C_in, cold_source.C_out) annotation (Line(
      points={{0,-150},{0,-161}},
      color={28,108,200},
      thickness=1));
  connect(pressureSensor1.C_in, temperatureSensor1.C_out) annotation (Line(
      points={{0,-120},{0,-130}},
      color={28,108,200},
      thickness=1));
  connect(HX.water_inlet, flowSensor1.C_out) annotation (Line(
      points={{0,-50},{0,-70}},
      color={28,108,200},
      thickness=1));
  connect(flowSensor1.C_in, pressureSensor1.C_out) annotation (Line(
      points={{0,-90},{0,-100}},
      color={28,108,200},
      thickness=1));
  connect(pressureSensor1.P_sensor, P_water) annotation (Line(points={{-10,-110},{-40,-110}}, color={0,0,127}));
  connect(flowSensor1.Q_sensor, Q_water) annotation (Line(points={{-10,-80},{-40,-80}}, color={0,0,127}));
  connect(temperatureSensor1.T_sensor, T_water_in) annotation (Line(points={{-10,-140},{-40,-140}}, color={0,0,127}));
  connect(HX.water_outlet, temperatureSensor2.C_in) annotation (Line(
      points={{0,50},{0,80}},
      color={28,108,200},
      thickness=1));
  connect(temperatureSensor2.C_out, cold_sink.C_in) annotation (Line(
      points={{0,100},{0,117}},
      color={28,108,200},
      thickness=1));
  connect(temperatureSensor2.T_sensor, T_water_out) annotation (Line(points={{-10,90},{-40,90}}, color={0,0,127}));
  connect(HX.U, U) annotation (Line(points={{-21,-30},{-60,-30}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false,   extent={{-100,-100},{100,100}}),
                                                                  graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor = {255,255,255},
                fillPattern = FillPattern.Solid,
                extent = {{-100,-100},{100,100}}),
        Polygon(
          origin = {20,14},
          lineColor = {78,138,73},
          fillColor = {95,95,95},
          pattern = LinePattern.None,
          fillPattern = FillPattern.Solid,
          points = {{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}}),
        Polygon(
          origin = {20,14},
          lineColor = {78,138,73},
          fillColor = {213,213,0},
          pattern = LinePattern.None,
          fillPattern = FillPattern.Solid,
          points = {{-58,46},{-4,14},{-58,-14},{-58,46}}),
        Polygon(
          origin = {20,14},
          lineColor = {78,138,73},
          fillColor = {28,108,200},
          pattern = LinePattern.None,
          fillPattern = FillPattern.Solid,
          points = {{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}), Diagram(
        coordinateSystem(preserveAspectRatio = false, extent={{-160,-180},{140,140}})),
    experiment(StopTime=500, __Dymola_Algorithm="Dassl"));
end Dynamic_SinglePhase_HX_direct;
