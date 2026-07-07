within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model OTC_reverse_3

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source(Q_0=123.7532696, h_out(
        start=762243.5))
    annotation (Placement(transformation(extent={{-216,-20},{-176,20}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink(Q_in(
        start=123.7532696))
    annotation (Placement(transformation(extent={{228,-20},{268,20}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source(Q_0=
        6.91480430909523,                                                    h_0=
        167327.44)
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={248,80})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink(Q_in(
        start=6.91480430909523))
    annotation (Placement(transformation(
        extent={{20,-20},{-20,20}},
        rotation=0,
        origin={-196,-80})));
  Utilities.Interfaces.RealOutput Kth annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
        rotation=180,
        origin={94,0}),                iconTransformation(extent={{-292,68},{-272,
            88}})));
  Utilities.Interfaces.RealExpression
                                  Kfr_cold annotation (Placement(transformation(
          extent={{6,14},{14,22}}),      iconTransformation(extent={{-282,54},{-262,
            74}})));
  Sensors_Control.WaterSteam.FlowSensor Q_cold_sensor(Q_start=6.91480430909523)
    annotation (Placement(transformation(extent={{192,70},{172,90}})));
  Sensors_Control.WaterSteam.PressureSensor P_cold_in_sensor(sensor_function="BC",
      P_start=120)
    annotation (Placement(transformation(extent={{222,70},{202,90}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_in_sensor(
    sensor_function="Calibration",
    causality="Kth",
    T_start=309.955582121001)
    annotation (Placement(transformation(extent={{162,70},{142,90}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_out_sensor(
      sensor_function="BC", T_start=460.051281987323)
    annotation (Placement(transformation(extent={{-30,-90},{-50,-70}})));
  Sensors_Control.WaterSteam.PressureSensor P_cold_out_sensor(
    sensor_function="Calibration",
    causality="Kfr_cold",
    P_start=118)
    annotation (Placement(transformation(extent={{-60,-90},{-80,-70}})));
  Sensors_Control.FlueGases.PressureSensor P_hot_in_sensor(sensor_function="BC", P_start=
        20.09)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Sensors_Control.FlueGases.FlowSensor Q_hot_sensor(sensor_function="BC", Q_start=
        123.7532696)
    annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
  Sensors_Control.FlueGases.TemperatureSensor T_hot_in_sensor(sensor_function="BC", T_start=
        463.0511864)
    annotation (Placement(transformation(extent={{-170,-10},{-150,10}})));
  Sensors_Control.FlueGases.PressureSensor P_hot_out_sensor(
    sensor_function="Calibration",
    causality="Kfr_hot",
    P_start=19.98927262)
    annotation (Placement(transformation(extent={{202,-10},{222,10}})));
  Sensors_Control.FlueGases.TemperatureSensor T_hot_out_sensor(sensor_function="BC", T_start=
        369.9805)
    annotation (Placement(transformation(extent={{172,-10},{192,10}})));
  Utilities.Interfaces.BoundaryCondition T_hot_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={182,20}), iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.BoundaryCondition T_cold_in annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-40,-60}), iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.BoundaryCondition Q_hot annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-130,20}), iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.Observable Q_cold annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={182,100}), iconTransformation(extent={{-102,84},{-82,104}})));
  Utilities.Interfaces.BoundaryCondition T_hot_in annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-160,20}), iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.BoundaryCondition P_hot_in annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-100,20}), iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.RealExpression
                                  Kfr_hot annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={10,-2}),  iconTransformation(extent={{-292,68},{-272,88}})));
  Utilities.Interfaces.Observable       P_hot_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={212,20}), iconTransformation(extent={{-226,4},{-186,44}})));
  Utilities.Interfaces.Observable        P_cold_in annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={212,100}), iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.CalibrationInput P_cold_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-70,-60}), iconTransformation(extent={{-226,4},{-186,44}})));
  Utilities.Interfaces.CalibrationInput P_cold_in1 annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={152,100}), iconTransformation(extent={{-376,26},{-336,66}})));
  MultiFluid.HeatExchangers.OTC oTC(S=10000)
                                    annotation (Placement(transformation(
        extent={{-40,50},{40,-50}},
        rotation=180,
        origin={50,0})));
equation

  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};

  connect(cold_source.C_out, P_cold_in_sensor.C_in)
    annotation (Line(points={{238,80},{222,80}}, color={28,108,200}));
  connect(P_cold_in_sensor.C_out, Q_cold_sensor.C_in)
    annotation (Line(points={{202,80},{192,80}}, color={28,108,200}));
  connect(Q_cold_sensor.C_out, T_cold_in_sensor.C_in)
    annotation (Line(points={{172,80},{162,80}}, color={28,108,200}));
  connect(cold_sink.C_in, P_cold_out_sensor.C_out)
    annotation (Line(points={{-186,-80},{-80,-80}}, color={28,108,200}));
  connect(P_cold_out_sensor.C_in, T_cold_out_sensor.C_out)
    annotation (Line(points={{-60,-80},{-50,-80}}, color={28,108,200}));
  connect(Q_hot_sensor.C_out, P_hot_in_sensor.C_in)
    annotation (Line(points={{-120,0},{-110,0}}, color={95,95,95}));
  connect(hot_source.C_out, T_hot_in_sensor.C_in)
    annotation (Line(points={{-186,0},{-170,0}}, color={95,95,95}));
  connect(T_hot_in_sensor.C_out, Q_hot_sensor.C_in)
    annotation (Line(points={{-150,0},{-140,0}}, color={95,95,95}));
  connect(P_hot_out_sensor.C_out, hot_sink.C_in)
    annotation (Line(points={{222,0},{238,0}}, color={95,95,95}));
  connect(T_hot_out_sensor.C_out, P_hot_out_sensor.C_in)
    annotation (Line(points={{192,0},{202,0}}, color={95,95,95}));
  connect(T_hot_out_sensor.T_sensor, T_hot_out)
    annotation (Line(points={{182,10},{182,20}}, color={0,0,127}));
  connect(T_cold_out_sensor.T_sensor, T_cold_in)
    annotation (Line(points={{-40,-70},{-40,-60}}, color={0,0,127}));
  connect(Q_hot_sensor.Q_sensor, Q_hot)
    annotation (Line(points={{-130,10},{-130,20}}, color={0,0,127}));
  connect(Q_cold_sensor.Q_sensor, Q_cold)
    annotation (Line(points={{182,90},{182,100}}, color={0,0,127}));
  connect(T_hot_in_sensor.T_sensor, T_hot_in)
    annotation (Line(points={{-160,10},{-160,20}}, color={0,0,127}));
  connect(P_hot_in, P_hot_in_sensor.P_sensor)
    annotation (Line(points={{-100,20},{-100,10}}, color={28,108,200}));
  connect(P_hot_out_sensor.P_sensor, P_hot_out)
    annotation (Line(points={{212,10},{212,20}}, color={0,0,127}));
  connect(P_cold_in_sensor.P_sensor, P_cold_in)
    annotation (Line(points={{212,90},{212,100}}, color={0,0,127}));
  connect(P_cold_out_sensor.P_sensor, P_cold_out)
    annotation (Line(points={{-70,-70},{-70,-60}}, color={0,0,127}));
  connect(P_cold_in1, T_cold_in_sensor.T_sensor) annotation (Line(points={{152,100},
          {152,90}},                    color={28,108,200}));
  connect(T_cold_in_sensor.C_out, oTC.C_cold_in) annotation (Line(points={{142,80},
          {6,80},{6,30},{30,30}}, color={28,108,200}));
  connect(oTC.C_cold_out, T_cold_out_sensor.C_in) annotation (Line(points={{30,-29},
          {10,-29},{10,-30},{-16,-30},{-16,-80},{-30,-80}}, color={28,108,200}));
  connect(oTC.C_hot_out, T_hot_out_sensor.C_in) annotation (Line(points={{50,50},
          {50,60},{112,60},{112,0},{172,0}}, color={95,95,95}));
  connect(P_hot_in_sensor.C_out, oTC.C_hot_in) annotation (Line(points={{-90,0},
          {-2,0},{-2,-62},{50,-62},{50,-49}}, color={95,95,95}));
  connect(oTC.Kth, Kth)
    annotation (Line(points={{75,0},{94,0}}, color={0,0,127}));
  connect(Kfr_cold.y, oTC.Kfr_cold)
    annotation (Line(points={{10,16},{10,10},{25,10}}, color={0,0,127}));
  connect(oTC.Kfr_hot, Kfr_hot.y)
    annotation (Line(points={{25,-10},{10,-10},{10,-4}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-220,-100},
            {280,160}}),                                        graphics={
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
          points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-220,
            -100},{280,160}})));
end OTC_reverse_3;
