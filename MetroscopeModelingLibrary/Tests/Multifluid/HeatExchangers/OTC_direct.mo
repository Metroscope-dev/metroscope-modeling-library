within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model OTC_direct

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source(Q_0=123.7532696, h_out(
        start=762243.5))
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={50,-164})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink(Q_in(
        start=123.7532696))
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={50,146})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source(Q_0=
        6.91480430909523,                                                    h_0=
        167327.44)
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-84,30})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink(Q_in(
        start=6.91480430909523))
    annotation (Placement(transformation(
        extent={{20,-20},{-20,20}},
        rotation=0,
        origin={-84,-30})));
  Utilities.Interfaces.RealExpression
                                  Kth(y=7.392668)
                                      annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
        rotation=270,
        origin={88,0}),                iconTransformation(extent={{-292,68},{-272,
            88}})));
  Utilities.Interfaces.RealExpression
                                  Kfr_cold(y=25000)
                                           annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
        rotation=90,
        origin={10,10}),                 iconTransformation(extent={{-282,54},{-262,
            74}})));
  Sensors_Control.WaterSteam.FlowSensor Q_cold_sensor(Q_start=6.91480430909523)
    annotation (Placement(transformation(extent={{-38,20},{-18,40}})));
  Sensors_Control.WaterSteam.PressureSensor P_cold_in_sensor(sensor_function="BC",
      P_start=120)
    annotation (Placement(transformation(extent={{-10,20},{10,40}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_in_sensor(
    sensor_function="Calibration",
    causality="Kth",
    T_start=309.955582121001)
    annotation (Placement(transformation(extent={{-68,20},{-48,40}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_out_sensor(
      sensor_function="BC", T_start=460.051281987323)
    annotation (Placement(transformation(extent={{-18,-40},{-38,-20}})));
  Sensors_Control.WaterSteam.PressureSensor P_cold_out_sensor(
    sensor_function="Calibration",
    causality="Kfr_cold",
    P_start=119.8191)
    annotation (Placement(transformation(extent={{-48,-40},{-68,-20}})));
  Sensors_Control.FlueGases.PressureSensor P_hot_in_sensor(sensor_function="BC", P_start=
        20.09)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,-70})));
  Sensors_Control.FlueGases.FlowSensor Q_hot_sensor(sensor_function="BC", Q_start=
        123.7532696)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,-100})));
  Sensors_Control.FlueGases.TemperatureSensor T_hot_in_sensor(sensor_function="BC", T_start=
        463.0511864)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,-130})));
  Sensors_Control.FlueGases.PressureSensor P_hot_out_sensor(
    sensor_function="Calibration",
    causality="Kfr_hot",
    P_start=19.98927262)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,100})));
  Sensors_Control.FlueGases.TemperatureSensor T_hot_out_sensor(sensor_function="BC", T_start=
        369.9805)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,70})));
  Utilities.Interfaces.BoundaryCondition T_hot_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={26,70}),  iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.BoundaryCondition T_cold_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-28,-10}), iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.BoundaryCondition Q_hot annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={26,-100}), iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.Observable Q_cold annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-28,50}),  iconTransformation(extent={{-166,80},{-146,100}})));
  Utilities.Interfaces.BoundaryCondition T_hot_in annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={26,-130}), iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.BoundaryCondition P_hot_in annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={26,-70}),  iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.RealExpression
                                  Kfr_hot(y=6.349405)
                                          annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={10,-10}), iconTransformation(extent={{-292,68},{-272,88}})));
  Utilities.Interfaces.Observable       P_hot_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={26,100}), iconTransformation(extent={{-226,4},{-186,44}})));
  Utilities.Interfaces.BoundaryCondition P_cold_in annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={0,50}),    iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.Observable       P_cold_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-58,-10}), iconTransformation(extent={{-226,4},{-186,44}})));
  Utilities.Interfaces.Observable T_cold_in annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-58,50}), iconTransformation(extent={{-376,26},{-336,66}})));
  MultiFluid.HeatExchangers.OTC OTC(S=10000)
                                    annotation (Placement(transformation(
        extent={{-40,50},{40,-50}},
        rotation=180,
        origin={50,0})));
equation

  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};

  connect(cold_sink.C_in, P_cold_out_sensor.C_out)
    annotation (Line(points={{-74,-30},{-68,-30}},  color={28,108,200}));
  connect(P_cold_out_sensor.C_in, T_cold_out_sensor.C_out)
    annotation (Line(points={{-48,-30},{-38,-30}}, color={28,108,200}));
  connect(T_hot_out_sensor.T_sensor, T_hot_out)
    annotation (Line(points={{40,70},{26,70}},   color={0,0,127}));
  connect(T_cold_out_sensor.T_sensor, T_cold_out)
    annotation (Line(points={{-28,-20},{-28,-10}}, color={0,0,127}));
  connect(Q_hot_sensor.Q_sensor, Q_hot)
    annotation (Line(points={{40,-100},{26,-100}}, color={0,0,127}));
  connect(Q_cold_sensor.Q_sensor, Q_cold)
    annotation (Line(points={{-28,40},{-28,50}},  color={0,0,127}));
  connect(T_hot_in_sensor.T_sensor, T_hot_in)
    annotation (Line(points={{40,-130},{26,-130}}, color={0,0,127}));
  connect(P_hot_in, P_hot_in_sensor.P_sensor)
    annotation (Line(points={{26,-70},{40,-70}},   color={28,108,200}));
  connect(P_cold_in_sensor.P_sensor, P_cold_in)
    annotation (Line(points={{0,40},{0,50}},      color={0,0,127}));
  connect(P_cold_out_sensor.P_sensor, P_cold_out)
    annotation (Line(points={{-58,-20},{-58,-10}}, color={0,0,127}));
  connect(T_cold_in, T_cold_in_sensor.T_sensor)
    annotation (Line(points={{-58,50},{-58,40}}, color={28,108,200}));
  connect(OTC.C_cold_out, T_cold_out_sensor.C_in) annotation (Line(points={{31,-30},
          {-6,-30},{-6,-30},{-18,-30}},                     color={28,108,200}));
  connect(Kfr_cold.y,OTC. Kfr_cold)
    annotation (Line(points={{12,10},{28,10}},         color={0,0,127}));
  connect(cold_source.C_out, T_cold_in_sensor.C_in)
    annotation (Line(points={{-74,30},{-68,30}}, color={28,108,200}));
  connect(T_cold_in_sensor.C_out, Q_cold_sensor.C_in)
    annotation (Line(points={{-48,30},{-38,30}}, color={28,108,200}));
  connect(Q_cold_sensor.C_out, P_cold_in_sensor.C_in)
    annotation (Line(points={{-18,30},{-10,30}}, color={28,108,200}));
  connect(P_cold_in_sensor.C_out, OTC.C_cold_in) annotation (Line(points={{10,
          30},{8,30},{8,30},{30,30}}, color={28,108,200}));
  connect(T_hot_in_sensor.C_in, hot_source.C_out)
    annotation (Line(points={{50,-140},{50,-154}}, color={95,95,95}));
  connect(P_hot_in_sensor.C_out, OTC.C_hot_in)
    annotation (Line(points={{50,-60},{50,-50}}, color={95,95,95}));
  connect(Q_hot_sensor.C_out, P_hot_in_sensor.C_in)
    annotation (Line(points={{50,-90},{50,-80}}, color={95,95,95}));
  connect(T_hot_in_sensor.C_out, Q_hot_sensor.C_in)
    annotation (Line(points={{50,-120},{50,-110}}, color={95,95,95}));
  connect(P_hot_out, P_hot_out_sensor.P_sensor)
    annotation (Line(points={{26,100},{40,100}}, color={0,0,127}));
  connect(hot_sink.C_in, P_hot_out_sensor.C_out)
    annotation (Line(points={{50,136},{50,110}}, color={95,95,95}));
  connect(P_hot_out_sensor.C_in, T_hot_out_sensor.C_out)
    annotation (Line(points={{50,90},{50,80}}, color={95,95,95}));
  connect(T_hot_out_sensor.C_in, OTC.C_hot_out)
    annotation (Line(points={{50,60},{50,50}}, color={95,95,95}));
  connect(Kth.y, OTC.Kth)
    annotation (Line(points={{86,0},{72,0}}, color={0,0,127}));
  connect(Kfr_hot.y, OTC.Kfr_hot)
    annotation (Line(points={{12,-10},{28,-10}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),                                  graphics={
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
          points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}})));
end OTC_direct;
