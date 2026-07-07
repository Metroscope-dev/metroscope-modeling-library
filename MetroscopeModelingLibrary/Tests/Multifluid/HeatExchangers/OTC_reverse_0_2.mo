within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model OTC_reverse_0_2

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source(
    Q_0=123.75327,
    h_0=762243.5,
    h_out(start=762243.5))
    annotation (Placement(transformation(extent={{-216,-20},{-176,20}})));
  MultiFluid.HeatExchangers.Superheater
              superheater(S_parameter=true)
    annotation (Placement(transformation(extent={{-70,50},{30,-50}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink
    annotation (Placement(transformation(extent={{76,-20},{116,20}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source(Q_0=
        6.781402, h_0=1405647.4)
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={96,80})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(
        extent={{20,-20},{-20,20}},
        rotation=0,
        origin={-196,-80})));
  Utilities.Interfaces.RealOutput Kth annotation (Placement(transformation(
          extent={{-62,36},{-54,44}}), iconTransformation(extent={{-292,68},{-272,
            88}})));
  Utilities.Interfaces.RealOutput Kfr_cold annotation (Placement(transformation(
          extent={{-62,-44},{-54,-36}}), iconTransformation(extent={{-282,54},{-262,
            74}})));
  Sensors_Control.WaterSteam.FlowSensor Q_cold_sensor(Q_start=6.781402)
    annotation (Placement(transformation(extent={{40,70},{20,90}})));
  Sensors_Control.WaterSteam.PressureSensor P_cold_in_sensor(sensor_function=
        "BC", P_start=130)
    annotation (Placement(transformation(extent={{70,70},{50,90}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_in_sensor(
    sensor_function="Calibration",
    causality="Kth",
    T_start=319.64444)
    annotation (Placement(transformation(extent={{10,70},{-10,90}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_out_sensor(
      sensor_function="BC", T_start=460.0513)
    annotation (Placement(transformation(extent={{-30,-90},{-50,-70}})));
  Sensors_Control.WaterSteam.PressureSensor P_cold_out_sensor(
    sensor_function="Calibration",
    causality="Kfr_cold",
    P_start=129.5)
    annotation (Placement(transformation(extent={{-60,-90},{-80,-70}})));
  Sensors_Control.FlueGases.PressureSensor P_hot_in_sensor(sensor_function="BC",
      P_start=1.1)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Sensors_Control.FlueGases.FlowSensor Q_hot_sensor(sensor_function="BC",
      Q_start=123.75327)
    annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
  Sensors_Control.FlueGases.TemperatureSensor T_hot_in_sensor(sensor_function=
        "BC", T_start=463.05118)
    annotation (Placement(transformation(extent={{-170,-10},{-150,10}})));
  Sensors_Control.FlueGases.PressureSensor P_hot_out_sensor(
    sensor_function="Calibration",
    causality="Kfr_hot",
    P_start=1) annotation (Placement(transformation(extent={{50,-10},{70,10}})));
  Sensors_Control.FlueGases.TemperatureSensor T_hot_out_sensor(sensor_function=
        "BC", T_start=435.55753)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Utilities.Interfaces.BoundaryCondition T_hot_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={30,20}), iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.CalibrationInput T_cold_in annotation (Placement(
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
        origin={30,100}), iconTransformation(extent={{-102,84},{-82,104}})));
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
  MetroscopeModelingLibrary.FlueGases.Pipes.FrictionPipe Kfr_hot_pipe
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Utilities.Interfaces.RealOutput Kfr_hot annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-70,20}), iconTransformation(extent={{-292,68},{-272,88}})));
  Utilities.Interfaces.CalibrationInput P_hot_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={60,20}), iconTransformation(extent={{-226,4},{-186,44}})));
  Utilities.Interfaces.BoundaryCondition P_cold_in annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={60,100}), iconTransformation(extent={{-376,26},{-336,66}})));
  Utilities.Interfaces.CalibrationInput P_cold_out annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-70,-60}), iconTransformation(extent={{-226,4},{-186,44}})));
  Utilities.Interfaces.BoundaryCondition P_cold_in1 annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={0,100}), iconTransformation(extent={{-376,26},{-336,66}})));
equation

  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};

  connect(Kth, superheater.Kth)
    annotation (Line(points={{-58,40},{-42,40}}, color={0,0,127}));
  connect(Kfr_cold, superheater.Kfr_cold)
    annotation (Line(points={{-58,-40},{-42,-40}}, color={0,0,127}));
  connect(cold_source.C_out, P_cold_in_sensor.C_in)
    annotation (Line(points={{86,80},{70,80}}, color={28,108,200}));
  connect(P_cold_in_sensor.C_out, Q_cold_sensor.C_in)
    annotation (Line(points={{50,80},{40,80}}, color={28,108,200}));
  connect(Q_cold_sensor.C_out, T_cold_in_sensor.C_in)
    annotation (Line(points={{20,80},{10,80}}, color={28,108,200}));
  connect(T_cold_in_sensor.C_out, superheater.C_cold_in)
    annotation (Line(points={{-10,80},{-20,80},{-20,50}}, color={28,108,200}));
  connect(T_cold_out_sensor.C_in, superheater.C_cold_out) annotation (Line(
        points={{-30,-80},{-20,-80},{-20,-50}}, color={28,108,200}));
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
    annotation (Line(points={{70,0},{86,0}}, color={95,95,95}));
  connect(superheater.C_hot_out, T_hot_out_sensor.C_in)
    annotation (Line(points={{0,0},{20,0}}, color={95,95,95}));
  connect(T_hot_out_sensor.C_out, P_hot_out_sensor.C_in)
    annotation (Line(points={{40,0},{50,0}}, color={95,95,95}));
  connect(T_hot_out_sensor.T_sensor, T_hot_out)
    annotation (Line(points={{30,10},{30,20}}, color={0,0,127}));
  connect(T_cold_out_sensor.T_sensor, T_cold_in)
    annotation (Line(points={{-40,-70},{-40,-60}}, color={0,0,127}));
  connect(Q_hot_sensor.Q_sensor, Q_hot)
    annotation (Line(points={{-130,10},{-130,20}}, color={0,0,127}));
  connect(Q_cold_sensor.Q_sensor, Q_cold)
    annotation (Line(points={{30,90},{30,100}}, color={0,0,127}));
  connect(T_hot_in_sensor.T_sensor, T_hot_in)
    annotation (Line(points={{-160,10},{-160,20}}, color={0,0,127}));
  connect(P_hot_in, P_hot_in_sensor.P_sensor)
    annotation (Line(points={{-100,20},{-100,10}}, color={28,108,200}));
  connect(P_hot_in_sensor.C_out, Kfr_hot_pipe.C_in)
    annotation (Line(points={{-90,0},{-80,0}}, color={95,95,95}));
  connect(Kfr_hot_pipe.C_out, superheater.C_hot_in)
    annotation (Line(points={{-60,0},{-40,0}}, color={95,95,95}));
  connect(Kfr_hot_pipe.Kfr, Kfr_hot)
    annotation (Line(points={{-70,4},{-70,20}}, color={0,0,127}));
  connect(P_hot_out_sensor.P_sensor, P_hot_out)
    annotation (Line(points={{60,10},{60,20}}, color={0,0,127}));
  connect(P_cold_in_sensor.P_sensor, P_cold_in)
    annotation (Line(points={{60,90},{60,100}}, color={0,0,127}));
  connect(P_cold_out_sensor.P_sensor, P_cold_out)
    annotation (Line(points={{-70,-70},{-70,-60}}, color={0,0,127}));
  connect(P_cold_in1, T_cold_in_sensor.T_sensor)
    annotation (Line(points={{0,100},{0,90}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-220,
            -100},{120,120}}),                                  graphics={
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
          points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}), Diagram(coordinateSystem(preserveAspectRatio=false, extent={
            {-220,-100},{120,120}})));
end OTC_reverse_0_2;
