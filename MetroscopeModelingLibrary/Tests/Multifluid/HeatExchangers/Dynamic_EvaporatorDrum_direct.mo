within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Dynamic_EvaporatorDrum_direct
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;

  MultiFluid.HeatExchangers.Dynamic_Evaporator_Drum Drum(A_dc=0.4, l_0=1.25)
                                                         annotation (Placement(transformation(extent={{-50,-120},{50,90}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source feedwater_source annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={180,10})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink drain_sink annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={-160,10})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink steam_sink annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={-162,100})));
  Sensors_Control.FlueGases.TemperatureSensor temperatureSensor(sensor_function="BC", T_start=400) annotation (Placement(transformation(extent={{-158,-80},{-138,-60}})));
  Sensors_Control.FlueGases.PressureSensor pressureSensor(sensor_function="BC", P_start=1.1) annotation (Placement(transformation(extent={{-124,-80},{-104,-60}})));
  Sensors_Control.FlueGases.FlowSensor flowSensor(sensor_function="BC", Q_start=650) annotation (Placement(transformation(extent={{-86,-80},{-66,-60}})));
  Sensors_Control.WaterSteam.TemperatureSensor temperatureSensor1(sensor_function="BC", T_start=241) annotation (Placement(transformation(extent={{160,0},{140,20}})));
  Sensors_Control.WaterSteam.PressureSensor pressureSensor1 annotation (Placement(transformation(extent={{130,0},{110,20}})));
  Sensors_Control.WaterSteam.FlowSensor flowSensor1(sensor_function="BC", Q_start=50) annotation (Placement(transformation(extent={{100,0},{80,20}})));
  Sensors_Control.WaterSteam.FlowSensor flowSensor2 annotation (Placement(transformation(extent={{-80,0},{-100,20}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source fg_source annotation (Placement(transformation(extent={{-202,-90},{-162,-50}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink fg_sink annotation (Placement(transformation(extent={{134,-90},{174,-50}})));
  Utilities.Interfaces.BoundaryCondition P_fg annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-114,-38}), iconTransformation(extent={{-458,-46},{-418,-6}})));
  Utilities.Interfaces.BoundaryCondition Q_fg annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-76,-38}), iconTransformation(extent={{-458,-46},{-418,-6}})));
  Utilities.Interfaces.BoundaryCondition T_fw annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={150,40}), iconTransformation(extent={{-458,-46},{-418,-6}})));
  Utilities.Interfaces.BoundaryCondition Q_fw annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={90,40}), iconTransformation(extent={{-458,-46},{-418,-6}})));
  Utilities.Interfaces.Observable P_fw annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={120,40}), iconTransformation(extent={{-458,-46},{-418,-6}})));
  Utilities.Interfaces.RealInput Kth(start=46) annotation (Placement(transformation(extent={{-66,-114},{-58,-106}}), iconTransformation(extent={{-356,-46},{-316,-6}})));
  Utilities.Interfaces.Observable level annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={74,100}), iconTransformation(extent={{-306,-2},{-286,18}})));
  Utilities.Interfaces.RealExpression realExpression(y=1e-6)
                                                          annotation (Placement(transformation(extent={{-100,30},{-80,50}})));

  Modelica.Blocks.Sources.Ramp T_fg_in1(
    height=-20,
    duration=60,
    offset=400,
    startTime=300) annotation (Placement(transformation(extent={{-184,-40},{-164,-20}})));
initial equation
  der(Drum.p) = 0;
  der(Drum.V_wt) = 0;

equation

  fg_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  Drum.l = 1.25; // constant level control


  connect(Drum.steam_out, steam_sink.C_in) annotation (Line(points={{-30,80},{-30,100},{-152,100}}, color={28,108,200}));
  connect(temperatureSensor.C_out, pressureSensor.C_in) annotation (Line(points={{-138,-70},{-124,-70}}, color={95,95,95}));
  connect(pressureSensor.C_out, flowSensor.C_in) annotation (Line(points={{-104,-70},{-86,-70}}, color={95,95,95}));
  connect(Drum.fg_inlet, flowSensor.C_out) annotation (Line(points={{-40,-70},{-66,-70}}, color={95,95,95}));
  connect(temperatureSensor1.C_in, feedwater_source.C_out) annotation (Line(points={{160,10},{170,10}}, color={28,108,200}));
  connect(pressureSensor1.C_in, temperatureSensor1.C_out) annotation (Line(points={{130,10},{140,10}}, color={28,108,200}));
  connect(Drum.fw_in, flowSensor1.C_out) annotation (Line(points={{41,10},{80,10}}, color={28,108,200}));
  connect(flowSensor1.C_in, pressureSensor1.C_out) annotation (Line(points={{100,10},{110,10}}, color={28,108,200}));
  connect(Drum.water_out, flowSensor2.C_in) annotation (Line(points={{-40,10},{-80,10}}, color={28,108,200}));
  connect(flowSensor2.C_out, drain_sink.C_in) annotation (Line(points={{-100,10},{-150,10}}, color={28,108,200}));
  connect(temperatureSensor.C_in, fg_source.C_out) annotation (Line(points={{-158,-70},{-172,-70}}, color={95,95,95}));
  connect(Drum.fg_outlet, fg_sink.C_in) annotation (Line(points={{40,-70},{144,-70}}, color={95,95,95}));
  connect(pressureSensor.P_sensor, P_fg) annotation (Line(points={{-114,-60},{-114,-38}}, color={0,0,127}));
  connect(flowSensor.Q_sensor, Q_fg) annotation (Line(points={{-76,-60},{-76,-38}}, color={0,0,127}));
  connect(temperatureSensor1.T_sensor, T_fw) annotation (Line(points={{150,20},{150,40}}, color={0,0,127}));
  connect(flowSensor1.Q_sensor, Q_fw) annotation (Line(points={{90,20},{90,40}}, color={0,0,127}));
  connect(pressureSensor1.P_sensor, P_fw) annotation (Line(points={{120,20},{120,40}}, color={0,0,127}));
  connect(Drum.K_conv_fg, Kth) annotation (Line(points={{-40,-110},{-62,-110}}, color={0,0,127}));
  connect(Drum.drum_level, level) annotation (Line(points={{52,40},{68,40},{68,100},{74,100}}, color={0,0,127}));
  connect(flowSensor2.Q_sensor, realExpression.y) annotation (Line(points={{-90,20},{-90,35}}, color={0,0,127}));
  connect(T_fg_in1.y, temperatureSensor.T_sensor) annotation (Line(points={{-163,-30},{-148,-30},{-148,-60}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-160},{200,160}})));
end Dynamic_EvaporatorDrum_direct;
