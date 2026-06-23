within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model Desuperheating_direct

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-142,-9.99996},{-122,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  MetroscopeModelingLibrary.WaterSteam.Pipes.Desuperheating pipe(Gain=1000)
                                                                annotation (Placement(transformation(extent={{-16.5,-16.3333},{16.5,16.3333}})));

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source water_injection annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-12,72})));
  Sensors_Control.WaterSteam.TemperatureSensor steam_t_sensor(T_start=575, signal_unit="degC") annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  Sensors_Control.WaterSteam.PressureSensor steam_p_sensor(P_start=120, signal_unit="barA") annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
  Sensors_Control.WaterSteam.FlowSensor steam_q_sensor(Q_start=80, signal_unit="kg/s") annotation (Placement(transformation(extent={{-108,-10},{-88,10}})));
  Utilities.Interfaces.BoundaryCondition Steam_Q annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-100,40}), iconTransformation(extent={{-290,-56},{-250,-16}})));
  Utilities.Interfaces.BoundaryCondition Steam_P annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-72,30}), iconTransformation(extent={{-290,-56},{-250,-16}})));
  Utilities.Interfaces.BoundaryCondition Steam_T annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-42,36}), iconTransformation(extent={{-290,-56},{-250,-16}})));
  Utilities.Interfaces.RealExpression Steam_T_setPoint(y=560) annotation (Placement(transformation(extent={{38,42},{58,62}})));
  Sensors_Control.WaterSteam.TemperatureSensor Water_T_sensor(T_start=80, signal_unit="degC") annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-12,24})));
  Utilities.Interfaces.BoundaryCondition Water_T annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={32,42}), iconTransformation(extent={{-290,-56},{-250,-16}})));
  Sensors_Control.WaterSteam.PressureSensor Water_p_sensor(P_start=30, signal_unit="barA") annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-12,52})));
  Utilities.Interfaces.BoundaryCondition Water_P annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={14,78}), iconTransformation(extent={{-290,-56},{-250,-16}})));
  Sensors_Control.WaterSteam.TemperatureSensor steam_t_sensor1(T_start=575, signal_unit="degC")
                                                                                               annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=180,
        origin={64,0})));
equation


  connect(steam_t_sensor.C_out, pipe.C_in) annotation (Line(points={{-30,0},{-16.5,0}}, color={28,108,200}));
  connect(steam_p_sensor.C_out, steam_t_sensor.C_in) annotation (Line(points={{-58,0},{-50,0}}, color={28,108,200}));
  connect(steam_q_sensor.C_out, steam_p_sensor.C_in) annotation (Line(points={{-88,0},{-78,0}}, color={28,108,200}));
  connect(source.C_out, steam_q_sensor.C_in) annotation (Line(points={{-127,2e-05},{-118,2e-05},{-118,0},{-108,0}}, color={28,108,200}));
  connect(steam_q_sensor.Q_sensor, Steam_Q) annotation (Line(points={{-98,10},{-98,40},{-100,40}}, color={0,0,127}));
  connect(Steam_P, steam_p_sensor.P_sensor) annotation (Line(points={{-72,30},{-72,14},{-68,14},{-68,10}}, color={28,108,200}));
  connect(steam_t_sensor.T_sensor, Steam_T) annotation (Line(points={{-40,10},{-40,28},{-42,28},{-42,36}}, color={0,0,127}));
  connect(Steam_T_setPoint.y, pipe.T_setpoint) annotation (Line(points={{48,47},{48,6.53332},{11.22,6.53332}}, color={0,0,127}));
  connect(Water_T_sensor.T_sensor, Water_T) annotation (Line(points={{-2,24},{32,24},{32,42}}, color={0,0,127}));
  connect(pipe.Water_in, Water_T_sensor.C_out) annotation (Line(points={{-11.55,4.89999},{-11.55,9.44999},{-12,9.44999},{-12,14}}, color={28,108,200}));
  connect(Water_P, Water_p_sensor.P_sensor) annotation (Line(points={{14,78},{14,52},{-2,52}}, color={28,108,200}));
  connect(Water_T_sensor.C_in, Water_p_sensor.C_out) annotation (Line(points={{-12,34},{-11.55,41.6665},{-12,41.6665},{-12,42}}, color={28,108,200}));
  connect(Water_p_sensor.C_in, water_injection.C_out) annotation (Line(points={{-12,62},{-12,67}}, color={28,108,200}));
  connect(sink.C_in, steam_t_sensor1.C_out) annotation (Line(points={{85,0},{74,0}}, color={28,108,200}));
  connect(steam_t_sensor1.C_in, pipe.C_out) annotation (Line(points={{54,0},{16.5,0}}, color={28,108,200}));
  connect(pipe.T_measured, steam_t_sensor1.T_sensor) annotation (Line(points={{11.55,-6.53332},{11.55,-26},{64,-26},{64,-10}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-160,-100},{100,100}})), Icon(coordinateSystem(extent={{-160,-100},{100,100}})));
end Desuperheating_direct;
