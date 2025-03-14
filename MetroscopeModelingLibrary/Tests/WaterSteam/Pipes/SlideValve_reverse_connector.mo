within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model SlideValve_reverse_connector
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;


  // Components
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source(h_out(start=1e6)) annotation (Placement(transformation(extent={{-194,-10},{-174,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-6,-6.10623e-16})));

  MetroscopeModelingLibrary.WaterSteam.Pipes.SlideValve_connector
                                                        slide_valve annotation (Placement(transformation(extent={{-78.5,-5.93938},{-45.5,26.7272}})));

  Sensors_Control.WaterSteam.PressureSensor P_out_sensor(init_P=9) annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  Utilities.Interfaces.RealInput P_out annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-28,60})));
  Sensors_Control.WaterSteam.TemperatureSensor T_in_sensor(init_T=180) annotation (Placement(transformation(extent={{-170,-10},{-150,10}})));
  Sensors_Control.WaterSteam.PressureSensor P_in_sensor(init_P=10) annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
  Sensors_Control.WaterSteam.FlowSensor Q_in_sensor(init_Q=100) annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Utilities.Interfaces.RealInput T_in annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-160,60})));
  Utilities.Interfaces.RealInput P_in annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-130,60})));
  Utilities.Interfaces.RealInput Q_in annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-100,60})));
  Utilities.Interfaces.RealOutput Cv annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-62,50})));
equation

  connect(slide_valve.C_out, P_out_sensor.C_in) annotation (Line(points={{-45.5,-1.81818e-06},{-42,-1.81818e-06},{-42,0},{-38,0}}, color={28,108,200}));
  connect(sink.C_in, P_out_sensor.C_out) annotation (Line(points={{-11,0},{-18,0}}, color={28,108,200}));
  connect(P_out, P_out_sensor.P_sensor) annotation (Line(points={{-28,60},{-28,10}}, color={0,0,127}));
  connect(T_in_sensor.C_in, source.C_out) annotation (Line(points={{-170,0},{-179,0}}, color={28,108,200}));
  connect(P_in_sensor.C_in, T_in_sensor.C_out) annotation (Line(points={{-140,0},{-150,0}}, color={28,108,200}));
  connect(slide_valve.C_in, Q_in_sensor.C_out) annotation (Line(points={{-78.5,0},{-90,0}}, color={28,108,200}));
  connect(Q_in_sensor.C_in, P_in_sensor.C_out) annotation (Line(points={{-110,0},{-120,0}}, color={28,108,200}));
  connect(T_in_sensor.T_sensor, T_in) annotation (Line(points={{-160,10},{-160,60}}, color={0,0,127}));
  connect(P_in_sensor.P_sensor, P_in) annotation (Line(points={{-130,10},{-130,60}}, color={0,0,127}));
  connect(Q_in_sensor.Q_sensor, Q_in) annotation (Line(points={{-100,10},{-100,60}}, color={0,0,127}));
  connect(slide_valve.Cv_signal, Cv) annotation (Line(points={{-62,24.0545},{-62,50}}, color={0,0,127}));
  connect(Cv, Cv) annotation (Line(points={{-62,50},{-62,50}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-240,-100},{20,100}})),  Icon(coordinateSystem(extent={{-240,-100},{20,100}})));
end SlideValve_reverse_connector;
