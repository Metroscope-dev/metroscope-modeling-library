within MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions;
model GT_louvers
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

  parameter String T_signal_unit = "degC" "Specify the ambient temperature signal unit. This should be the unit of T_start and of the tag linked to the sensor." annotation (choices(choice="degC", choice="K", choice="degF"));
  parameter String P_signal_unit = "barA" "Specify the ambient pressure signal unit. This should be the unit of P_start and of the tag linked to the sensor." annotation(choices(choice="barA", choice="barG", choice="mbar", choice="MPaA", choice="kPaA"));
  parameter String H_signal_unit = "%" "Specify the ambient relative humidity signal unit. This should be the unit of H_start and of the tag linked to the sensor."
                                                                                                                                                                   annotation (choices(choice="1", choice="%"));

  parameter Real T_start = 25 "Write here the build value of the quantity. This value will be used in the simulation.";
  parameter Real P_start = 1 "Write here the build value of the quantity. This value will be used in the simulation.";
  parameter Real H_start = 50 "Write here the build value of the quantity. This value will be used in the simulation.";

  Connectors.Outlet           C_out annotation (Placement(transformation(extent={{92,-10},{112,10}}), iconTransformation(extent={{50,-10},{70,10}})));
  Sensors_Control.RefMoistAir.TemperatureSensor
                                             temperatureSensor(T_start=T_start, signal_unit=T_signal_unit)
                                                               annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  Sensors_Control.RefMoistAir.PressureSensor
                                          pressureSensor(P_start=P_start, signal_unit=P_signal_unit)
                                                         annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Sensors_Control.RefMoistAir.RelativeHumiditySensor
                                                  relativeHumiditySensor(H_start=H_start, signal_unit=H_signal_unit)
                                                                         annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  Utilities.Interfaces.GenericReal T "Ambient temperature" annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-40,100}),
                         iconTransformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-40,100})));
  Utilities.Interfaces.GenericReal P "Ambient pressure" annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={0,100}), iconTransformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={0,100})));
  Utilities.Interfaces.GenericReal H "Ambient relative humidity" annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={40,100}),iconTransformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={40,100})));
  Source                             source(h_0=50000)
                                            annotation (Placement(transformation(extent={{-90,-20},{-50,20}})));
equation
  connect(T, temperatureSensor.T_sensor) annotation (Line(points={{-40,100},{-40,10}}, color={0,0,127}));
  connect(P, pressureSensor.P_sensor) annotation (Line(points={{0,100},{0,10}}, color={0,0,127}));
  connect(H, relativeHumiditySensor.H_sensor) annotation (Line(points={{40,100},{40,10}}, color={0,0,127}));
  connect(C_out, C_out) annotation (Line(points={{102,0},{102,0}}, color={95,95,95}));
  connect(source.C_out, temperatureSensor.C_in) annotation (Line(points={{-60,0},{-50,0}}, color={0,127,127}));
  connect(temperatureSensor.C_out, pressureSensor.C_in) annotation (Line(points={{-30,0},{-10,0}}, color={0,127,127}));
  connect(pressureSensor.C_out, relativeHumiditySensor.C_in) annotation (Line(points={{10,0},{30,0}}, color={0,127,127}));
  connect(relativeHumiditySensor.C_out, C_out) annotation (Line(points={{50,0},{102,0}}, color={0,127,127}));
                                                                                       annotation (IconMap(primitivesVisible=false),
              Icon(coordinateSystem(initialScale=0.4, extent={{-100,-100},{100,100}}), graphics={
                          Polygon(
          points={{60,96},{60,-96},{-90,-96},{-50,-56},{-90,-56},{-50,-16},{-90,-16},{-50,24},{-90,24},{-50,64},{-90,64},{-58,96},{60,96}},
          lineColor={0,127,127},
          fillColor={0,127,127},
          fillPattern=FillPattern.Solid),                                 Line(
          points={{64,100},{64,-100},{-100,-100},{-60,-60},{-100,-60},{-60,-20},{-100,-20},{-60,20},{-100,20},{-60,60},{-100,60},{-60,100},{64,100}},
          color={0,127,127},
          thickness=0.5),
        Line(
          points={{-40,46},{-26,32},{-2,28}},
          color={255,255,255},
          thickness=0.5,
          smooth=Smooth.Bezier,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{-42,2},{-28,-12},{-4,-16}},
          color={255,255,255},
          thickness=0.5,
          smooth=Smooth.Bezier,
          arrow={Arrow.None,Arrow.Filled}),
        Line(
          points={{-42,-40},{-28,-54},{-4,-58}},
          color={255,255,255},
          thickness=0.5,
          smooth=Smooth.Bezier,
          arrow={Arrow.None,Arrow.Filled}),
        Text(
          extent={{-52,91},{-28,69}},
          textColor={0,0,0},
          fontSize=5,
          textString="T"),
        Text(
          extent={{-12,91},{12,69}},
          textColor={0,0,0},
          fontSize=5,
          textString="P"),
        Text(
          extent={{28,91},{52,69}},
          textColor={0,0,0},
          fontSize=5,
          textString="H"),
        Text(
          extent={{-100,-100},{100,-160}},
          textColor={0,0,0},
          fontSize=3,
          textString=DynamicSelect("",String(temperatureSensor.T_degC)+" degC\n"+
          String(pressureSensor.P_barA)+" barA\n"+
          String(relativeHumiditySensor.relative_humidity_pc)+" %%"))}),                                                              Diagram(coordinateSystem(initialScale=0.4, extent={{-100,-100},{100,100}})));

end GT_louvers;
