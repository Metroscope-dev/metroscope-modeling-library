within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Dynamic_Condenser_direct
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Dynamic_Condenser dynamic_Condenser annotation (Placement(transformation(extent={{-70,-30},{30,70}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-132,80},{-92,120}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-20,-120})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={100,-84})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={100,108})));
  Utilities.Interfaces.RealInput UA(start=2.68e7) annotation (Placement(transformation(extent={{-104,26},{-96,34}}), iconTransformation(extent={{-300,18},{-260,58}})));
  Utilities.Interfaces.RealInput Qv_cold(start=7.5) annotation (Placement(transformation(extent={{-104,6},{-96,14}}), iconTransformation(extent={{-300,18},{-260,58}})));
  Sensors_Control.WaterSteam.FlowSensor Q_steam_sensor(sensor_function="BC", Q_start=102) annotation (Placement(transformation(extent={{-50,90},{-30,110}})));
  Sensors_Control.WaterSteam.PressureSensor P_consender_sensor annotation (Placement(transformation(extent={{-80,90},{-60,110}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_sensor(sensor_function="BC", T_start=20) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={100,-50})));
  Sensors_Control.WaterSteam.PressureSensor P_cold_sensor(sensor_function="BC", P_start=5) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={100,-20})));
  Utilities.Interfaces.BoundaryCondition Q_steam annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-40,120}), iconTransformation(extent={{-300,18},{-260,58}})));
  Utilities.Interfaces.Observable P_condenser annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-70,120}), iconTransformation(extent={{-322,34},{-302,54}})));
  Utilities.Interfaces.BoundaryCondition P_cold annotation (Placement(transformation(extent={{56,-24},{64,-16}}), iconTransformation(extent={{-318,-12},{-278,28}})));
  Utilities.Interfaces.BoundaryCondition T_cold annotation (Placement(transformation(extent={{56,-54},{64,-46}}), iconTransformation(extent={{-308,-32},{-268,8}})));
equation
  hot_source.h_out = 2465407.5;

  connect(dynamic_Condenser.C_hot_out, hot_sink.C_in) annotation (Line(
      points={{-20,-40},{-20,-110}},
      color={28,108,200},
      thickness=1));
  connect(dynamic_Condenser.C_cold_out, cold_sink.C_in) annotation (Line(
      points={{30,20},{100,20},{100,98}},
      color={28,108,200},
      thickness=1));
  connect(dynamic_Condenser.UA, UA) annotation (Line(points={{-70,30},{-100,30}}, color={0,0,127}));
  connect(dynamic_Condenser.Qv_cold, Qv_cold) annotation (Line(points={{-70,10},{-100,10}}, color={0,0,127}));
  connect(Q_steam_sensor.C_out, dynamic_Condenser.C_hot_in) annotation (Line(
      points={{-30,100},{-20,100},{-20,60}},
      color={28,108,200},
      thickness=1));
  connect(hot_source.C_out, P_consender_sensor.C_in) annotation (Line(
      points={{-102,100},{-80,100}},
      color={28,108,200},
      thickness=1));
  connect(P_consender_sensor.C_out, Q_steam_sensor.C_in) annotation (Line(
      points={{-60,100},{-50,100}},
      color={28,108,200},
      thickness=1));
  connect(T_cold_sensor.C_in, cold_source.C_out) annotation (Line(
      points={{100,-60},{100,-74}},
      color={28,108,200},
      thickness=1));
  connect(dynamic_Condenser.C_cold_in, P_cold_sensor.C_out) annotation (Line(
      points={{30,0},{100,0},{100,-10}},
      color={28,108,200},
      thickness=1));
  connect(P_cold_sensor.C_in, T_cold_sensor.C_out) annotation (Line(
      points={{100,-30},{100,-40}},
      color={28,108,200},
      thickness=1));
  connect(Q_steam_sensor.Q_sensor, Q_steam) annotation (Line(points={{-40,110},{-40,120}}, color={0,0,127}));
  connect(P_consender_sensor.P_sensor, P_condenser) annotation (Line(points={{-70,110},{-70,120}}, color={0,0,127}));
  connect(P_cold_sensor.P_sensor, P_cold) annotation (Line(points={{90,-20},{60,-20}}, color={0,0,127}));
  connect(T_cold_sensor.T_sensor, T_cold) annotation (Line(points={{90,-50},{60,-50}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
        Ellipse(lineColor={28,108,200},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor={0,0,255},
                fillColor={28,108,200},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}),                                  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},{140,140}})));
end Dynamic_Condenser_direct;
