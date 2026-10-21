within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Reheater_reverse
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater reheater(S=1000)
    annotation (Placement(transformation(extent={{-44,-24},{52,24}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor
    pressureSensor(
    P_start=65,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-84,-10},{-64,10}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor
    temperatureSensor(
    T_start=100,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-120,-10},{-100,10}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor flowSensor(
    Q_start=1000,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-154,-10},{-134,10}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor
    temperatureSensor1
    annotation (Placement(transformation(extent={{118,-10},{138,10}})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor flowSensor1(
      Q_start=95) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={44,84})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor
    pressureSensor1(
    P_start=35,
    signal_unit="barA",
    display_unit="barA") annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={46,116})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor
    temperatureSensor2(T_start=224)
    annotation (Placement(transformation(extent={{30,-70},{50,-50}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-196,-20},{-156,20}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{84,-80},{124,-40}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink1
    annotation (Placement(transformation(extent={{198,-20},{238,20}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source1
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={46,178})));
  Utilities.Interfaces.GenericReal                              kth_cond
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-10,-58})));
  Utilities.Interfaces.RealOutput                               kth_subc
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-28,62})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealExpression kfr(y=0)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-56,50})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor
    temperatureSensor3(
    T_start=280,
    signal_unit="degC",
    display_unit="degC") annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={46,150})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealExpression realExpression(
      y=100) annotation (Placement(transformation(extent={{-120,20},{-100,40}})));
  Utilities.Interfaces.BoundaryCondition feedwater_flow annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-140,38}), iconTransformation(extent={{-284,-108},{-244,-68}})));
  Utilities.Interfaces.BoundaryCondition feedwater_temp annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-80,42}), iconTransformation(extent={{-284,-108},{-244,-68}})));
  Utilities.Interfaces.Observable feedwater_temp1 annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={128,36}), iconTransformation(extent={{-284,-108},{-244,-68}})));
  Utilities.Interfaces.CalibrationInput steam_flow annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-6,92}), iconTransformation(extent={{-284,-108},{-244,-68}})));
  Utilities.Interfaces.BoundaryCondition steam_pressure annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={0,122}), iconTransformation(extent={{-284,-108},{-244,-68}})));
  Utilities.Interfaces.BoundaryCondition steam_flow1 annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-2,154}), iconTransformation(extent={{-284,-108},{-244,-68}})));
  Utilities.Interfaces.CalibrationInput drain_temp annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={72,-24}), iconTransformation(extent={{-284,-108},{-244,-68}})));
equation

  connect(reheater.C_cold_in, pressureSensor.C_out)
    annotation (Line(points={{-44.6,0},{-64,0}}, color={28,108,200}));
  connect(temperatureSensor.C_out, pressureSensor.C_in)
    annotation (Line(points={{-100,0},{-84,0}},color={28,108,200}));
  connect(reheater.C_cold_out, temperatureSensor1.C_in)
    annotation (Line(points={{52,0},{118,0}}, color={28,108,200}));
  connect(temperatureSensor2.C_in, reheater.C_hot_out) annotation (Line(points={
          {30,-60},{10,-60},{10,-54},{4,-54},{4,-24}}, color={28,108,200}));
  connect(flowSensor.C_in, source.C_out)
    annotation (Line(points={{-154,0},{-166,0}}, color={28,108,200}));
  connect(temperatureSensor2.C_out, sink.C_in)
    annotation (Line(points={{50,-60},{94,-60}}, color={28,108,200}));
  connect(temperatureSensor1.C_out, sink1.C_in)
    annotation (Line(points={{138,0},{208,0}}, color={28,108,200}));
  connect(reheater.C_hot_in, flowSensor1.C_out) annotation (Line(points={{4,24},
          {4,68},{44,68},{44,74}}, color={28,108,200}));
  connect(flowSensor1.C_in, pressureSensor1.C_out)
    annotation (Line(points={{44,94},{44,106},{46,106}},
                                                color={28,108,200}));
  connect(kfr.y, reheater.Kfr_cold)
    annotation (Line(points={{-56,45},{-56,12},{-45.2,12}}, color={0,0,127}));

  connect(source1.C_out, temperatureSensor3.C_in)
    annotation (Line(points={{46,168},{46,160}}, color={28,108,200}));
  connect(temperatureSensor3.C_out, pressureSensor1.C_in)
    annotation (Line(points={{46,140},{46,126}}, color={28,108,200}));
  connect(temperatureSensor.C_in, flowSensor.C_out)
    annotation (Line(points={{-120,0},{-134,0}}, color={28,108,200}));
  connect(realExpression.y, temperatureSensor.T_sensor) annotation (Line(points={{-110,25},
          {-110,10}},                             color={0,0,127}));
  connect(flowSensor.Q_sensor, feedwater_flow) annotation (Line(points={{-144,
          10},{-144,30},{-140,30},{-140,38}}, color={0,0,127}));
  connect(pressureSensor.P_sensor, feedwater_temp) annotation (Line(points={{
          -74,10},{-74,34},{-80,34},{-80,42}}, color={0,0,127}));
  connect(temperatureSensor1.T_sensor, feedwater_temp1)
    annotation (Line(points={{128,10},{128,36}}, color={0,0,127}));
  connect(flowSensor1.Q_sensor, steam_flow)
    annotation (Line(points={{34,84},{-6,84},{-6,92}}, color={0,0,127}));
  connect(steam_pressure, pressureSensor1.P_sensor) annotation (Line(points={{0,
          122},{30,122},{30,116},{36,116}}, color={28,108,200}));
  connect(temperatureSensor3.T_sensor, steam_flow1) annotation (Line(points={{
          36,150},{8,150},{8,154},{-2,154}}, color={0,0,127}));
  connect(temperatureSensor2.T_sensor, drain_temp) annotation (Line(points={{40,
          -50},{40,-34},{72,-34},{72,-24}}, color={0,0,127}));
  connect(kth_cond, kth_cond)
    annotation (Line(points={{-10,-58},{-10,-58}}, color={0,0,127}));
  connect(kth_subc, kth_subc)
    annotation (Line(points={{-28,62},{-28,62}}, color={0,0,127}));
  connect(reheater.Kth_cond, kth_cond) annotation (Line(points={{-26,-25.2},{
          -26,-58},{-10,-58}}, color={0,0,127}));
  connect(reheater.Kth_subc, kth_subc) annotation (Line(points={{-26,25.2},{-26,
          46},{-28,46},{-28,62}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            100}}), graphics={
        Ellipse(lineColor={28,108,200},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor={0,0,255},
                fillColor={28,108,200},
                pattern=LinePattern.None,
                fillPattern=FillPattern.Solid,
                points={{-34,60},{66,0},{-34,-60},{-34,60}})}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,100}})));

end Reheater_reverse;
