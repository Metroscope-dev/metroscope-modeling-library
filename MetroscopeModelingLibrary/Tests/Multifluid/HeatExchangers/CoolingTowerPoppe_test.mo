within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model CoolingTowerPoppe_test
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary Conditions
    // Hot Water Inlet
  input Real waterInletTemp(start=28) "deg_C";
  input Units.VolumeFlowRate waterInletFlow(start=39) "m3/s";
  input Real waterInletPress(start=1) "bar";

    // Cold Air Inlet
  input Real AirInletTemp(start=6) "deg_C";
  input Real airInletPress(start=1) "bar";
  input Units.Fraction cold_source_relative_humidity(start=0.8) "1";

  // Input for calibration
  input Real WaterOutletTemp(start=20) "deg_C";

  // Calibrated Parameters
  output Real hd;

  // Parameters
  //parameter Real Lfi = 15 "m";
  //parameter Real afi = 200 "m-1";
  //parameter Real Afr = 3000 "m2";
  //parameter Real D = 20 "m";
  //parameter Real Cf = 1;
 // output Real V_inlet(start = 13.251477) "m/s";

  // Observables
  input Real airInletFlow(start=52552.133) "m3/s";

  output Real airOutletPress(start=1) "bar";
  output Real AirOutletTemp(start=35) "deg_C";

  // Output
  output Units.Fraction cold_sink_relative_humidity(start=1) "1";

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-120,-30},{-100,-10}})));

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{66,-30},{86,-10}})));
  MultiFluid.HeatExchangers.CoolingTowerPoppeTrial
                                               CoolingTower annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,90})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-92})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor waterInletPress_sensor annotation (Placement(transformation(extent={{-36,-30},{-16,-10}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.TemperatureSensor AirInletTemp_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,66})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor waterInletTemp_sensor annotation (Placement(transformation(extent={{-68,-30},{-48,-10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor WaterOutletTemp_sensor annotation (Placement(transformation(extent={{34,-30},{54,-10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor waterFlow_sensor annotation (Placement(transformation(extent={{-98,-30},{-78,-10}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.TemperatureSensor AirOutletTemp_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-70})));
  MetroscopeModelingLibrary.Sensors.MoistAir.FlowSensor airInletFlow_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,36})));
  MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor airInletPress_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,10})));
  MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor airOutletPress_sensor
                                                                                 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-44})));
equation
  // Hot Water Inlet
  waterFlow_sensor.Qv = waterInletFlow;
  waterInletTemp_sensor.T_degC = waterInletTemp;
  waterInletPress_sensor.P_barA = waterInletPress;

  // Cold Air Inlet
  airInletPress_sensor.P_barA = airInletPress;
  cold_source.relative_humidity = cold_source_relative_humidity;
  airInletFlow_sensor.Qv = airInletFlow;
  AirInletTemp_sensor.T_degC = AirInletTemp;

  // Hot Water Outlet

  // Cold Air Outlet
  cold_sink.relative_humidity = cold_sink_relative_humidity;
  airOutletPress_sensor.P_barA = airOutletPress;

  // Calibrated Parameters
  CoolingTower.hd = hd;
  //CoolingTower.Cf = Cf;

  // Parameters
  //CoolingTower.Lfi = Lfi;
  //CoolingTower.afi = afi;
  //CoolingTower.Afr = Afr;
  //CoolingTower.D = D;
  //CoolingTower.V_inlet = V_inlet;

  // Observable for Calibration
  WaterOutletTemp_sensor.T_degC = WaterOutletTemp;
  AirOutletTemp_sensor.T_degC = AirOutletTemp;

  connect(AirInletTemp_sensor.C_in, cold_source.C_out) annotation (Line(points={{1.77636e-15,76},{1.77636e-15,80.5},{-8.88178e-16,80.5},{-8.88178e-16,85}}, color={85,170,255}));
  connect(waterInletPress_sensor.C_in,waterInletTemp_sensor. C_out) annotation (Line(points={{-36,-20},{-48,-20}},
                                                                                                               color={28,108,200}));
  connect(WaterOutletTemp_sensor.C_out, hot_sink.C_in) annotation (Line(points={{54,-20},{71,-20}},
                                                                                                color={28,108,200}));
  connect(hot_source.C_out, waterFlow_sensor.C_in) annotation (Line(points={{-105,-20},{-98,-20}},
                                                                                              color={28,108,200}));
  connect(waterInletTemp_sensor.C_in, waterFlow_sensor.C_out) annotation (Line(points={{-68,-20},{-78,-20}},
                                                                                                         color={28,108,200}));
  connect(AirOutletTemp_sensor.C_out, cold_sink.C_in) annotation (Line(points={{-1.77636e-15,-80},{-1.77636e-15,-83.5},{8.88178e-16,-83.5},{8.88178e-16,-87}}, color={85,170,255}));
  connect(AirInletTemp_sensor.C_out, airInletFlow_sensor.C_in) annotation (Line(points={{-1.77636e-15,56},{-1.77636e-15,52},{1.77636e-15,52},{1.77636e-15,46}}, color={85,170,255}));
  connect(airInletFlow_sensor.C_out, airInletPress_sensor.C_in) annotation (Line(points={{0,26},{0,20}}, color={85,170,255}));
  connect(AirOutletTemp_sensor.C_in, airOutletPress_sensor.C_out) annotation (Line(points={{1.77636e-15,-60},{1.77636e-15,-57},{-1.77636e-15,-57},{-1.77636e-15,-54}}, color={85,170,255}));
  connect(waterInletPress_sensor.C_out, CoolingTower.water_inlet_connector) annotation (Line(points={{-16,-20},{-9.16667,-20}}, color={28,108,200}));
  connect(CoolingTower.water_outlet_connector, WaterOutletTemp_sensor.C_in) annotation (Line(points={{9.16667,-20},{34,-20}}, color={28,108,200}));
  connect(CoolingTower.air_outlet_connector, airOutletPress_sensor.C_in) annotation (Line(points={{0,-29.1667},{0,-34}}, color={85,170,255}));
  connect(airInletPress_sensor.C_out, CoolingTower.air_inlet_connector) annotation (Line(points={{0,0},{0,-10.8333}}, color={85,170,255}));
  annotation (Diagram(coordinateSystem(extent={{-120,-100},{100,100}})), Icon(coordinateSystem(extent={{-120,-100},{100,100}}), graphics={
        Ellipse(lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                extent={{-110,-100},{90,100}}),
        Polygon(
          origin={12,14},
          lineColor={78,138,73},
          fillColor={95,95,95},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}}),
        Polygon(
          origin={12,14},
          lineColor={78,138,73},
          fillColor={213,213,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58,46},{-4,14},{-58,-14},{-58,46}}),
        Polygon(
          origin={12,14},
          lineColor={78,138,73},
          fillColor={28,108,200},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}));
end CoolingTowerPoppe_test;
