within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model CoolingTower_reverse
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary Conditions
    // Hot Water Inlet
  input Real waterInletTemp(start=45) "deg_C";
  input Units.VolumeFlowRate waterFlow(start=39) "m3/s";
  input Real waterInletPress(start=1) "bar";

    // Cold Air Inlet
  input Real airInletPress(start=1) "bar";
  input Real AirInletTemp(start=20) "deg_C";
  input Units.Fraction cold_source_relative_humidity(start=0.8) "1";

  // Input for calibration
  input Real AirOutletTemp(start=35) "deg_C";


  // Calibrated Parameters
  output Real hd(start = 0.00943308);
  parameter Real Kfr = 0;

  // Parameters
  parameter Real Lfi = 15 "m";
  parameter Real afi = 200 "m-1";
  parameter Real Afr = 3000 "m2";
  output Real V_inlet(start = 13.251477) "m/s";

  // Observables
  output Real airFlow(start=440) "m3/s";
  output Real Q_makeup(start=1311.1932);
  output Real Q_cold(start=52552.133);

  output Real WaterOutletTemp(start=25) "deg_C";
  output Real airOutletPress(start=1);

  // Output
  output Units.Fraction cold_sink_relative_humidity(start=1) "1";

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-120,-30},{-100,-10}})));

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{66,-30},{86,-10}})));
  MultiFluid.HeatExchangers.CoolingTower3                           CoolingTower annotation (Placement(transformation(extent={{-12,-30},{8,-10}})));
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
  MetroscopeModelingLibrary.Sensors.MoistAir.FlowSensor airFlow_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,36})));
  MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor airInletPress_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,12})));
  MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor airOutletPress_sensor
                                                                                 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-44})));
equation
  // Hot Water Inlet
  waterFlow_sensor.Qv = waterFlow;
  waterInletTemp_sensor.T_degC = waterInletTemp;
  waterInletPress_sensor.P_barA = waterInletPress;

  // Cold Air Inlet
  airInletPress_sensor.P_barA = airInletPress;
  cold_source.relative_humidity = cold_source_relative_humidity;
  airFlow_sensor.Qv = airFlow;
  AirInletTemp_sensor.T_degC = AirInletTemp;

  // Hot Water Outlet

  // Cold Air Outlet
  cold_sink.relative_humidity = cold_sink_relative_humidity;
  airOutletPress_sensor.P_barA = airOutletPress;

  // Calibrated Parameters
  CoolingTower.hd = hd;
  CoolingTower.Kfr = Kfr;

  CoolingTower.Q_makeup = Q_makeup;
  CoolingTower.Q_cold = Q_cold;

  // Parameters
  CoolingTower.Lfi = Lfi;
  CoolingTower.afi = afi;
  CoolingTower.Afr = Afr;
  CoolingTower.V_inlet = V_inlet;

  // Observable for Calibration
  WaterOutletTemp_sensor.T_degC = WaterOutletTemp;
  AirOutletTemp_sensor.T_degC = AirOutletTemp;

  connect(CoolingTower.C_hot_in, waterInletPress_sensor.C_out) annotation (Line(points={{-9.5,-20},{-16,-20}},
                                                                                                         color={28,108,200}));
  connect(AirInletTemp_sensor.C_in, cold_source.C_out) annotation (Line(points={{1.77636e-15,76},{1.77636e-15,80.5},{-8.88178e-16,80.5},{-8.88178e-16,85}}, color={85,170,255}));
  connect(waterInletPress_sensor.C_in,waterInletTemp_sensor. C_out) annotation (Line(points={{-36,-20},{-48,-20}},
                                                                                                               color={28,108,200}));
  connect(CoolingTower.C_hot_out, WaterOutletTemp_sensor.C_in) annotation (Line(points={{5.5,-20},{34,-20}},
                                                                                                       color={28,108,200}));
  connect(WaterOutletTemp_sensor.C_out, hot_sink.C_in) annotation (Line(points={{54,-20},{71,-20}},
                                                                                                color={28,108,200}));
  connect(hot_source.C_out, waterFlow_sensor.C_in) annotation (Line(points={{-105,-20},{-98,-20}},
                                                                                              color={28,108,200}));
  connect(waterInletTemp_sensor.C_in, waterFlow_sensor.C_out) annotation (Line(points={{-68,-20},{-78,-20}},
                                                                                                         color={28,108,200}));
  connect(AirOutletTemp_sensor.C_out, cold_sink.C_in) annotation (Line(points={{-1.77636e-15,-80},{-1.77636e-15,-83.5},{8.88178e-16,-83.5},{8.88178e-16,-87}}, color={85,170,255}));
  connect(AirInletTemp_sensor.C_out, airFlow_sensor.C_in) annotation (Line(points={{-1.77636e-15,56},{-1.77636e-15,52},{1.77636e-15,52},{1.77636e-15,46}}, color={85,170,255}));
  connect(airFlow_sensor.C_out, airInletPress_sensor.C_in) annotation (Line(points={{0,26},{0,22}},                                                         color={85,170,255}));
  connect(airInletPress_sensor.C_out, CoolingTower.C_cold_in) annotation (Line(points={{0,2},{-1,2},{-1,-12.5},{-2,-12.5}},                    color={85,170,255}));
  connect(CoolingTower.C_cold_out, airOutletPress_sensor.C_in) annotation (Line(points={{-2,-27.5},{-2,-31.5},{0,-31.5},{0,-34}},                 color={85,170,255}));
  connect(AirOutletTemp_sensor.C_in, airOutletPress_sensor.C_out) annotation (Line(points={{1.77636e-15,-60},{1.77636e-15,-57},{-1.77636e-15,-57},{-1.77636e-15,-54}}, color={85,170,255}));
  annotation (Diagram(coordinateSystem(extent={{-120,-100},{100,100}})), Icon(coordinateSystem(extent={{-120,-100},{100,100}})));
end CoolingTower_reverse;
