within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model CoolingTower_direct
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
  output Real AirOutletTemp(start=35) "deg_C";
  output Real WaterOutletTemp(start=25) "deg_C";

  // Calibrated Parameters
  parameter Real hd = 0.023336338;
  parameter Real Cf = 10.460426;

  // Parameters
  parameter Real Lfi = 15 "m";
  parameter Real afi = 200 "m-1";
  parameter Real Afr = 3000 "m2";
  parameter Real Vd = 7.5 "m/s";

  // Observables
  output Real airFlow(start=440) "m3/s";
  output Real Q_makeup;

  // Output
  output Units.Fraction cold_sink_relative_humidity(start=1) "1";

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-122,-10},{-102,10}})));

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{64,-10},{84,10}})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.CoolingTower2 CoolingTower annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,90})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-80})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor waterInletPress_sensor annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.TemperatureSensor AirInletTemp_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,66})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor waterInletTemp_sensor annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor WaterOutletTemp_sensor annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor waterFlow_sensor annotation (Placement(transformation(extent={{-102,-10},{-82,10}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.TemperatureSensor AirOutletTemp_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-56})));
  MetroscopeModelingLibrary.Sensors.MoistAir.FlowSensor airFlow_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,36})));
  MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor airInletPress_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,8})));
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

  // Calibrated Parameters
  CoolingTower.hd = hd;
  CoolingTower.Cf = Cf;

  CoolingTower.Q_makeup = Q_makeup;
  //CoolingTower.Q_cold = Q_cold;

  // Parameters
  CoolingTower.Lfi = Lfi;
  CoolingTower.afi = afi;
  CoolingTower.Afr = Afr;
  CoolingTower.Vd = Vd;

  // Observable for Calibration
  WaterOutletTemp_sensor.T_degC = WaterOutletTemp;
  AirOutletTemp_sensor.T_degC = AirOutletTemp;

  connect(CoolingTower.C_hot_in, waterInletPress_sensor.C_out) annotation (Line(points={{-9,-20},{-16,-20},{-16,-14},{-14,-14},{-14,-4},{-12,-4},{-12,0},{-32,0}},
                                                                                                         color={28,108,200}));
  connect(AirInletTemp_sensor.C_in, cold_source.C_out) annotation (Line(points={{1.77636e-15,76},{1.77636e-15,80.5},{-8.88178e-16,80.5},{-8.88178e-16,85}}, color={85,170,255}));
  connect(waterInletPress_sensor.C_in,waterInletTemp_sensor. C_out) annotation (Line(points={{-52,0},{-56,0}}, color={28,108,200}));
  connect(CoolingTower.C_hot_out, WaterOutletTemp_sensor.C_in) annotation (Line(points={{9,-20},{24,-20},{24,0},{30,0}},
                                                                                                       color={28,108,200}));
  connect(WaterOutletTemp_sensor.C_out, hot_sink.C_in) annotation (Line(points={{50,0},{69,0}}, color={28,108,200}));
  connect(hot_source.C_out, waterFlow_sensor.C_in) annotation (Line(points={{-107,0},{-102,0}},
                                                                                              color={28,108,200}));
  connect(waterInletTemp_sensor.C_in, waterFlow_sensor.C_out) annotation (Line(points={{-76,0},{-82,0}}, color={28,108,200}));
  connect(CoolingTower.C_cold_out, AirOutletTemp_sensor.C_in) annotation (Line(points={{0,-29},{0,-37.5},{1.77636e-15,-37.5},{1.77636e-15,-46}}, color={85,170,255}));
  connect(AirOutletTemp_sensor.C_out, cold_sink.C_in) annotation (Line(points={{-1.77636e-15,-66},{-1.77636e-15,-70.5},{8.88178e-16,-70.5},{8.88178e-16,-75}}, color={85,170,255}));
  connect(AirInletTemp_sensor.C_out, airFlow_sensor.C_in) annotation (Line(points={{-1.77636e-15,56},{-1.77636e-15,52},{1.77636e-15,52},{1.77636e-15,46}}, color={85,170,255}));
  connect(airFlow_sensor.C_out, airInletPress_sensor.C_in) annotation (Line(points={{-1.77636e-15,26},{-1.77636e-15,22},{1.77636e-15,22},{1.77636e-15,18}}, color={85,170,255}));
  connect(airInletPress_sensor.C_out, CoolingTower.C_cold_in) annotation (Line(points={{-1.77636e-15,-2},{-1,-2},{-1,-11},{0,-11}},            color={85,170,255}));
  annotation (Diagram(coordinateSystem(extent={{-120,-100},{100,100}})), Icon(coordinateSystem(extent={{-120,-100},{100,100}})));
end CoolingTower_direct;
