within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model CoolingTowerMerkel_direct
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary Conditions
    // Hot Water Inlet
  input Real waterInletTemp(start=28) "deg_C";
  input Units.VolumeFlowRate waterFlow(start=40) "m3/s";
  input Real waterInletPress(start=1) "bar";

    // Cold Air Inlet
  input Real AirInletTemp(start=6) "deg_C";
  input Real airInletPress(start=1) "bar";
  input Units.Fraction cold_source_relative_humidity(start=0.8) "1";

  // Observables for calibration
  output Real WaterOutletTemp(start=24.999994) "deg_C";

  // Calibrated Parameters
  parameter Real hd = 0.012856079;
  parameter Real Kfr = 0;

  // Parameters
  parameter Real Lfi = 15 "m";
  parameter Real afi = 200 "m-1";
  parameter Real Afr = 3000 "m2";
  parameter Real D = 20 "m";
  parameter Real Cf = 15;
  output Real V_inlet(start = 4.3490353) "m/s";

  parameter Real eta_fan = 1;
  parameter Real W_fan = 40000 "W";

  // Observables
  output Real airInletFlow(start=12894.166) "kg/s";
  output Real Q_evap(start=379.48428) "kg/s";
  output Real Q_cold_in(start=15214.605);
  output Real Ratio;
  output Real W;
  output Real deltaP_fan;

  output Real AirOutletTemp(start=35) "deg_C";
  output Real airOutletPress(start=1);

  // Output
  output Units.Fraction cold_sink_relative_humidity(start=1) "1";

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-138,-10},{-118,10}})));

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{64,-10},{84,10}})));
  MultiFluid.HeatExchangers.CoolingTowerMerkel CoolingTower annotation (Placement(transformation(extent={{10,-10},{-10,10}}, rotation=180)));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,114})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-92})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor waterInletPress_sensor annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.TemperatureSensor AirInletTemp_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,90})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor waterInletTemp_sensor annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor WaterOutletTemp_sensor annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor waterFlow_sensor annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.TemperatureSensor AirOutletTemp_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-64})));
  MetroscopeModelingLibrary.Sensors.MoistAir.FlowSensor airInletFlow_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,58})));
  MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor airInletPress_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,28})));
  MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor airOutletPress_sensor
                                                                                 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-30})));
equation
  // Hot Water Inlet
  waterFlow_sensor.Qv = waterFlow;
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
  CoolingTower.Cf = Cf;

  CoolingTower.Q_evap = Q_evap;
  CoolingTower.Q_cold_in = Q_cold_in;
  CoolingTower.Ratio = Ratio;
  CoolingTower.W = W;
  CoolingTower.deltaP_fan = deltaP_fan;

  // Parameters
  CoolingTower.Lfi = Lfi;
  CoolingTower.afi = afi;
  CoolingTower.Afr = Afr;
  CoolingTower.V_inlet = V_inlet;

  CoolingTower.eta_fan = eta_fan;
  CoolingTower.W_fan = W_fan;

  // Observable for Calibration
  WaterOutletTemp_sensor.T_degC = WaterOutletTemp;
  AirOutletTemp_sensor.T_degC = AirOutletTemp;

  connect(CoolingTower.C_hot_in, waterInletPress_sensor.C_out) annotation (Line(points={{-7.5,0},{-14,0},{-14,0},{-26,0}},
                                                                                                         color={28,108,200}));
  connect(AirInletTemp_sensor.C_in, cold_source.C_out) annotation (Line(points={{0,100},{0,109}},                                                           color={85,170,255}));
  connect(waterInletPress_sensor.C_in,waterInletTemp_sensor. C_out) annotation (Line(points={{-46,0},{-58,0}}, color={28,108,200}));
  connect(CoolingTower.C_hot_out, WaterOutletTemp_sensor.C_in) annotation (Line(points={{7.5,0},{16,0},{16,0},{30,0}},
                                                                                                       color={28,108,200}));
  connect(WaterOutletTemp_sensor.C_out, hot_sink.C_in) annotation (Line(points={{50,0},{69,0}}, color={28,108,200}));
  connect(hot_source.C_out, waterFlow_sensor.C_in) annotation (Line(points={{-123,0},{-110,0}},
                                                                                              color={28,108,200}));
  connect(waterInletTemp_sensor.C_in, waterFlow_sensor.C_out) annotation (Line(points={{-78,0},{-90,0}}, color={28,108,200}));
  connect(AirOutletTemp_sensor.C_out, cold_sink.C_in) annotation (Line(points={{-1.77636e-15,-74},{0,-83.5},{0,-87}},                                          color={85,170,255}));
  connect(AirInletTemp_sensor.C_out, airInletFlow_sensor.C_in) annotation (Line(points={{0,80},{0,68}},                                                         color={85,170,255}));
  connect(airInletFlow_sensor.C_out, airInletPress_sensor.C_in) annotation (Line(points={{0,48},{0,38}},                                                         color={85,170,255}));
  connect(airInletPress_sensor.C_out, CoolingTower.C_cold_in) annotation (Line(points={{0,18},{0,10},{0,8.83333},{0,8.83333}},                 color={85,170,255}));
  connect(CoolingTower.C_cold_out, airOutletPress_sensor.C_in) annotation (Line(points={{0,-8.66667},{0,-10},{0,-10},{0,-20}},                    color={85,170,255}));
  connect(AirOutletTemp_sensor.C_in, airOutletPress_sensor.C_out) annotation (Line(points={{0,-54},{0,-40}},                                                           color={85,170,255}));
  annotation (Diagram(coordinateSystem(extent={{-160,-120},{120,140}})), Icon(coordinateSystem(extent={{-160,-120},{120,140}}), graphics={
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
          points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}),
    experiment(StopTime=1, __Dymola_Algorithm="Dassl"));
end CoolingTowerMerkel_direct;
