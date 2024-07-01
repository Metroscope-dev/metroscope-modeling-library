within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model CoolingTowerPoppe_direct
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary Conditions
    // Hot Water Inlet
  input Real waterInletTemp(start=30) "deg_C";
  input Units.VolumeFlowRate waterInletFlow(start=30) "m3/s";
  input Real waterInletPress(start=1) "bar";

    // Cold Air Inlet
  input Real AirInletTemp(start=15) "deg_C";
  input Real airInletPress(start=1) "bar";
  input Units.Fraction cold_source_relative_humidity(start=0.5) "1";

  // Input for calibration
  output Real WaterOutletTemp(start=20) "deg_C";

  // Calibrated Parameters
  parameter Real hd(start=8.849857);

  // Parameters
  parameter Units.Area Afr = 3000;
  parameter Real Lfi = 15;
  parameter Real Cf = 0.025509778;

  parameter Real eta_fan = 1;
  parameter Real W_fan = 40000 "W";


  // Observables
  output Real airInletFlow(start=50000) "m3/s";

  output Real airOutletPress(start=1) "bar";
  output Real AirOutletTemp(start=25) "deg_C";

  // Output
  output Units.Fraction cold_sink_relative_humidity(start=0.40412638) "1";
  output Real V_inlet(start = 12.871763) "m/s";      //No known start value
  output Real deltaP_fan;

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-120,-30},{-100,-10}})));

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{66,-30},{86,-10}})));
  MultiFluid.HeatExchangers.CoolingTowerPoppe CoolingTower(
    air_outlet_flow(h_out_0=20400.438),
    air_inlet_flow(h_out_0=108262.83),
    w_out(start=0.0018949909)) annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=180,
        origin={0,-20})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,90})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-92})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor waterInletPress_sensor(P_0=100000)
                                                                                     annotation (Placement(transformation(extent={{-36,-30},{-16,-10}})));
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
  CoolingTower.Cf = Cf;
  CoolingTower.Afr = Afr;
  CoolingTower.Lfi = Lfi;

  CoolingTower.deltaP_fan = deltaP_fan;

  CoolingTower.eta_fan = eta_fan;
  CoolingTower.W_fan = W_fan;

  // Parameters
  CoolingTower.V_inlet = V_inlet;

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
  connect(waterInletPress_sensor.C_out, CoolingTower.water_inlet_connector) annotation (Line(points={{-16,-20},{-8,-20},{-8,-20},{-9.16667,-20}},
                                                                                                                                color={28,108,200}));
  connect(CoolingTower.water_outlet_connector, WaterOutletTemp_sensor.C_in) annotation (Line(points={{9.16667,-20},{18,-20},{18,-20},{34,-20}},
                                                                                                                              color={28,108,200}));
  connect(CoolingTower.air_outlet_connector, airOutletPress_sensor.C_in) annotation (Line(points={{0,-29},{0,-28},{0,-28},{0,-34}},
                                                                                                                         color={85,170,255}));
  connect(airInletPress_sensor.C_out, CoolingTower.air_inlet_connector) annotation (Line(points={{0,0},{0,-10},{0,-11},{0,-11}},
                                                                                                                      color={85,170,255}));
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
          points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}),
    experiment(__Dymola_Algorithm="Dassl"));
end CoolingTowerPoppe_direct;
