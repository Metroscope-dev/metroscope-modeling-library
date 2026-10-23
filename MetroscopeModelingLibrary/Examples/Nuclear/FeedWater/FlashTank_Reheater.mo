within MetroscopeModelingLibrary.Examples.Nuclear.FeedWater;
model FlashTank_Reheater
  import MetroscopeModelingLibrary.Utilities.Units;
  parameter Units.MassFlowRate Q_cold_0 = 1000;
  parameter Units.MassFlowRate Q_hot_0 = 500;

  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.DryReheater dry_reheater(Q_hot_0=Q_hot_0,
    Q_cold_0=Q_cold_0,
    S=100) annotation (Placement(transformation(extent={{66,22},{34,38}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink feed_water_sink annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-120,30})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source feed_water_source annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={185,30}, rotation=180)));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_extraction_source annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={50,150}, rotation=270)));
  MetroscopeModelingLibrary.WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-78,-44},{-36,-2}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source drains_cooling_source annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-110,150}, rotation=270)));
  MetroscopeModelingLibrary.WaterSteam.Pipes.FrictionPipe reheater_to_flash_tank_DP(Q_0=Q_hot_0) annotation (Placement(transformation(
        extent={{-12.5,-12.5},{12.5,12.5}},
        rotation=180,
        origin={0,10})));
  MetroscopeModelingLibrary.WaterSteam.Machines.Pump feed_water_pump(Q_0=Q_hot_0/3) annotation (Placement(transformation(
        extent={{9,-9},{-9,9}},
        rotation=0,
        origin={-60,-80})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.FrictionPipe flash_tank_to_reheater_DP(Q_0=Q_hot_0/2) annotation (Placement(transformation(extent={{100,42},{73,69}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source power_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-52})));
  RotationalSpeedSensor feed_water_pump_VRot_sensor(sensor_function="BC",
    causality="",
    VRot_start=1000) annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=180,
        origin={-60,-110})));
  // MML4 interfaces: quantities are declared at their connected instruments.
  input Units.SpecificEnthalpy extraction_h(start=2.5e6);
  input Units.SpecificEnthalpy cooling_drains_h(start=8e5);
  Utilities.Interfaces.BoundaryCondition feed_water_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={150,2})));
  Utilities.Interfaces.BoundaryCondition feed_water_T annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={120,2})));
  Utilities.Interfaces.BoundaryCondition feed_water_Q annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={90,2})));
  Utilities.Interfaces.BoundaryCondition extraction_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={78,110})));
  Utilities.Interfaces.BoundaryCondition cooling_drains_Q annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-82,110})));
  Utilities.Interfaces.BoundaryCondition feed_water_pump_speed annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-60,-136.16})));
  Utilities.Interfaces.RealOutput dry_reheater_Kfr_cold annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={48.4,22})));
  Utilities.Interfaces.RealOutput dry_reheater_Kth annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={42,26.4})));
  Utilities.Interfaces.RealOutput reheater_to_flash_tank_DP_Kfr annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-12,-7})));
  Utilities.Interfaces.RealOutput feed_water_pump_rh annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-83.4,-99.2})));
  Utilities.Interfaces.RealOutput drains_height_delta_z annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-63,-6.8})));
  Utilities.Interfaces.RealOutput dry_reheater_Kfr_hot annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={16.8,-6})));
  Utilities.Interfaces.RealOutput flash_tank_to_reheater_Kfr annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={74.98,48.9})));
  SensorsControl.WaterSteam.PressureSensor feed_water_P_sensor(sensor_function="BC",
    causality="",
    P_start=50,
    signal_unit="barA",
    display_unit="barA",
    P_0=5e6,
    Q_0=1000,
    h_0=550000) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={150,30}, rotation=180)));
  SensorsControl.WaterSteam.TemperatureSensor feed_water_T_sensor(sensor_function="BC",
    causality="",
    T_start=130,
    signal_unit="degC",
    display_unit="degC",
    P_0=5e6,
    Q_0=1000,
    h_0=550000,
    T_0=403.15) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={120,30}, rotation=180)));
  SensorsControl.WaterSteam.FlowSensor feed_water_Q_sensor(sensor_function="BC",
    causality="",
    Q_start=1000,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=5e6,
    Q_0=1000,
    h_0=550000) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={90,30}, rotation=180)));
  SensorsControl.WaterSteam.PressureSensor extraction_P_sensor(sensor_function="BC",
    causality="",
    P_start=11,
    signal_unit="barA",
    display_unit="barA",
    P_0=11e5,
    Q_0=100,
    h_0=2.5e6) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={50,110}, rotation=270)));
  SensorsControl.WaterSteam.FlowSensor cooling_drains_Q_sensor(sensor_function="BC",
    causality="",
    Q_start=Q_hot_0,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=10e5,
    Q_0=Q_hot_0,
    h_0=8e5) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-110,110}, rotation=270)));
  WaterSteam.Pipes.HeightVariationPipe drains_height annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-45,10}, rotation=180)));
  WaterSteam.Pipes.FrictionPipe dry_reheater_hot_pipe(Q_0=100, P_in_0=11e5, P_out_0=10.9e5, h_0=800000) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={30,10}, rotation=180)));

equation
  // Boundary conditions

  // Reheater

  // Pressure losses

  // Pump
  // Observables used for calibration

  // Fixed parameters
  feed_water_pump.VRotn = 1000;
  feed_water_pump.rm = 1;
  feed_water_pump.a1 = 0;
  feed_water_pump.a2 = 0;
  feed_water_pump.b1 = 0;
  feed_water_pump.b2 = 0;
  feed_water_pump.rh_min = 0.20;

  connect(power_source.C_out, feed_water_pump.C_power) annotation (Line(points={{-60,-56.8},{-60,-70.28}}, color={244,125,35}));
  connect(feed_water_sink.C_in, dry_reheater.C_cold_out) annotation (Line(points={{-115,30},{34,30}}, color={63,81,181}));
  connect(feed_water_pump.C_out, feed_water_sink.C_in) annotation (Line(points={{-69,-80},{-99,-80},{-99,30},{-115,30}}, color={63,81,181}));
  connect(flash_tank_to_reheater_DP.C_out, dry_reheater.C_hot_in) annotation (Line(points={{73,55.5},{50,55.5},{50,38}}, color={63,81,181}));
  connect(flashTank.C_hot_liquid, feed_water_pump.C_in) annotation (Line(points={{-36,-31.4},{-36,-30},{-28,-30},{-28,-80},{-51,-80}}, color={28,108,200}));
  connect(flashTank.C_hot_steam, flash_tank_to_reheater_DP.C_in) annotation (Line(points={{-36,-14.6},{112,-14.6},{112,55.5},{100,55.5}}, color={28,108,200}));
  connect(feed_water_pump.VRot, feed_water_pump_VRot_sensor.VRot) annotation (Line(points={{-60,-90.8},{-60,-101.84}}, color={0,0,127}));

  // Instrument signals and connected characteristics.
  turbine_extraction_source.h_out = extraction_h;
  drains_cooling_source.h_out = cooling_drains_h;
  connect(feed_water_P_sensor.P_sensor, feed_water_P) annotation (Line(points={{150,20},{150,2}}, color={0,0,127}));
  connect(feed_water_T_sensor.T_sensor, feed_water_T) annotation (Line(points={{120,20},{120,2}}, color={0,0,127}));
  connect(feed_water_Q_sensor.Q_sensor, feed_water_Q) annotation (Line(points={{90,20},{90,2}}, color={0,0,127}));
  connect(extraction_P_sensor.P_sensor, extraction_P) annotation (Line(points={{60,110},{78,110}}, color={0,0,127}));
  connect(cooling_drains_Q_sensor.Q_sensor, cooling_drains_Q) annotation (Line(points={{-100,110},{-82,110}}, color={0,0,127}));
  connect(feed_water_source.C_out, feed_water_P_sensor.C_in) annotation (Line(points={{180,30},{160,30}}, color={28,108,200}));
  connect(feed_water_P_sensor.C_out, feed_water_T_sensor.C_in) annotation (Line(points={{140,30},{130,30}}, color={28,108,200}));
  connect(feed_water_T_sensor.C_out, feed_water_Q_sensor.C_in) annotation (Line(points={{110,30},{100,30}}, color={28,108,200}));
  connect(feed_water_Q_sensor.C_out, dry_reheater.C_cold_in) annotation (Line(points={{80,30},{66.2,30}}, color={28,108,200}));
  connect(turbine_extraction_source.C_out, extraction_P_sensor.C_in) annotation (Line(points={{50,145},{50,120}}, color={28,108,200}));
  connect(extraction_P_sensor.C_out, dry_reheater.C_hot_in) annotation (Line(points={{50,100},{50,38}}, color={28,108,200}));
  connect(drains_cooling_source.C_out, cooling_drains_Q_sensor.C_in) annotation (Line(points={{-110,145},{-110,120}}, color={28,108,200}));
  connect(cooling_drains_Q_sensor.C_out, flashTank.C_in) annotation (Line(points={{-110,100},{-78,100},{-78,-14.6}}, color={28,108,200}));
  connect(reheater_to_flash_tank_DP.C_out, drains_height.C_in) annotation (Line(points={{-12.5,10},{-35,10}}, color={28,108,200}));
  connect(drains_height.C_out, flashTank.C_in) annotation (Line(points={{-55,10},{-78,10},{-78,-14.6}}, color={28,108,200}));
  connect(dry_reheater.C_hot_out, dry_reheater_hot_pipe.C_in) annotation (Line(points={{50,22},{40,22},{40,10}}, color={28,108,200}));
  connect(dry_reheater_hot_pipe.C_out, reheater_to_flash_tank_DP.C_in) annotation (Line(points={{20,10},{12.5,10}}, color={28,108,200}));
  connect(feed_water_pump_VRot_sensor.VRot_sensor, feed_water_pump_speed) annotation (Line(points={{-60,-118.16},{-60,-136.16}}, color={0,0,127}));
  connect(dry_reheater_Kfr_cold, dry_reheater.Kfr_cold) annotation (Line(points={{48.4,22},{66.4,22},{66.4,34}}, color={0,0,127}));
  dry_reheater_Kfr_cold = 1;
  connect(dry_reheater_Kth, dry_reheater.Kth) annotation (Line(points={{42,26.4},{60,26.4},{60,38.4}}, color={0,0,127}));
  dry_reheater_Kth = 1e5;
  connect(reheater_to_flash_tank_DP_Kfr, reheater_to_flash_tank_DP.Kfr) annotation (Line(points={{-12,-7},{6,-7},{6,5}}, color={0,0,127}));
  reheater_to_flash_tank_DP_Kfr = 1;
  connect(feed_water_pump_rh, feed_water_pump.rh) annotation (Line(points={{-83.4,-99.2},{-65.4,-99.2},{-65.4,-87.2}}, color={0,0,127}));
  feed_water_pump_rh = 1;
  connect(drains_height_delta_z, drains_height.delta_z) annotation (Line(points={{-63,-6.8},{-45,-6.8},{-45,5.2}}, color={0,0,127}));
  drains_height_delta_z = -10;
  connect(dry_reheater_Kfr_hot, dry_reheater_hot_pipe.Kfr) annotation (Line(points={{16.8,-6},{34.8,-6},{34.8,6}}, color={0,0,127}));
  dry_reheater_Kfr_hot = 1;
  connect(flash_tank_to_reheater_Kfr, flash_tank_to_reheater_DP.Kfr) annotation (Line(points={{74.98,48.9},{92.98,48.9},{92.98,60.9}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-210,-170},{210,180}})), Icon(coordinateSystem(extent={{-140,-140},{140,140}}), graphics={
        Rectangle(
          extent={{-94,62},{106,-60}},
          lineColor={28,108,200},
          fillColor={236,238,248},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Polygon(
          points={{-94,-30},{106,-30},{106,-60},{-94,-60},{-94,-30}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-82,2},{-76,-4}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-74,-10},{-68,-16}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-68,6},{-62,0}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-86,-20},{-80,-26}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-78,14},{-72,8}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-62,-14},{-56,-20}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-86,40},{-80,34}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-62,18},{-56,12}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-48,-22},{-42,-28}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-88,-6},{-82,-12}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-86,22},{-80,16}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-70,-20},{-64,-26}},
          lineThickness=1,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-94,62},{106,-60}},
          lineColor={28,108,200},
          lineThickness=1)}));
end FlashTank_Reheater;
