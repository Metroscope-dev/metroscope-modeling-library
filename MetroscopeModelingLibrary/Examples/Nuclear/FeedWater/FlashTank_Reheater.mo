within MetroscopeModelingLibrary.Examples.Nuclear.FeedWater;
model FlashTank_Reheater
  import MetroscopeModelingLibrary.Utilities.Units;
  parameter Units.MassFlowRate Q_cold_0 = 1000;
  parameter Units.MassFlowRate Q_hot_0 = 500;

  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.DryReheater dry_reheater(Q_hot_0=Q_hot_0, Q_cold_0=Q_cold_0) annotation (Placement(transformation(extent={{66,22},{34,38}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink feed_water_sink annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-120,30})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source feed_water_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={100,30})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_extraction_source
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={50,78})));
  MetroscopeModelingLibrary.WaterSteam.Volumes.FlashTank flashTank
    annotation (Placement(transformation(extent={{-78,-44},{-36,-2}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source drains_cooling_source
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-78,78})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe reheater_to_flash_tank_DP(Q_0=Q_hot_0)
    annotation (Placement(transformation(
        extent={{-12.5,-12.5},{12.5,12.5}},
        rotation=180,
        origin={0,10})));
  MetroscopeModelingLibrary.WaterSteam.Machines.Pump feed_water_pump(Q_0=Q_hot_0/3) annotation (Placement(transformation(
        extent={{9,-9},{-9,9}},
        rotation=0,
        origin={-60,-80})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe flash_tank_to_reheater_DP(Q_0=Q_hot_0/2) annotation (Placement(transformation(extent={{100,42},{73,69}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source power_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-52})));
  Sensors.Outline.VRotSensor feed_water_pump_VRot_sensor annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=180,
        origin={-60,-110})));
equation
  // Boundary conditions
  turbine_extraction_source.h_out = 2.5e6;
  turbine_extraction_source.P_out = 11e5;

  feed_water_source.P_out = 50e5;
  feed_water_source.T_out = 130 + 273.15;
  feed_water_source.Q_out = -1000;

  drains_cooling_source.h_out = 8e5;
  drains_cooling_source.Q_out = -Q_hot_0;

  // Reheater
  dry_reheater.S_condensing = 100;
  dry_reheater.Kfr_hot = 1;
  dry_reheater.Kfr_cold = 1;
  dry_reheater.Kth = 1e5;

  // Pressure losses
  reheater_to_flash_tank_DP.delta_z = -10;
  reheater_to_flash_tank_DP.Kfr = 1;
  flash_tank_to_reheater_DP.delta_z = 0;

  // Pump
  // Observables used for calibration
  feed_water_pump.VRot = 1000;
  // Fixed parameters
  feed_water_pump.VRotn = 1000;
  feed_water_pump.rm = 1;
  feed_water_pump.a1 = 0;
  feed_water_pump.a2 = 0;
  feed_water_pump.b1 = 0;
  feed_water_pump.b2 = 0;
  feed_water_pump.rhmin = 0.20;
  feed_water_pump.rh = 1;

  connect(power_source.C_out, feed_water_pump.C_power) annotation (Line(points={{-60,-56.8},{-60,-70.28}}, color={244,125,35}));
  connect(turbine_extraction_source.C_out, dry_reheater.C_hot_in) annotation (Line(points={{50,73},{50,38}}, color={63,81,181}));
  connect(feed_water_source.C_out, dry_reheater.C_cold_in) annotation (Line(points={{95,30},{66.2,30}}, color={63,81,181}));
  connect(feed_water_sink.C_in, dry_reheater.C_cold_out) annotation (Line(points={{-115,30},{34,30}}, color={63,81,181}));
  connect(drains_cooling_source.C_out, flashTank.C_in)
    annotation (Line(points={{-78,73},{-78,-14},{-78,-14},{-78,-14.6}},
                                                      color={63,81,181}));
  connect(dry_reheater.C_hot_out, reheater_to_flash_tank_DP.C_in) annotation (Line(points={{50,22},{50,10},{12.5,10}}, color={63,81,181}));
  connect(reheater_to_flash_tank_DP.C_out, flashTank.C_in) annotation (Line(points={{-12.5,10},{-78,10},{-78,-14.6}}, color={63,81,181}));
  connect(feed_water_pump.C_out, feed_water_sink.C_in) annotation (Line(points={{-69,-80},{-99,-80},{-99,30},{-115,30}}, color={63,81,181}));
  connect(flash_tank_to_reheater_DP.C_out, dry_reheater.C_hot_in) annotation (Line(points={{73,55.5},{50,55.5},{50,38}}, color={63,81,181}));
  connect(flashTank.C_hot_liquid, feed_water_pump.C_in) annotation (Line(points={{-36,-31.4},{-36,-30},{-28,-30},{-28,-80},{-51,-80}}, color={28,108,200}));
  connect(flashTank.C_hot_steam, flash_tank_to_reheater_DP.C_in) annotation (Line(points={{-36,-14.6},{112,-14.6},{112,55.5},{100,55.5}}, color={28,108,200}));
  connect(feed_water_pump.VRot, feed_water_pump_VRot_sensor.VRot) annotation (Line(points={{-60,-90.8},{-60,-101.84}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(extent={{-140,-140},{140,140}})), Icon(coordinateSystem(extent={{-140,-140},{140,140}}), graphics={
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
