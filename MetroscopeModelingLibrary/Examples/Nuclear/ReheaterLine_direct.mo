within MetroscopeModelingLibrary.Examples.Nuclear;
model ReheaterLine_direct
  // Boundary conditions
  input Real main_steam_source_1_P(start=15.7) "barA";
  input Real main_steam_source_2_P(start=26.4) "barA";
  input Units.SpecificEnthalpy main_steam_source_1_h(start=1.829e6);
  input Units.SpecificEnthalpy main_steam_source_2_h(start=1.661e6);

  input Real reheaters_P_cold_in(start=83.4) "barA";
  input Units.PositiveMassFlowRate reheaters_Q_cold_in(start=1682);
  input Real reheaters_T_cold_in(start=150.6);
  input Real drains_sink_P(start=4.78);

  // Component parameters
  parameter Units.Area S_tot = 100;
  parameter Units.Fraction level = 0.3;
  parameter Units.FrictionCoefficient Kfr_hot = 0;
  parameter Units.FrictionCoefficient reheater_1A_Kfr_cold = 120;
  parameter Units.FrictionCoefficient reheater_1B_Kfr_cold = 120;
  parameter Units.FrictionCoefficient reheater_2A_Kfr_cold = 120;
  parameter Units.FrictionCoefficient reheater_2B_Kfr_cold = 120;

  // Calibrated parameters
  parameter Units.HeatExchangeCoefficient reheater_1A_Kth_cond = 142694.38;
  parameter Units.HeatExchangeCoefficient reheater_1B_Kth_cond = 142694.38;
  parameter Units.HeatExchangeCoefficient reheater_2A_Kth_cond = 128084.8;
  parameter Units.HeatExchangeCoefficient reheater_2B_Kth_cond = 128084.8;
  parameter Units.HeatExchangeCoefficient reheater_1A_Kth_subc = 122580.32;
  parameter Units.HeatExchangeCoefficient reheater_1B_Kth_subc = 122580.32;
  parameter Units.HeatExchangeCoefficient reheater_2A_Kth_subc = 38013.305;
  parameter Units.HeatExchangeCoefficient reheater_2B_Kth_subc = 38013.305;

  parameter Units.Cv reheater_1A_drains_valve_cvmax = 1177.3293;
  parameter Units.Cv reheater_1B_drains_valve_cvmax = 1177.3293;
  parameter Units.Cv reheater_2A_drains_valve_cvmax = 459.99002;
  parameter Units.Cv reheater_2B_drains_valve_cvmax = 459.99002;
  output Real reheaters_T_cold_out;

  // Observables used for calibration
  output Real reheater_1A_T_cold_out;
  output Real reheater_1B_T_cold_out;
  output Real reheater_2A_T_cold_out;
  output Real reheater_2B_T_cold_out;

  output Real reheater_1A_drains_T;
  output Real reheater_1B_drains_T;
  output Real reheater_2A_drains_T;
  output Real reheater_2B_drains_T;

  output Units.Percentage reheater_1A_drains_valve_opening_sensor_opening;
  output Units.Percentage reheater_1B_drains_valve_opening_sensor_opening;
  output Units.Percentage reheater_2A_drains_valve_opening_sensor_opening;
  output Units.Percentage reheater_2B_drains_valve_opening_sensor_opening;

  WaterSteam.HeatExchangers.Reheater reheater_1A annotation (Placement(transformation(extent={{76,52},{44,68}})));
  WaterSteam.HeatExchangers.Reheater reheater_1B annotation (Placement(transformation(extent={{76,-68},{44,-52}})));
  WaterSteam.HeatExchangers.Reheater reheater_2A annotation (Placement(transformation(extent={{-44,52},{-76,68}})));
  WaterSteam.HeatExchangers.Reheater reheater_2B annotation (Placement(transformation(extent={{-44,-68},{-76,-52}})));
  WaterSteam.Pipes.ControlValve reheater_2A_drains_valve annotation (Placement(transformation(extent={{-36,18},{-26,30}})));
  Sensors.Outline.OpeningSensor reheater_2A_drains_valve_opening_sensor annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=0,
        origin={-31,45})));
  WaterSteam.Pipes.ControlValve reheater_1A_drains_valve annotation (Placement(transformation(extent={{86,18},{96,30}})));
  Sensors.Outline.OpeningSensor reheater_1A_drains_valve_opening_sensor annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=0,
        origin={91,43})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-182,0})));
  WaterSteam.BoundaryConditions.Source main_steam_source_2 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,146})));
  Sensors.WaterSteam.TemperatureSensor reheaters_T_cold_out_sensor annotation (Placement(transformation(extent={{-148,-10},{-168,10}})));
  Sensors.WaterSteam.TemperatureSensor reheater_2A_drains_T_sensor annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-60,36})));
  Sensors.WaterSteam.TemperatureSensor reheater_1A_drains_T_sensor annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={60,36})));
  Sensors.WaterSteam.TemperatureSensor reheater_1A_T_cold_out_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,60})));
  Sensors.WaterSteam.TemperatureSensor reheaters_T_cold_in_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={134,0})));
  WaterSteam.BoundaryConditions.Source main_steam_source_1 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,146})));
  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={210,0})));
  Sensors.WaterSteam.PressureSensor main_steam_source_2_P_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,120})));
  Sensors.WaterSteam.PressureSensor main_steam_source_1_P_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,120})));
  Sensors.WaterSteam.TemperatureSensor reheater_2B_drains_T_sensor annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-60,-90})));
  Sensors.WaterSteam.TemperatureSensor reheater_1B_drains_T_sensor annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={60,-90})));
  Sensors.WaterSteam.TemperatureSensor reheater_1B_T_cold_out_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,-60})));
  WaterSteam.Pipes.ControlValve reheater_2B_drains_valve annotation (Placement(transformation(extent={{-36,-114},{-26,-102}})));
  Sensors.Outline.OpeningSensor reheater_2B_drains_valve_opening_sensor annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=0,
        origin={-31,-87})));
  WaterSteam.Pipes.ControlValve reheater_1B_drains_valve annotation (Placement(transformation(extent={{84,-114},{94,-102}})));
  Sensors.Outline.OpeningSensor reheater_1B_drains_valve_opening_sensor annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=0,
        origin={89,-87})));
  Sensors.WaterSteam.PressureSensor drains_sink_P_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={146,-112})));
  WaterSteam.BoundaryConditions.Sink drains_sink annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={190,-112})));
  Sensors.WaterSteam.FlowSensor reheaters_Q_cold_in_sensor annotation (Placement(transformation(extent={{172,-10},{152,10}})));
  Sensors.WaterSteam.PressureSensor reheaters_P_cold_in_sensor annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={188,0})));
  Sensors.WaterSteam.TemperatureSensor reheater_2B_T_cold_out_sensor annotation (Placement(transformation(extent={{-110,-70},{-130,-50}})));
  Sensors.WaterSteam.TemperatureSensor reheater_2A_T_cold_out_sensor annotation (Placement(transformation(extent={{-110,50},{-130,70}})));
equation
  // Cold source
  // Boundary conditions
  reheaters_P_cold_in_sensor.P_barA = reheaters_P_cold_in;
  reheaters_Q_cold_in_sensor.Q = reheaters_Q_cold_in;
  reheaters_T_cold_in_sensor.T_degC = reheaters_T_cold_in;

  // Reheaters 1
    // Boundary conditions
    main_steam_source_1_P_sensor.P_barA = main_steam_source_1_P;
    main_steam_source_1.h_out = main_steam_source_1_h;

    // Component parameters 1A
      reheater_1A.S_tot = S_tot;
      reheater_1A.Kfr_hot = Kfr_hot;
      reheater_1A.Kfr_cold = reheater_1A_Kfr_cold;
      reheater_1A.level = level;
    // Calibrated parameters 1A
      reheater_1A.Kth_cond = reheater_1A_Kth_cond;
      reheater_1A.Kth_subc = reheater_1A_Kth_subc;
      reheater_1A_drains_valve.Cvmax = reheater_1A_drains_valve_cvmax;
    // Quantities definition
      reheater_1A_drains_valve_opening_sensor.Opening_pc = reheater_1A_drains_valve_opening_sensor_opening;
      reheater_1A_drains_T_sensor.T_degC = reheater_1A_drains_T;
      reheater_1A_T_cold_out_sensor.T_degC = reheater_1A_T_cold_out;

    // Component parameters 1B
      reheater_1B.S_tot = S_tot;
      reheater_1B.Kfr_hot = Kfr_hot;
      reheater_1B.Kfr_cold = reheater_1B_Kfr_cold;
      reheater_1B.level = level;
    // Calibrated parameters 1B
      reheater_1B.Kth_cond = reheater_1B_Kth_cond;
      reheater_1B.Kth_subc = reheater_1B_Kth_subc;
      reheater_1B_drains_valve.Cvmax = reheater_1B_drains_valve_cvmax;
    // Quantities definition
      reheater_1B_drains_valve_opening_sensor.Opening_pc = reheater_1B_drains_valve_opening_sensor_opening;
      reheater_1B_drains_T_sensor.T_degC = reheater_1B_drains_T;
      reheater_1B_T_cold_out_sensor.T_degC = reheater_1B_T_cold_out;


  // Reheaters 2
    // Boundary conditions
      main_steam_source_2_P_sensor.P_barA = main_steam_source_2_P;
      main_steam_source_2.h_out = main_steam_source_2_h;

    // Component parameters 2A
      reheater_2A.S_tot = S_tot;
      reheater_2A.Kfr_hot = Kfr_hot;
      reheater_2A.Kfr_cold = reheater_2A_Kfr_cold;
      reheater_2A.level = level;
    // Calibrated parameters 2A
      reheater_2A.Kth_cond = reheater_2A_Kth_cond;
      //reheater_2A.Kth_cond = reheater_2B_Kth_cond;
      reheater_2A.Kth_subc = reheater_2A_Kth_subc;
      reheater_2A_drains_valve.Cvmax = reheater_2A_drains_valve_cvmax;
    // Quantities definition
      reheater_2A_drains_valve_opening_sensor.Opening_pc = reheater_2A_drains_valve_opening_sensor_opening;
      reheater_2A_drains_T_sensor.T_degC = reheater_2A_drains_T;
      reheater_2A_T_cold_out_sensor.T_degC = reheater_2A_T_cold_out;

    // Component parameters 2B
      reheater_2B.S_tot = S_tot;
      reheater_2B.Kfr_hot = Kfr_hot;
      reheater_2B.Kfr_cold = reheater_2B_Kfr_cold;
      reheater_2B.level = level;
    // Calibrated parameters 2B
      reheater_2B.Kth_cond = reheater_2B_Kth_cond;
      reheater_2B.Kth_subc = reheater_2B_Kth_subc;
      reheater_2B_drains_valve.Cvmax = reheater_2B_drains_valve_cvmax;
    // Quantities definition
      reheater_2B_drains_valve_opening_sensor.Opening_pc = reheater_2B_drains_valve_opening_sensor_opening;
      reheater_2B_drains_T_sensor.T_degC = reheater_2B_drains_T;
      reheater_2B_T_cold_out_sensor.T_degC = reheater_2B_T_cold_out;

  // Sinks
  // Quantities definition
  reheaters_T_cold_out_sensor.T_degC = reheaters_T_cold_out;
  // Boundary condition
  drains_sink_P_sensor.P_barA = drains_sink_P;
  connect(reheater_2A_drains_valve.C_out, reheater_1A.C_hot_in) annotation (Line(points={{-26,20.1818},{-6,20.1818},{-6,20},{30,20},{30,82},{60,82},{60,68}}, color={217,67,180}));
  connect(reheater_2A_drains_valve.Opening, reheater_2A_drains_valve_opening_sensor.Opening) annotation (Line(points={{-31,28.9091},{-31,39.9}}, color={0,0,127}));
  connect(reheater_1A_drains_valve.Opening, reheater_1A_drains_valve_opening_sensor.Opening) annotation (Line(points={{91,28.9091},{91,37.9}}, color={0,0,127}));
  connect(reheater_1B.C_cold_in,reheater_1A. C_cold_in) annotation (Line(
      points={{76.2,-60},{112,-60},{112,60},{76.2,60}},
      color={28,108,200}));
  connect(reheaters_T_cold_out_sensor.C_out, sink.C_in) annotation (Line(points={{-168,0},{-172.5,0},{-172.5,-1.11022e-15},{-177,-1.11022e-15}}, color={28,108,200}));
  connect(reheater_2A.C_hot_out, reheater_2A_drains_T_sensor.C_in) annotation (Line(points={{-60,52},{-60,46}}, color={217,67,180}));
  connect(reheater_2A_drains_valve.C_in, reheater_2A_drains_T_sensor.C_out) annotation (Line(points={{-36,20.1818},{-60,20.1818},{-60,26}}, color={217,67,180}));
  connect(reheater_1A.C_hot_out, reheater_1A_drains_T_sensor.C_in) annotation (Line(points={{60,52},{60,46}}, color={217,67,180}));
  connect(reheater_1A_drains_valve.C_in, reheater_1A_drains_T_sensor.C_out) annotation (Line(points={{86,20.1818},{72,20.1818},{72,20},{60,20},{60,26}}, color={217,67,180}));
  connect(reheater_1A.C_cold_out, reheater_1A_T_cold_out_sensor.C_in) annotation (Line(points={{44,60},{10,60}},color={28,108,200}));
  connect(reheater_1A_T_cold_out_sensor.C_out, reheater_2A.C_cold_in) annotation (Line(points={{-10,60},{-43.8,60}}, color={28,108,200}));
  connect(reheater_1B.C_hot_in, reheater_1A.C_hot_in) annotation (Line(points={{60,-52},{60,0},{20,0},{20,100},{60,100},{60,68}},
                                                                                                                                color={238,46,47}));
  connect(reheaters_T_cold_in_sensor.C_out, reheater_1A.C_cold_in) annotation (Line(points={{124,6.66134e-16},{112,6.66134e-16},{112,60},{76.2,60}}, color={28,108,200}));
  connect(reheater_2A.C_hot_in, main_steam_source_2_P_sensor.C_out) annotation (Line(points={{-60,68},{-60,100},{-80,100},{-80,110}}, color={238,46,47}));
  connect(main_steam_source_2_P_sensor.C_in, main_steam_source_2.C_out) annotation (Line(points={{-80,130},{-80,141}},   color={238,46,47}));
  connect(reheater_2B.C_hot_in, main_steam_source_2_P_sensor.C_out) annotation (Line(points={{-60,-52},{-60,0},{-100,0},{-100,100},{-80,100},{-80,110}}, color={238,46,47}));
  connect(reheater_1A.C_hot_in, main_steam_source_1_P_sensor.C_out) annotation (Line(points={{60,68},{60,100},{40,100},{40,110}},
                                                                                                                                color={238,46,47}));
  connect(main_steam_source_1_P_sensor.C_in, main_steam_source_1.C_out) annotation (Line(points={{40,130},{40,141}}, color={238,46,47}));
  connect(reheater_2B_drains_T_sensor.C_in, reheater_2B.C_hot_out) annotation (Line(points={{-60,-80},{-60,-68}}, color={217,67,180}));
  connect(reheater_1B_drains_T_sensor.C_in, reheater_1B.C_hot_out) annotation (Line(points={{60,-80},{60,-68}}, color={217,67,180}));
  connect(reheater_1B.C_cold_out, reheater_1B_T_cold_out_sensor.C_in) annotation (Line(points={{44,-60},{10,-60}}, color={28,108,200}));
  connect(reheater_1B_T_cold_out_sensor.C_out, reheater_2B.C_cold_in) annotation (Line(points={{-10,-60},{-43.8,-60}},
                                                                                                                     color={28,108,200}));
  connect(reheater_2B_drains_valve.Opening, reheater_2B_drains_valve_opening_sensor.Opening) annotation (Line(points={{-31,-103.091},{-31,-92.1}}, color={0,0,127}));
  connect(reheater_2B_drains_valve.C_out, reheater_1B.C_hot_in) annotation (Line(points={{-26,-111.818},{20,-111.818},{20,-40},{60,-40},{60,-52}},
                                                                                                                                                 color={217,67,180}));
  connect(reheater_2B_drains_valve.C_in, reheater_2B_drains_T_sensor.C_out) annotation (Line(points={{-36,-111.818},{-60,-111.818},{-60,-100}}, color={217,67,180}));
  connect(reheater_1B_drains_valve.Opening, reheater_1B_drains_valve_opening_sensor.Opening) annotation (Line(points={{89,-103.091},{89,-92.1}}, color={0,0,127}));
  connect(reheater_1B_drains_valve.C_in, reheater_1B_drains_T_sensor.C_out) annotation (Line(points={{84,-111.818},{84,-112},{60,-112},{60,-100}}, color={217,67,180}));
  connect(reheater_1B_drains_valve.C_out, drains_sink_P_sensor.C_in) annotation (Line(points={{94,-111.818},{94,-112},{136,-112}}, color={28,108,200}));
  connect(reheater_1A_drains_valve.C_out, drains_sink_P_sensor.C_in) annotation (Line(points={{96,20.1818},{98,20.1818},{98,20},{100,20},{100,-112},{136,-112}},
                                                                                                                                                 color={217,67,180}));
  connect(drains_sink_P_sensor.C_out, drains_sink.C_in) annotation (Line(points={{156,-112},{185,-112}}, color={217,67,180}));
  connect(reheaters_T_cold_in_sensor.C_in, reheaters_Q_cold_in_sensor.C_out) annotation (Line(points={{144,-1.72085e-15},{149,-1.72085e-15},{149,0},{152,0}}, color={28,108,200}));
  connect(reheaters_Q_cold_in_sensor.C_in, reheaters_P_cold_in_sensor.C_out) annotation (Line(points={{172,0},{178,0}}, color={28,108,200}));
  connect(cold_source.C_out, reheaters_P_cold_in_sensor.C_in) annotation (Line(points={{205,5.55112e-17},{202.5,5.55112e-17},{202.5,0},{198,0}}, color={28,108,200}));
  connect(reheater_2B.C_cold_out, reheater_2B_T_cold_out_sensor.C_in) annotation (Line(points={{-76,-60},{-110,-60}}, color={28,108,200}));
  connect(reheater_2B_T_cold_out_sensor.C_out, reheaters_T_cold_out_sensor.C_in) annotation (Line(points={{-130,-60},{-139,-60},{-139,0},{-148,0}}, color={28,108,200}));
  connect(reheater_2A.C_cold_out, reheater_2A_T_cold_out_sensor.C_in) annotation (Line(points={{-76,60},{-110,60}}, color={28,108,200}));
  connect(reheater_2A_T_cold_out_sensor.C_out, reheaters_T_cold_out_sensor.C_in) annotation (Line(points={{-130,60},{-140,60},{-140,0},{-148,0}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,-80},{160,80}}),   graphics={
        Polygon(
          points={{160,80},{160,60},{160,-62.5},{160,-80},{120,-80},{-10,-80},{-160,-80},{-160,80},{-10,80},{120,80},{160,80}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{106,60},{106,38},{106,-42},{106,-62},{86,-62},{-6,-62},{-134,-62},{-134,60},{-2,60},{86,60},{106,60}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={205,225,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{132,-18},{144,-18},{144,-48},{142,-60},{86,-62},{-6,-62},{-116,-62},{-138,-18},{-120,-18},{-4,-20},{118,-18},{132,-18}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={35,138,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{118,42},{126,44},{126,-44},{118,-44},{108,-44},{-6,-44},{-114,-44},{-116,42},{-2,42},{110,42},{118,42}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{126,48},{132,48},{132,-50},{124,-50},{112,-50},{2,-50},{-120,-50},{-120,48},{2,48},{112,48},{126,48}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{106,30},{112,30},{112,-32},{104,-32},{92,-32},{-22,-32},{-120,-32},{-118,28},{-20,30},{92,30},{106,30}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{114,36},{120,36},{120,-36},{112,-38},{102,-38},{-10,-38},{-120,-40},{-124,34},{-12,36},{100,36},{114,36}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Rectangle(
          extent={{154,50},{104,-66}},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{150,60},{104,24}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=10),
        Rectangle(
          extent={{150,50},{102,16}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0),
        Rectangle(
          extent={{122,60},{102,38}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0),
        Rectangle(
          extent={{20,-23},{-20,23}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=10,
          origin={125,-42},
          rotation=90),
        Rectangle(
          extent={{10.5,-11.5},{-10.5,11.5}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0,
          origin={114.5,-51.5},
          rotation=90),
        Rectangle(
          extent={{14,-23},{-14,23}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0,
          origin={125,-32},
          rotation=90)}),
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-220,-160},{220,160}})));
end ReheaterLine_direct;
