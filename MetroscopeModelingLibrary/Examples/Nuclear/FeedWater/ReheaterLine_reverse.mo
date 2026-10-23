within MetroscopeModelingLibrary.Examples.Nuclear.FeedWater;
model ReheaterLine_reverse
  input Utilities.Units.SpecificEnthalpy main_steam_source_1_h(start=1.829e6);
  input Utilities.Units.SpecificEnthalpy main_steam_source_2_h(start=1.661e6);

  // Component parameters
  parameter Utilities.Units.Area S_tot=100;
  parameter Utilities.Units.Fraction level=0.3;

  WaterSteam.HeatExchangers.Reheater reheater_1A(S=S_tot,
    level=level) annotation (Placement(transformation(extent={{76,52},{44,68}})));
  WaterSteam.HeatExchangers.Reheater reheater_1B(S=S_tot,
    level=level) annotation (Placement(transformation(extent={{76,-68},{44,-52}})));
  WaterSteam.HeatExchangers.Reheater reheater_2A(S=S_tot,
    level=level) annotation (Placement(transformation(extent={{-44,52},{-76,68}})));
  WaterSteam.HeatExchangers.Reheater reheater_2B(S=S_tot,
    level=level) annotation (Placement(transformation(extent={{-44,-68},{-76,-52}})));
  WaterSteam.Pipes.ControlValve reheater_2A_drains_valve annotation (Placement(transformation(extent={{-36,18},{-26,30}})));
  SensorsControl.Outline.OpeningSensor reheater_2A_drains_valve_opening_sensor(sensor_function="Calibration",
    causality="reheater_2A_drains_valve_cvmax",
    Opening_start=50,
    signal_unit="%",
    Opening_pc_0=50) annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=0,
        origin={-31,45})));
  WaterSteam.Pipes.ControlValve reheater_1A_drains_valve annotation (Placement(transformation(extent={{86,18},{96,30}})));
  SensorsControl.Outline.OpeningSensor reheater_1A_drains_valve_opening_sensor(sensor_function="Calibration",
    causality="reheater_1A_drains_valve_cvmax",
    Opening_start=50,
    signal_unit="%",
    Opening_pc_0=50) annotation (Placement(transformation(
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
  SensorsControl.WaterSteam.TemperatureSensor reheaters_T_cold_out_sensor(sensor_function="Unidentified",
    causality="",
    T_start=224.2,
    signal_unit="degC",
    display_unit="degC",
    P_0=81.4e5,
    Q_0=1682,
    h_0=960000,
    T_0=273.15 + (224.2)) annotation (Placement(transformation(extent={{-148,-10},{-168,10}})));
  SensorsControl.WaterSteam.TemperatureSensor reheater_2A_drains_T_sensor(sensor_function="Calibration",
    causality="reheater_2A_Kth_subc",
    T_start=202.8,
    signal_unit="degC",
    display_unit="degC",
    P_0=26.4e5,
    Q_0=100,
    h_0=800000,
    T_0=273.15 + (202.8)) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-60,36})));
  SensorsControl.WaterSteam.TemperatureSensor reheater_1A_drains_T_sensor(sensor_function="Calibration",
    causality="reheater_1A_Kth_subc",
    T_start=156.2,
    signal_unit="degC",
    display_unit="degC",
    P_0=15.7e5,
    Q_0=100,
    h_0=800000,
    T_0=273.15 + (156.2)) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={60,36})));
  SensorsControl.WaterSteam.TemperatureSensor reheater_1A_T_cold_out_sensor(sensor_function="Calibration",
    causality="reheater_1A_Kth_cond",
    T_start=197.7,
    signal_unit="degC",
    display_unit="degC",
    P_0=81.4e5,
    Q_0=841,
    h_0=960000,
    T_0=273.15 + (197.7)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,60})));
  SensorsControl.WaterSteam.TemperatureSensor reheaters_T_cold_in_sensor(sensor_function="BC",
    causality="",
    T_start=150.6,
    signal_unit="degC",
    display_unit="degC",
    P_0=83.4e5,
    Q_0=1682,
    h_0=650000,
    T_0=273.15 + (150.6)) annotation (Placement(transformation(
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
  SensorsControl.WaterSteam.PressureSensor main_steam_source_2_P_sensor(sensor_function="BC",
    causality="",
    P_start=26.4,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(26.4),
    Q_0=100,
    h_0=800000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,120})));
  SensorsControl.WaterSteam.PressureSensor main_steam_source_1_P_sensor(sensor_function="BC",
    causality="",
    P_start=15.7,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(15.7),
    Q_0=100,
    h_0=800000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,120})));
  SensorsControl.WaterSteam.TemperatureSensor reheater_2B_drains_T_sensor(sensor_function="Calibration",
    causality="reheater_2B_Kth_subc",
    T_start=202.8,
    signal_unit="degC",
    display_unit="degC",
    P_0=26.4e5,
    Q_0=100,
    h_0=800000,
    T_0=273.15 + (202.8)) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-60,-90})));
  SensorsControl.WaterSteam.TemperatureSensor reheater_1B_drains_T_sensor(sensor_function="Calibration",
    causality="reheater_1B_Kth_subc",
    T_start=156.2,
    signal_unit="degC",
    display_unit="degC",
    P_0=15.7e5,
    Q_0=100,
    h_0=800000,
    T_0=273.15 + (156.2)) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={60,-90})));
  SensorsControl.WaterSteam.TemperatureSensor reheater_1B_T_cold_out_sensor(sensor_function="Calibration",
    causality="reheater_1B_Kth_cond",
    T_start=197.7,
    signal_unit="degC",
    display_unit="degC",
    P_0=81.4e5,
    Q_0=841,
    h_0=960000,
    T_0=273.15 + (197.7)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,-60})));
  WaterSteam.Pipes.ControlValve reheater_2B_drains_valve annotation (Placement(transformation(extent={{-36,-114},{-26,-102}})));
  SensorsControl.Outline.OpeningSensor reheater_2B_drains_valve_opening_sensor(sensor_function="Calibration",
    causality="reheater_2B_drains_valve_cvmax",
    Opening_start=50,
    signal_unit="%",
    Opening_pc_0=50) annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=0,
        origin={-31,-87})));
  WaterSteam.Pipes.ControlValve reheater_1B_drains_valve annotation (Placement(transformation(extent={{84,-114},{94,-102}})));
  SensorsControl.Outline.OpeningSensor reheater_1B_drains_valve_opening_sensor(sensor_function="Calibration",
    causality="reheater_1B_drains_valve_cvmax",
    Opening_start=50,
    signal_unit="%",
    Opening_pc_0=50) annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=0,
        origin={89,-87})));
  SensorsControl.WaterSteam.PressureSensor drains_sink_P_sensor(sensor_function="BC",
    causality="",
    P_start=4.78,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(4.78),
    Q_0=100,
    h_0=800000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={146,-112})));
  WaterSteam.BoundaryConditions.Sink drains_sink annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={190,-112})));
  SensorsControl.WaterSteam.FlowSensor reheaters_Q_cold_in_sensor(sensor_function="BC",
    causality="",
    Q_start=1682,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=83.4e5,
    Q_0=1682,
    h_0=650000) annotation (Placement(transformation(extent={{172,-10},{152,10}})));
  SensorsControl.WaterSteam.PressureSensor reheaters_P_cold_in_sensor(sensor_function="BC",
    causality="",
    P_start=83.4,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(83.4),
    Q_0=1682,
    h_0=650000) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={188,0})));
  SensorsControl.WaterSteam.TemperatureSensor reheater_2B_T_cold_out_sensor(sensor_function="Calibration",
    causality="reheater_2B_Kth_cond",
    T_start=224.2,
    signal_unit="degC",
    display_unit="degC",
    P_0=81.4e5,
    Q_0=841,
    h_0=960000,
    T_0=273.15 + (224.2)) annotation (Placement(transformation(extent={{-110,-70},{-130,-50}})));
  SensorsControl.WaterSteam.TemperatureSensor reheater_2A_T_cold_out_sensor(sensor_function="Calibration",
    causality="reheater_2A_Kth_cond",
    T_start=224.2,
    signal_unit="degC",
    display_unit="degC",
    P_0=81.4e5,
    Q_0=841,
    h_0=960000,
    T_0=273.15 + (224.2)) annotation (Placement(transformation(extent={{-110,50},{-130,70}})));
  // MML4 interfaces: quantities are declared at their connected instruments.
  Utilities.Interfaces.BoundaryCondition reheaters_P_cold_in annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={188,28})));
  Utilities.Interfaces.BoundaryCondition reheaters_Q_cold_in annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={162,28})));
  Utilities.Interfaces.BoundaryCondition reheaters_T_cold_in annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={134,-28})));
  Utilities.Interfaces.BoundaryCondition main_steam_source_1_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={68,120})));
  Utilities.Interfaces.CalibrationInput reheater_1A_drains_valve_opening_sensor_opening annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={91,66.1})));
  Utilities.Interfaces.CalibrationInput reheater_1A_drains_T annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={32,36})));
  Utilities.Interfaces.CalibrationInput reheater_1A_T_cold_out annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-0,32})));
  Utilities.Interfaces.CalibrationInput reheater_1B_drains_valve_opening_sensor_opening annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={89,-63.9})));
  Utilities.Interfaces.CalibrationInput reheater_1B_drains_T annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={32,-90})));
  Utilities.Interfaces.CalibrationInput reheater_1B_T_cold_out annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-0,-88})));
  Utilities.Interfaces.BoundaryCondition main_steam_source_2_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-52,120})));
  Utilities.Interfaces.CalibrationInput reheater_2A_drains_valve_opening_sensor_opening annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-31,68.1})));
  Utilities.Interfaces.CalibrationInput reheater_2A_drains_T annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-88,36})));
  Utilities.Interfaces.CalibrationInput reheater_2A_T_cold_out annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-120,88})));
  Utilities.Interfaces.CalibrationInput reheater_2B_drains_valve_opening_sensor_opening annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-31,-63.9})));
  Utilities.Interfaces.CalibrationInput reheater_2B_drains_T annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-88,-90})));
  Utilities.Interfaces.CalibrationInput reheater_2B_T_cold_out annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-120,-32})));
  Utilities.Interfaces.Observable reheaters_T_cold_out annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-158,28})));
  Utilities.Interfaces.BoundaryCondition drains_sink_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={146,-84})));
  Utilities.Interfaces.RealOutput reheater_1A_Kfr_cold annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={100,55})));
  Utilities.Interfaces.RealOutput reheater_1A_Kth_cond annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={85,80})));
  Utilities.Interfaces.RealOutput reheater_1A_Kth_subc annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={85,100})));
  Utilities.Interfaces.RealOutput reheater_1A_drains_valve_cvmax annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={113,12})));
  Utilities.Interfaces.RealOutput reheater_1B_Kfr_cold annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={100,-55})));
  Utilities.Interfaces.RealOutput reheater_1B_Kth_cond annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={85,-80})));
  Utilities.Interfaces.RealOutput reheater_1B_Kth_subc annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={85,-100})));
  Utilities.Interfaces.RealOutput reheater_1B_drains_valve_cvmax annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={111,-120})));
  Utilities.Interfaces.RealOutput reheater_2A_Kfr_cold annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-20,55})));
  Utilities.Interfaces.RealOutput reheater_2A_Kth_cond annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-35,80})));
  Utilities.Interfaces.RealOutput reheater_2A_Kth_subc annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-35,100})));
  Utilities.Interfaces.RealOutput reheater_2A_drains_valve_cvmax annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-9,12})));
  Utilities.Interfaces.RealOutput reheater_2B_Kfr_cold annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-20,-55})));
  Utilities.Interfaces.RealOutput reheater_2B_Kth_cond annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-35,-80})));
  Utilities.Interfaces.RealOutput reheater_2B_Kth_subc annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-35,-100})));
  Utilities.Interfaces.RealOutput reheater_2B_drains_valve_cvmax annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-9,-120})));

equation
  // Cold source
  // Boundary conditions

  // Reheaters 1
    // Boundary conditions

    main_steam_source_1.h_out = main_steam_source_1_h;

    // Component parameters 1A

    // Calibrated parameters 1A

    // Quantities definition

    // Component parameters 1B

    // Calibrated parameters 1B

    // Quantities definition

  // Reheaters 2
    // Boundary conditions

      main_steam_source_2.h_out = main_steam_source_2_h;

    // Component parameters 2A

    // Calibrated parameters 2A

      //reheater_2A.Kth_cond = reheater_2B_Kth_cond;

    // Quantities definition

    // Component parameters 2B

    // Calibrated parameters 2B

    // Quantities definition

  // Sinks
  // Quantities definition

  // Boundary condition

  connect(reheater_2A_drains_valve.C_out, reheater_1A.C_hot_in) annotation (Line(points={{-26,20.4},{-6,20.4},{-6,20.1818},{-6,20},{30,20},{30,82},{60,82},{60,68}}, color={217,67,180}));
  connect(reheater_2A_drains_valve.Opening, reheater_2A_drains_valve_opening_sensor.Opening) annotation (Line(points={{-31,30},{-31,39.9}}, color={0,0,127}));
  connect(reheater_1A_drains_valve.Opening, reheater_1A_drains_valve_opening_sensor.Opening) annotation (Line(points={{91,30},{91,37.9}}, color={0,0,127}));
  connect(reheater_1B.C_cold_in,reheater_1A.C_cold_in) annotation (Line(
      points={{76.2,-60},{112,-60},{112,60},{76.2,60}},
      color={28,108,200}));
  connect(reheaters_T_cold_out_sensor.C_out, sink.C_in) annotation (Line(points={{-168,0},{-172.5,0},{-177,0}}, color={28,108,200}));
  connect(reheater_2A.C_hot_out, reheater_2A_drains_T_sensor.C_in) annotation (Line(points={{-60,52},{-60,46}}, color={217,67,180}));
  connect(reheater_2A_drains_valve.C_in, reheater_2A_drains_T_sensor.C_out) annotation (Line(points={{-36,20.4},{-60,20.4},{-60,20.1818},{-60,26}}, color={217,67,180}));
  connect(reheater_1A.C_hot_out, reheater_1A_drains_T_sensor.C_in) annotation (Line(points={{60,52},{60,46}}, color={217,67,180}));
  connect(reheater_1A_drains_valve.C_in, reheater_1A_drains_T_sensor.C_out) annotation (Line(points={{86,20.4},{72,20.4},{72,20.1818},{72,20},{60,20},{60,26}}, color={217,67,180}));
  connect(reheater_1A.C_cold_out, reheater_1A_T_cold_out_sensor.C_in) annotation (Line(points={{44,60},{10,60}},color={28,108,200}));
  connect(reheater_1A_T_cold_out_sensor.C_out, reheater_2A.C_cold_in) annotation (Line(points={{-10,60},{-43.8,60}}, color={28,108,200}));
  connect(reheater_1B.C_hot_in, reheater_1A.C_hot_in) annotation (Line(points={{60,-52},{60,0},{20,0},{20,100},{60,100},{60,68}},
                                                                                                                                color={238,46,47}));
  connect(reheaters_T_cold_in_sensor.C_out, reheater_1A.C_cold_in) annotation (Line(points={{124,0},{112,0},{112,60},{76.2,60}}, color={28,108,200}));
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
  connect(reheater_2B_drains_valve.Opening, reheater_2B_drains_valve_opening_sensor.Opening) annotation (Line(points={{-31,-102},{-31,-92.1}}, color={0,0,127}));
  connect(reheater_2B_drains_valve.C_out, reheater_1B.C_hot_in) annotation (Line(points={{-26,-111.6},{20,-111.6},{20,-111.818},{20,-40},{60,-40},{60,-52}},
                                                                                                                                                 color={217,67,180}));
  connect(reheater_2B_drains_valve.C_in, reheater_2B_drains_T_sensor.C_out) annotation (Line(points={{-36,-111.6},{-60,-111.6},{-60,-111.818},{-60,-100}}, color={217,67,180}));
  connect(reheater_1B_drains_valve.Opening, reheater_1B_drains_valve_opening_sensor.Opening) annotation (Line(points={{89,-102},{89,-92.1}}, color={0,0,127}));
  connect(reheater_1B_drains_valve.C_in, reheater_1B_drains_T_sensor.C_out) annotation (Line(points={{84,-111.6},{84,-112},{60,-112},{60,-100}}, color={217,67,180}));
  connect(reheater_1B_drains_valve.C_out, drains_sink_P_sensor.C_in) annotation (Line(points={{94,-111.6},{94,-112},{136,-112}}, color={28,108,200}));
  connect(reheater_1A_drains_valve.C_out, drains_sink_P_sensor.C_in) annotation (Line(points={{96,20.4},{98,20.4},{98,20.1818},{98,20},{100,20},{100,-112},{136,-112}},
                                                                                                                                                 color={217,67,180}));
  connect(drains_sink_P_sensor.C_out, drains_sink.C_in) annotation (Line(points={{156,-112},{185,-112}}, color={217,67,180}));
  connect(reheaters_T_cold_in_sensor.C_in, reheaters_Q_cold_in_sensor.C_out) annotation (Line(points={{144,0},{149,0},{152,0}}, color={28,108,200}));
  connect(reheaters_Q_cold_in_sensor.C_in, reheaters_P_cold_in_sensor.C_out) annotation (Line(points={{172,0},{178,0}}, color={28,108,200}));
  connect(cold_source.C_out, reheaters_P_cold_in_sensor.C_in) annotation (Line(points={{205,0},{202.5,0},{198,0}}, color={28,108,200}));
  connect(reheater_2B.C_cold_out, reheater_2B_T_cold_out_sensor.C_in) annotation (Line(points={{-76,-60},{-110,-60}}, color={28,108,200}));
  connect(reheater_2B_T_cold_out_sensor.C_out, reheaters_T_cold_out_sensor.C_in) annotation (Line(points={{-130,-60},{-139,-60},{-139,0},{-148,0}}, color={28,108,200}));
  connect(reheater_2A.C_cold_out, reheater_2A_T_cold_out_sensor.C_in) annotation (Line(points={{-76,60},{-110,60}}, color={28,108,200}));
  connect(reheater_2A_T_cold_out_sensor.C_out, reheaters_T_cold_out_sensor.C_in) annotation (Line(points={{-130,60},{-140,60},{-140,0},{-148,0}}, color={28,108,200}));

  // Instrument signals and connected characteristics.
  connect(reheaters_P_cold_in_sensor.P_sensor, reheaters_P_cold_in) annotation (Line(points={{188,10},{188,28}}, color={0,0,127}));
  connect(reheaters_Q_cold_in_sensor.Q_sensor, reheaters_Q_cold_in) annotation (Line(points={{162,10},{162,28}}, color={0,0,127}));
  connect(reheaters_T_cold_in_sensor.T_sensor, reheaters_T_cold_in) annotation (Line(points={{134,-10},{134,-28}}, color={0,0,127}));
  connect(main_steam_source_1_P_sensor.P_sensor, main_steam_source_1_P) annotation (Line(points={{50,120},{68,120}}, color={0,0,127}));
  connect(reheater_1A_drains_valve_opening_sensor.opening_sensor, reheater_1A_drains_valve_opening_sensor_opening) annotation (Line(points={{91,48.1},{91,66.1}}, color={0,0,127}));
  connect(reheater_1A_drains_T_sensor.T_sensor, reheater_1A_drains_T) annotation (Line(points={{50,36},{32,36}}, color={0,0,127}));
  connect(reheater_1A_T_cold_out_sensor.T_sensor, reheater_1A_T_cold_out) annotation (Line(points={{0,50},{0,32}}, color={0,0,127}));
  connect(reheater_1B_drains_valve_opening_sensor.opening_sensor, reheater_1B_drains_valve_opening_sensor_opening) annotation (Line(points={{89,-81.9},{89,-63.9}}, color={0,0,127}));
  connect(reheater_1B_drains_T_sensor.T_sensor, reheater_1B_drains_T) annotation (Line(points={{50,-90},{32,-90}}, color={0,0,127}));
  connect(reheater_1B_T_cold_out_sensor.T_sensor, reheater_1B_T_cold_out) annotation (Line(points={{0,-70},{0,-88}}, color={0,0,127}));
  connect(main_steam_source_2_P_sensor.P_sensor, main_steam_source_2_P) annotation (Line(points={{-70,120},{-52,120}}, color={0,0,127}));
  connect(reheater_2A_drains_valve_opening_sensor.opening_sensor, reheater_2A_drains_valve_opening_sensor_opening) annotation (Line(points={{-31,50.1},{-31,68.1}}, color={0,0,127}));
  connect(reheater_2A_drains_T_sensor.T_sensor, reheater_2A_drains_T) annotation (Line(points={{-70,36},{-88,36}}, color={0,0,127}));
  connect(reheater_2A_T_cold_out_sensor.T_sensor, reheater_2A_T_cold_out) annotation (Line(points={{-120,70},{-120,88}}, color={0,0,127}));
  connect(reheater_2B_drains_valve_opening_sensor.opening_sensor, reheater_2B_drains_valve_opening_sensor_opening) annotation (Line(points={{-31,-81.9},{-31,-63.9}}, color={0,0,127}));
  connect(reheater_2B_drains_T_sensor.T_sensor, reheater_2B_drains_T) annotation (Line(points={{-70,-90},{-88,-90}}, color={0,0,127}));
  connect(reheater_2B_T_cold_out_sensor.T_sensor, reheater_2B_T_cold_out) annotation (Line(points={{-120,-50},{-120,-32}}, color={0,0,127}));
  connect(reheaters_T_cold_out_sensor.T_sensor, reheaters_T_cold_out) annotation (Line(points={{-158,10},{-158,28}}, color={0,0,127}));
  connect(drains_sink_P_sensor.P_sensor, drains_sink_P) annotation (Line(points={{146,-102},{146,-84}}, color={0,0,127}));
  connect(reheater_1A_Kfr_cold, reheater_1A.Kfr_cold) annotation (Line(points={{100,55},{76.4,55},{76.4,64}}, color={0,0,127}));
  reheater_1A_Kfr_cold = 120;
  connect(reheater_1A_Kth_cond, reheater_1A.Kth_cond) annotation (Line(points={{85,80},{70,80},{70,51.6}}, color={0,0,127}));
  connect(reheater_1A_Kth_subc, reheater_1A.Kth_subc) annotation (Line(points={{85,100},{70,100},{70,68.4}}, color={0,0,127}));
  connect(reheater_1A_drains_valve_cvmax, reheater_1A_drains_valve.Cv_max) annotation (Line(points={{113,12},{89,12},{89,27.6}}, color={0,0,127}));
  connect(reheater_1B_Kfr_cold, reheater_1B.Kfr_cold) annotation (Line(points={{100,-55},{76.4,-55},{76.4,-56}}, color={0,0,127}));
  reheater_1B_Kfr_cold = 120;
  connect(reheater_1B_Kth_cond, reheater_1B.Kth_cond) annotation (Line(points={{85,-80},{70,-80},{70,-68.4}}, color={0,0,127}));
  connect(reheater_1B_Kth_subc, reheater_1B.Kth_subc) annotation (Line(points={{85,-100},{70,-100},{70,-51.6}}, color={0,0,127}));
  connect(reheater_1B_drains_valve_cvmax, reheater_1B_drains_valve.Cv_max) annotation (Line(points={{111,-120},{87,-120},{87,-104.4}}, color={0,0,127}));
  connect(reheater_2A_Kfr_cold, reheater_2A.Kfr_cold) annotation (Line(points={{-20,55},{-43.6,55},{-43.6,64}}, color={0,0,127}));
  reheater_2A_Kfr_cold = 120;
  connect(reheater_2A_Kth_cond, reheater_2A.Kth_cond) annotation (Line(points={{-35,80},{-50,80},{-50,51.6}}, color={0,0,127}));
  connect(reheater_2A_Kth_subc, reheater_2A.Kth_subc) annotation (Line(points={{-35,100},{-50,100},{-50,68.4}}, color={0,0,127}));
  connect(reheater_2A_drains_valve_cvmax, reheater_2A_drains_valve.Cv_max) annotation (Line(points={{-9,12},{-33,12},{-33,27.6}}, color={0,0,127}));
  connect(reheater_2B_Kfr_cold, reheater_2B.Kfr_cold) annotation (Line(points={{-20,-55},{-43.6,-55},{-43.6,-56}}, color={0,0,127}));
  reheater_2B_Kfr_cold = 120;
  connect(reheater_2B_Kth_cond, reheater_2B.Kth_cond) annotation (Line(points={{-35,-80},{-50,-80},{-50,-68.4}}, color={0,0,127}));
  connect(reheater_2B_Kth_subc, reheater_2B.Kth_subc) annotation (Line(points={{-35,-100},{-50,-100},{-50,-51.6}}, color={0,0,127}));
  connect(reheater_2B_drains_valve_cvmax, reheater_2B_drains_valve.Cv_max) annotation (Line(points={{-9,-120},{-33,-120},{-33,-104.4}}, color={0,0,127}));
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
end ReheaterLine_reverse;
