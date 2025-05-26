within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model MetroscopiaNPP_reverse_EG
    WaterSteam.HeatExchangers.Condenser                           condenser(
    Q_cold_0=54000,
    Q_hot_0=1000,
    Psat_0=6980,
    P_cold_in_0=300000,
    P_cold_out_0=300000,
    T_cold_in_0=288.15,
    T_cold_out_0=298.15,
    h_cold_in_0=63e3,
    h_cold_out_0=105e3,
    h_hot_in_0=2.4e6)                                                                      annotation (Placement(transformation(extent={{4.5,
            8.5432},{35.5,34.321}})));
    WaterSteam.BoundaryConditions.Sink                           cold_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,20})));
    WaterSteam.BoundaryConditions.Source                           cold_source annotation (Placement(transformation(extent={{-90,10},
            {-70,30}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   CW_T_in_sensor(
    Q_0=54000,
    P_0=300000,
    h_0=63e3,
    sensor_function="BC",
    T_0=288.15)                                                                 annotation (Placement(transformation(extent={{-57,13},
            {-43,27}})));
  Sensors_Control.WaterSteam.PressureSensor                   CW_P_in_sensor(
    Q_0=54000,
    P_0=300000,
    h_0=63e3,
    sensor_function="BC")                                                    annotation (Placement(transformation(extent={{-27,13},
            {-13,27}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   CW_T_out_sensor(
    Q_0=54000,
    P_0=300000,
    h_0=105e3,
    sensor_function="Calibration",
    causality="condenser_Q_cold",
    T_0=298.15)                                                                  annotation (Placement(transformation(extent={{53,13},
            {67,27}})));
    WaterSteam.BoundaryConditions.Source                           cold_source1
                                                                               annotation (Placement(transformation(extent={{-10,-10},
            {10,10}},
        rotation=0,
        origin={-240,120})));
    WaterSteam.BoundaryConditions.Sink                           cold_sink1
                                                                           annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-132,-60})));
  Utilities.Interfaces.RealInput CW_T_in(start=15)
                                         annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-50,36}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput CW_P_in(start=3)
                                         annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-20,36}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput  CW_T_out(start=25)
                                           annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={60,36}), iconTransformation(extent={{-154,38},{-134,58}})));
  Utilities.Interfaces.RealOutput condenser_Kth annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=-90,
        origin={4,50}), iconTransformation(extent={{-160,20},{-140,40}})));
  Utilities.Interfaces.RealOutput condenser_Q_cold annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-8,50}), iconTransformation(extent={{-174,14},{-154,34}})));
  Sensors_Control.WaterSteam.PressureSensor                   P_cond_sensor(
    Q_0=1000,
    P_0=6900,
    h_0=2.4e6,
    sensor_function="Calibration",
    causality="condenser_Kth",
    display_unit="mbar",
    signal_unit="mbar")                                                     annotation (Placement(transformation(extent={{-88,114},
            {-76,126}})));
  Utilities.Interfaces.RealInput P_cond(start=69.8) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-82,136}), iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.Machines.FixedSpeedPump                 extraction_pump(
    T_in_0=312.05,
    T_out_0=312.15,
    P_in_0=6980,
    P_out_0=700000,
    h_in_0=163e3,
    h_out_0=164e3,
    Q_0=1060)                                                                       annotation (Placement(transformation(extent={{-24,-68},
            {-40,-52}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   extraction_pump_T_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=164e3,
    sensor_function="Calibration",
    causality="extraction_pump_rh",
    T_0=312.15)                                                                               annotation (Placement(transformation(extent={{-53,-67},
            {-67,-53}})));
  Sensors_Control.WaterSteam.PressureSensor                   extraction_pump_P_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=164e3,
    sensor_function="Calibration",
    causality="extraction_pump_hn")                                                        annotation (Placement(transformation(extent={{7,-7},{-7,7}}, origin={-80,-60})));
  Utilities.Interfaces.RealInput extraction_pump_P_out(start=7) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-80,-42}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput extraction_pump_T_out(start=39) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-60,-42}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealOutput extraction_pump_hn annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-22,-42}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealOutput extraction_pump_rh annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-14,-42}), iconTransformation(extent={{-174,14},{-154,34}})));
  WaterSteam.Machines.SteamTurbine                           LPT2(
    T_in_0=425.15,
    T_out_0=312.05,
    P_in_0=500000,
    P_out_0=6900,
    h_in_0=2.7e6,
    h_out_0=2.4e6,
    Q_0=1000) annotation (Placement(transformation(extent={{-145,112},{-127,128}})));
  Utilities.Interfaces.RealOutput LPT2_Cst annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-144,140}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealOutput turbines_eta_is annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-142,146}), iconTransformation(extent={{-174,14},{-154,34}})));
    Power.BoundaryConditions.Sink                           powerSink annotation (Placement(transformation(extent={{-54,150},
            {-34,170}})));
    Power.Machines.Generator                           generator annotation (Placement(transformation(extent={{-108,
            148},{-68,172}})));
    Sensors_Control.Power.PowerSensor                   W_elec_sensor annotation (Placement(transformation(extent={{-68,154},
            {-56,166}})));
  Utilities.Interfaces.RealInput W_elec(start=570) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-62,174}), iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.Pipes.SteamExtractionSplitter                           LP_extract(
    Q_in_0=1060,
    Q_ext_0=55,
    P_0=500000,
    T_0=425.15,
    h_0=2.7e6)                                                                    annotation (Placement(transformation(extent={{-210,
            110},{-190,128}})));
  Sensors_Control.WaterSteam.PressureSensor                   LP_extract_P_sensor(
    Q_0=55,
    P_0=500000,
    h_0=2.7e6,
    sensor_function="Calibration",
    causality="LPT2_Cst")                                                         annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-200,92})));
    WaterSteam.BoundaryConditions.Sink                           cold_sink2
                                                                           annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-200,56})));
  Utilities.Interfaces.RealInput LP_extract_P(start=5) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-180,92}), iconTransformation(extent={{-170,22},{-130,62}})));
equation
  cold_source1.T_out = 1;
  cold_source1.Q_out = 1;
  // cold_source1.P_out = 1;

  // Condenser
    // Parameters
  condenser.water_height = 1;
  condenser.C_incond = 0;
  condenser.P_offset = 0;
  condenser.Kfr_cold = 0;

  LP_extract.alpha = 1;
  cold_sink2.Q_in = 1;
  connect(cold_source.C_out,CW_T_in_sensor. C_in) annotation (Line(points={{-75,20},
          {-57,20}},                                                                                                           color={28,108,200}));
  connect(CW_T_in_sensor.C_out,CW_P_in_sensor. C_in) annotation (Line(points={{-43,20},
          {-27,20}},                                                                              color={28,108,200}));
  connect(CW_P_in_sensor.C_out,condenser. C_cold_in) annotation (Line(points={{-13,20},
          {4.5,20}},                                                                                                             color={28,108,200}));
  connect(CW_T_out_sensor.C_out,cold_sink. C_in) annotation (Line(points={{67,20},
          {85,20}},                                                                                                           color={28,108,200}));
  connect(condenser.C_cold_out,CW_T_out_sensor. C_in) annotation (Line(points={{35.19,
          20},{53,20}},                                                                                                            color={28,108,200}));
  connect(CW_T_in_sensor.T_sensor, CW_T_in)
    annotation (Line(points={{-50,27},{-50,36}}, color={0,0,127}));
  connect(CW_P_in_sensor.P_sensor, CW_P_in)
    annotation (Line(points={{-20,27},{-20,36}}, color={0,0,127}));
  connect(CW_T_out_sensor.T_sensor, CW_T_out)
    annotation (Line(points={{60,27},{60,36}}, color={0,0,127}));
  connect(condenser_Q_cold, condenser.Qv_cold_in) annotation (Line(points={{-8,50},
          {-8,27.1605},{2.95,27.1605}}, color={0,0,127}));
  connect(condenser_Kth, condenser.Kth) annotation (Line(points={{4,50},{4,35.7531},
          {7.6,35.7531}}, color={0,0,127}));
  connect(condenser.C_hot_in, P_cond_sensor.C_out) annotation (Line(points={{20,
          34.6074},{20,120},{-76,120}}, color={28,108,200}));
  connect(P_cond_sensor.P_sensor, P_cond)
    annotation (Line(points={{-82,126},{-82,136}}, color={0,0,127}));
  connect(extraction_pump.C_out,extraction_pump_T_out_sensor. C_in) annotation (Line(points={{-40,-60},
          {-53,-60}},                                                                                              color={28,108,200}));
  connect(extraction_pump_T_out_sensor.C_out,extraction_pump_P_out_sensor. C_in) annotation (Line(points={{-67,-60},
          {-73,-60}},                                                                                                           color={28,108,200}));
  connect(extraction_pump_P_out_sensor.C_out, cold_sink1.C_in)
    annotation (Line(points={{-87,-60},{-127,-60}}, color={28,108,200}));
  connect(extraction_pump.C_in, condenser.C_hot_out) annotation (Line(points={{-24,
          -60},{20,-60},{20,8.5432}}, color={28,108,200}));
  connect(extraction_pump_T_out_sensor.T_sensor, extraction_pump_T_out)
    annotation (Line(points={{-60,-53},{-60,-42}}, color={0,0,127}));
  connect(extraction_pump_P_out_sensor.P_sensor, extraction_pump_P_out)
    annotation (Line(points={{-80,-53},{-80,-42}}, color={0,0,127}));
  connect(extraction_pump_hn, extraction_pump_hn)
    annotation (Line(points={{-22,-42},{-22,-42}}, color={0,0,127}));
  connect(extraction_pump.hn, extraction_pump_hn) annotation (Line(points={{-26.88,
          -53.6},{-22,-53.6},{-22,-42}}, color={0,0,127}));
  connect(extraction_pump.rh, extraction_pump_rh) annotation (Line(points={{-25.12,
          -55.2},{-14,-55.2},{-14,-42}}, color={0,0,127}));
  connect(P_cond_sensor.C_in, LPT2.C_out)
    annotation (Line(points={{-88,120},{-127,120}}, color={28,108,200}));
  connect(LPT2.Cst, LPT2_Cst) annotation (Line(points={{-144.1,125.6},{-144,125.6},
          {-144,140}}, color={0,0,127}));
  connect(LPT2_Cst, LPT2_Cst)
    annotation (Line(points={{-144,140},{-144,140}}, color={0,0,127}));
  connect(turbines_eta_is, LPT2.eta_is) annotation (Line(points={{-142,146},{-142,
          126.08},{-142.3,126.08}}, color={0,0,127}));
  connect(powerSink.C_in,W_elec_sensor. C_out) annotation (Line(points={{-49,160},
          {-56.12,160}},                                                                      color={244,125,35}));
  connect(W_elec_sensor.C_in,generator. C_out) annotation (Line(points={{-68,160},
          {-74,160}},                                                                      color={244,125,35}));
  connect(LPT2.C_W_out, generator.C_in) annotation (Line(points={{-127,126.72},{
          -120,126.72},{-120,160},{-100.4,160}}, color={244,125,35}));
  connect(W_elec_sensor.W_sensor, W_elec)
    annotation (Line(points={{-62,166},{-62,174}}, color={0,0,127}));
  connect(LPT2.C_in,LP_extract. C_main_out)
    annotation (Line(points={{-145,120},{-189.4,120}}, color={28,108,200}));
  connect(LP_extract_P_sensor.P_sensor,LP_extract_P)
    annotation (Line(points={{-193,92},{-180,92}}, color={0,0,127}));
  connect(LP_extract.C_ext_out,LP_extract_P_sensor. C_in)
    annotation (Line(points={{-200,113.2},{-200,99}}, color={28,108,200}));
  connect(LP_extract_P_sensor.C_out,cold_sink2. C_in)
    annotation (Line(points={{-200,85},{-200,61}}, color={28,108,200}));
  connect(cold_source1.C_out, LP_extract.C_in)
    annotation (Line(points={{-235,120},{-210.6,120}}, color={28,108,200}));
  connect(turbines_eta_is, turbines_eta_is)
    annotation (Line(points={{-142,146},{-142,146}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-420,-100},
            {100,180}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-420,-100},{100,180}})));
end MetroscopiaNPP_reverse_EG;
