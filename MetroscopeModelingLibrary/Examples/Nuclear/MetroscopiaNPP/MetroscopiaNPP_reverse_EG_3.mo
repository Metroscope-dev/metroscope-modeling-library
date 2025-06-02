within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model MetroscopiaNPP_reverse_EG_3
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
    WaterSteam.BoundaryConditions.Source                           steam_source
                                                                               annotation (Placement(transformation(extent={{-10,-10},
            {10,10}},
        rotation=0,
        origin={-392,120})));
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
        origin={-6,48}), iconTransformation(extent={{-174,14},{-154,34}})));
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
        origin={-80,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput extraction_pump_T_out(start=39) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-60,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealOutput extraction_pump_hn annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-22,-44}), iconTransformation(extent={{-174,14},{-154,34}})));
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
        origin={-210,152}), iconTransformation(extent={{-174,14},{-154,34}})));
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
  Utilities.Interfaces.RealInput LP_extract_P(start=5) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-180,92}), iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.HeatExchangers.DryReheater                           LP_heater(
    Q_cold_0=1060,
    Q_hot_0=55,
    P_cold_in_0=700000,
    P_cold_out_0=600000,
    P_hot_in_0=500000,
    P_hot_out_0=500000,
    T_cold_in_0=312.15,
    T_cold_out_0=338.15,
    T_hot_in_0=425.15,
    T_hot_out_0=425.15,
    h_cold_in_0=164e3,
    h_cold_out_0=272e3,
    h_hot_in_0=2.7e6,
    h_hot_out_0=640e3)                                                        annotation (Placement(transformation(extent={{-184,
            -68},{-216,-52}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   LP_heater_T_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=272e3,
    sensor_function="Calibration",
    causality="LP_heater_Kth",
    T_0=338.15)                                                                         annotation (Placement(transformation(extent={{-253,
            -67},{-267,-53}})));
  Sensors_Control.WaterSteam.PressureSensor                   LP_heater_P_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=272e3,
    sensor_function="Calibration",
    causality="LP_heater_Kfr_cold")                                                  annotation (Placement(transformation(extent={{-223,
            -67},{-237,-53}})));
  Utilities.Interfaces.RealInput LP_heater_P_out(start=6) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-230,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput LP_heater_T_out(start=65) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-260,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealOutput LP_heater_Kfr_cold annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-194,-42}), iconTransformation(extent={{-328,-88},{-308,-68}})));
  Utilities.Interfaces.RealOutput LP_heater_Kth annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-188,-40}), iconTransformation(extent={{-328,-88},{-308,-68}})));
    WaterSteam.Pipes.ControlValve                           LP_reheater_drains_control_valve(
    P_in_0=500000,
    P_out_0=6900,
    Q_0=55,
    T_0=425.15,
    h_0=640e3)                                                                               annotation (Placement(transformation(extent={{-145,
            -102.182},{-135,-90.182}})));
  Sensors_Control.Outline.OpeningSensor                   LP_reheater_drains_control_valve_opening_sensor(
      output_signal_unit="%")                                                                             annotation (Placement(transformation(extent={{-145,
            -87},{-135,-77}})));
  Utilities.Interfaces.RealInput LP_reheater_drains_control_valve_opening(start=15)
              annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-140,-70}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealOutput LP_heater_drains_control_valve_Cvmax
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-150,-78}),  iconTransformation(extent={{-328,-88},{-308,-68}})));
  WaterSteam.Machines.SteamTurbine                           LPT1(
    T_in_0=501.15,
    T_out_0=425.15,
    P_in_0=1940000,
    P_out_0=500000,
    h_in_0=2.85e6,
    h_out_0=2.7e6,
    Q_0=1060) annotation (Placement(transformation(extent={{-281,112},{-263,128}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   superheater_T_out_sensor(
    Q_0=1060,
    P_0=1940000,
    h_0=2.85e6,
    T_0=501.15)                                                                           annotation (Placement(transformation(extent={{-346,
            114},{-334,126}})));
  Utilities.Interfaces.RealOutput LPT1_Cst annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-280,140}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealInput superheater_T_out(start=228) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-340,136}), iconTransformation(extent={{-170,22},{-130,62}})));
  WaterSteam.Pipes.HeightVariationPipe deaerator_inlet_pipe
    annotation (Placement(transformation(extent={{-320,-70},{-300,-50}})));
  Modelica.Blocks.Sources.RealExpression deaerator_inlet_pipe_delta_z(y=5)
    annotation (Placement(transformation(extent={{-336,-56},{-316,-36}})));
  Modelica.Blocks.Sources.RealExpression condenser_Kfr_cold(y=0)
    annotation (Placement(transformation(extent={{-28,50},{-8,70}})));
  WaterSteam.Pipes.HeightVariationPipe deaerator_outlet_pipe
    annotation (Placement(transformation(extent={{-386,-70},{-366,-50}})));
  Modelica.Blocks.Sources.RealExpression deaerator_outlet_pipe_delta_z(y=-5)
    annotation (Placement(transformation(extent={{-402,-56},{-382,-36}})));
    WaterSteam.Machines.FixedSpeedPump                 feedwater_pump(
    T_in_0=350.05,
    T_out_0=353.15,
    P_in_0=600000,
    P_out_0=5900000,
    h_in_0=322e3,
    h_out_0=340e3,
    Q_0=1500)                                                                                         annotation (Placement(transformation(extent={{-426,
            -68},{-442,-52}})));
  Utilities.Interfaces.RealOutput feedwater_pump_hn annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-426,-42}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealOutput feedwater_pump_rh annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-422,-40}), iconTransformation(extent={{-174,14},{-154,34}})));
  Sensors_Control.WaterSteam.PressureSensor                   HP_pump_P_out_sensor(
    Q_0=1500,
    P_0=5900000,
    h_0=340e3,
    sensor_function="Calibration",
    causality="feedwater_pump_hn")                                                 annotation (Placement(transformation(extent={{7,-7},{-7,7}}, origin={-490,-60})));
  Sensors_Control.WaterSteam.TemperatureSensor                   HP_pump_T_out_sensor(
    Q_0=1500,
    P_0=5900000,
    h_0=340e3,
    sensor_function="Calibration",
    causality="feedwater_pump_rh",
    T_0=353.15)                                                                       annotation (Placement(transformation(extent={{-453,
            -67},{-467,-53}})));
  Utilities.Interfaces.RealInput HP_pump_P_out(start=59) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-490,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput HP_pump_T_out(start=80) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-460,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.HeatExchangers.Reheater                           HP_heater(
    Q_cold_0=1500,
    Q_hot_0=387,
    P_cold_in_0=5900000,
    P_cold_out_0=5800000,
    P_hot_in_0=3100000,
    P_hot_out_0=3100000,
    T_cold_in_0=353.15,
    T_cold_out_0=483.15,
    T_hot_in_0=508.85,
    T_hot_out_0=363.15,
    h_cold_in_0=340e3,
    h_cold_out_0=0.9e6,
    h_hot_in_0=2.55e6,
    h_hot_out_0=379e3)                                                                                annotation (Placement(transformation(extent={{-544,
            -68},{-576,-52}})));
  Utilities.Interfaces.RealOutput HP_heater_Kfr_cold annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-566,-42}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealOutput HP_heater_Kth_subc annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-548,-38}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealOutput HP_heater_Kth_cond annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-554,-40}), iconTransformation(extent={{-174,14},{-154,34}})));
    WaterSteam.BoundaryConditions.Source rh_hot_in annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-560,-12})));
  Sensors_Control.WaterSteam.FlowSensor                   Q_feedwater_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6)                                                               annotation (Placement(transformation(extent={{-663,
            -67},{-677,-53}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   HP_heater_T_out_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6,
    sensor_function="Calibration",
    causality="HP_heater_Kth_subc")                                                     annotation (Placement(transformation(extent={{-643,
            -67},{-657,-53}})));
  Sensors_Control.WaterSteam.PressureSensor                   HP_heater_P_out_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6,
    sensor_function="Calibration",
    causality="HP_heater_Kfr_cold")                                                  annotation (Placement(transformation(extent={{-623,
            -67},{-637,-53}})));
  Utilities.Interfaces.RealInput HP_heater_P_out(start=50) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-630,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput HP_heater_T_out(start=100) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-650,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealOutput Q_feedwater annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-670,-44}), iconTransformation(extent={{-734,-82},{-714,-62}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   HP_heater_T_drains_sensor(
    Q_0=387,
    P_0=3100000,
    h_0=379e3,
    sensor_function="Calibration",
    causality="HP_heater_Kth_cond",
    T_0=363.15)                                                                            annotation (Placement(transformation(
        extent={{7,7},{-7,-7}},
        rotation=90,
        origin={-560,-86})));
  Utilities.Interfaces.RealInput HP_heater_T_drains(start=90)  annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-544,-86}), iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.Pipes.ControlValve                           HP_reheater_drains_control_valve annotation (Placement(transformation(extent={{-485,
            -102.182},{-475,-90.182}})));
  Sensors_Control.Outline.OpeningSensor                   HP_reheater_drains_control_valve_opening_sensor(
      output_signal_unit="%")                                                                             annotation (Placement(transformation(extent={{-485,
            -91},{-475,-81}})));
  Utilities.Interfaces.RealOutput HP_heater_drains_control_valve_Cvmax
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-490,-80}), iconTransformation(extent={{-328,-88},{-308,-68}})));
  Utilities.Interfaces.RealInput HP_reheater_drains_control_valve_opening(start=15)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-480,-74}), iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.HeatExchangers.SteamGenerator                           steam_generator annotation (Placement(transformation(extent={{-762,
            -106},{-718,-14}})));
  Sensors_Control.WaterSteam.FlowSensor                   Q_purge_sensor(
    Q_0=5,
    h_0=1154502,
    sensor_function="BC")                                                annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-740,-122})));
  Sensors_Control.Power.PowerSensor                   thermal_power_sensor(
      sensor_function="BC")                                                annotation (Placement(transformation(extent={{-782,
            -68},{-766,-52}})));
  Sensors_Control.WaterSteam.PressureSensor                   P_steam_sensor(
    Q_0=1500,
    P_0=5000000,
    h_0=2.778e6,
    sensor_function="BC")                                                    annotation (Placement(transformation(
        extent={{-7,7},{7,-7}},
        rotation=90,
        origin={-740,20})));
  Power.BoundaryConditions.Source                           source annotation (Placement(transformation(extent={{-808,
            -70},{-788,-50}})));
  WaterSteam.BoundaryConditions.Sink                           sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-740,-140})));
  Utilities.Interfaces.RealInput thermal_power(start=2820, nominal=1e3)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-774,-42}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput P_steam(start=50) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-724,20}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput Q_purge(start=5) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-722,-122}), iconTransformation(extent={{-170,22},{-130,62}})));
  WaterSteam.BoundaryConditions.Sink                           sink2
                                                                    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-740,48})));
equation
  // BC LOCALES
  // steam_source.Q_out = -1300;
  steam_source.P_out = 1;

  rh_hot_in.T_out = 1;
  rh_hot_in.P_out = 1;

  // Parameters
  condenser.C_incond = 0;

  // Hypotheses
  steam_generator.P_purge = P_steam * 1e5;
  steam_generator.vapor_fraction = 0.99;


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
  connect(condenser_Q_cold, condenser.Qv_cold_in) annotation (Line(points={{-6,48},
          {-6,27.1605},{2.95,27.1605}}, color={0,0,127}));
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
  connect(extraction_pump.C_in, condenser.C_hot_out) annotation (Line(points={{-24,
          -60},{20,-60},{20,8.5432}}, color={28,108,200}));
  connect(extraction_pump_T_out_sensor.T_sensor, extraction_pump_T_out)
    annotation (Line(points={{-60,-53},{-60,-44}}, color={0,0,127}));
  connect(extraction_pump_P_out_sensor.P_sensor, extraction_pump_P_out)
    annotation (Line(points={{-80,-53},{-80,-44}}, color={0,0,127}));
  connect(extraction_pump_hn, extraction_pump_hn)
    annotation (Line(points={{-22,-44},{-22,-44}}, color={0,0,127}));
  connect(extraction_pump.hn, extraction_pump_hn) annotation (Line(points={{-26.88,
          -53.6},{-22,-53.6},{-22,-44}}, color={0,0,127}));
  connect(extraction_pump.rh, extraction_pump_rh) annotation (Line(points={{-25.12,
          -55.2},{-14,-55.2},{-14,-42}}, color={0,0,127}));
  connect(P_cond_sensor.C_in, LPT2.C_out)
    annotation (Line(points={{-88,120},{-127,120}}, color={28,108,200}));
  connect(LPT2.Cst, LPT2_Cst) annotation (Line(points={{-144.1,125.6},{-144,125.6},
          {-144,140}}, color={0,0,127}));
  connect(LPT2_Cst, LPT2_Cst)
    annotation (Line(points={{-144,140},{-144,140}}, color={0,0,127}));
  connect(turbines_eta_is, LPT2.eta_is) annotation (Line(points={{-210,152},{-210,
          132},{-142,132},{-142,126.08},{-142.3,126.08}},
                                    color={0,0,127}));
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
  connect(turbines_eta_is, turbines_eta_is)
    annotation (Line(points={{-210,152},{-210,152}}, color={0,0,127}));
  connect(LP_heater.C_cold_out,LP_heater_P_out_sensor. C_in)
    annotation (Line(points={{-216,-60},{-223,-60}}, color={28,108,200}));
  connect(LP_heater_P_out_sensor.C_out,LP_heater_T_out_sensor. C_in)
    annotation (Line(points={{-237,-60},{-253,-60}}, color={28,108,200}));
  connect(LP_heater_P_out_sensor.P_sensor,LP_heater_P_out)
    annotation (Line(points={{-230,-53},{-230,-44}}, color={0,0,127}));
  connect(LP_heater_T_out_sensor.T_sensor,LP_heater_T_out)
    annotation (Line(points={{-260,-53},{-260,-44}}, color={0,0,127}));
  connect(LP_heater.C_cold_in, extraction_pump_P_out_sensor.C_out)
    annotation (Line(points={{-183.8,-60},{-87,-60}}, color={28,108,200}));
  connect(LP_heater.Kfr_cold, LP_heater_Kfr_cold)
    annotation (Line(points={{-194,-50},{-194,-42}}, color={0,0,127}));
  connect(LP_heater.Kfr_cold, LP_heater_Kfr_cold)
    annotation (Line(points={{-194,-50},{-194,-42}}, color={0,0,127}));
  connect(LP_heater.Kth, LP_heater_Kth)
    annotation (Line(points={{-188,-50},{-188,-40}}, color={0,0,127}));
  connect(LP_reheater_drains_control_valve.Opening,
    LP_reheater_drains_control_valve_opening_sensor.                                                Opening) annotation (Line(points={{-140,
          -91.2729},{-140,-87.1}},                                                                                                                                  color={0,0,127}));
  connect(LP_heater.C_hot_out, LP_reheater_drains_control_valve.C_in)
    annotation (Line(points={{-200,-68},{-200,-100},{-145,-100}}, color={28,108,
          200}));
  connect(LP_reheater_drains_control_valve_opening_sensor.opening_sensor,
    LP_reheater_drains_control_valve_opening)
    annotation (Line(points={{-140,-76.9},{-140,-70}}, color={0,0,127}));
  connect(LP_heater_drains_control_valve_Cvmax,
    LP_reheater_drains_control_valve.Cv_max) annotation (Line(points={{-150,-78},
          {-150,-94.0002},{-142,-94.0002}},     color={0,0,127}));
  connect(LP_heater_drains_control_valve_Cvmax,
    LP_heater_drains_control_valve_Cvmax)
    annotation (Line(points={{-150,-78},{-150,-78}}, color={0,0,127}));
  connect(LP_reheater_drains_control_valve.C_out, condenser.C_hot_in)
    annotation (Line(points={{-135,-100},{120,-100},{120,100},{20,100},{20,
          34.6074}},
        color={28,108,200}));
  connect(LP_extract_P_sensor.C_out, LP_heater.C_hot_in)
    annotation (Line(points={{-200,85},{-200,-52}}, color={28,108,200}));
  connect(steam_source.C_out, superheater_T_out_sensor.C_in)
    annotation (Line(points={{-387,120},{-346,120}}, color={28,108,200}));
  connect(superheater_T_out_sensor.C_out, LPT1.C_in)
    annotation (Line(points={{-334,120},{-281,120}}, color={28,108,200}));
  connect(LPT1.C_out, LP_extract.C_in)
    annotation (Line(points={{-263,120},{-210.6,120}}, color={28,108,200}));
  connect(LPT1.C_W_out, generator.C_in) annotation (Line(points={{-263,126.72},{
          -252,126.72},{-252,160},{-100.4,160}}, color={244,125,35}));
  connect(LPT1.Cst, LPT1_Cst) annotation (Line(points={{-280.1,125.6},{-280.1,126},
          {-280,126},{-280,140}}, color={0,0,127}));
  connect(superheater_T_out_sensor.T_sensor, superheater_T_out)
    annotation (Line(points={{-340,126},{-340,136}}, color={0,0,127}));
  connect(LP_heater_T_out_sensor.C_out, deaerator_inlet_pipe.C_out)
    annotation (Line(points={{-267,-60},{-300,-60}}, color={28,108,200}));
  connect(deaerator_inlet_pipe_delta_z.y, deaerator_inlet_pipe.delta_z)
    annotation (Line(points={{-315,-46},{-310,-46},{-310,-56}}, color={0,0,127}));
  connect(LPT1.eta_is, turbines_eta_is) annotation (Line(points={{-278.3,126.08},
          {-278,126.08},{-278,132},{-210,132},{-210,152}},
                                                      color={0,0,127}));
  connect(condenser_Kfr_cold.y, condenser.Kfr_cold) annotation (Line(points={{-7,
          60},{-2,60},{-2,31.4568},{2.95,31.4568}}, color={0,0,127}));
  connect(deaerator_inlet_pipe.C_in, deaerator_outlet_pipe.C_out)
    annotation (Line(points={{-320,-60},{-366,-60}}, color={28,108,200}));
  connect(deaerator_outlet_pipe_delta_z.y, deaerator_outlet_pipe.delta_z)
    annotation (Line(points={{-381,-46},{-376,-46},{-376,-56}}, color={0,0,127}));
  connect(deaerator_outlet_pipe.C_in, feedwater_pump.C_in)
    annotation (Line(points={{-386,-60},{-426,-60}}, color={28,108,200}));
  connect(feedwater_pump_hn, feedwater_pump_hn)
    annotation (Line(points={{-426,-42},{-426,-42}}, color={0,0,127}));
  connect(feedwater_pump.rh, feedwater_pump_rh) annotation (Line(points={{-427.12,
          -55.2},{-422,-55.2},{-422,-40}}, color={0,0,127}));
  connect(feedwater_pump_rh, feedwater_pump_rh)
    annotation (Line(points={{-422,-40},{-422,-40}}, color={0,0,127}));
  connect(feedwater_pump.hn, feedwater_pump_hn) annotation (Line(points={{-428.88,
          -53.6},{-426,-53.6},{-426,-42}}, color={0,0,127}));
  connect(feedwater_pump.C_out, HP_pump_T_out_sensor.C_in)
    annotation (Line(points={{-442,-60},{-453,-60}}, color={28,108,200}));
  connect(HP_pump_T_out_sensor.C_out, HP_pump_P_out_sensor.C_in)
    annotation (Line(points={{-467,-60},{-483,-60}}, color={28,108,200}));
  connect(HP_pump_P_out_sensor.P_sensor, HP_pump_P_out)
    annotation (Line(points={{-490,-53},{-490,-44}}, color={0,0,127}));
  connect(HP_pump_T_out_sensor.T_sensor, HP_pump_T_out)
    annotation (Line(points={{-460,-53},{-460,-44}}, color={0,0,127}));
  connect(HP_pump_P_out_sensor.C_out, HP_heater.C_cold_in)
    annotation (Line(points={{-497,-60},{-543.8,-60}}, color={28,108,200}));
  connect(HP_heater.Kfr_cold, HP_heater_Kfr_cold)
    annotation (Line(points={{-566,-50},{-566,-42}}, color={0,0,127}));
  connect(HP_heater.Kth_subc, HP_heater_Kth_subc)
    annotation (Line(points={{-548,-50},{-548,-38}}, color={0,0,127}));
  connect(HP_heater.Kth_cond, HP_heater_Kth_cond)
    annotation (Line(points={{-554,-50},{-554,-40}}, color={0,0,127}));
  connect(HP_heater_Kfr_cold, HP_heater_Kfr_cold)
    annotation (Line(points={{-566,-42},{-566,-42}}, color={0,0,127}));
  connect(rh_hot_in.C_out, HP_heater.C_hot_in)
    annotation (Line(points={{-560,-17},{-560,-52}}, color={28,108,200}));
  connect(HP_heater_P_out_sensor.C_out,HP_heater_T_out_sensor. C_in) annotation (Line(points={{-637,
          -60},{-643,-60}},                                                                                                                     color={28,108,200}));
  connect(HP_heater_T_out_sensor.C_out,Q_feedwater_sensor. C_in) annotation (Line(points={{-657,
          -60},{-663,-60}},                                                                                      color={28,108,200}));
  connect(HP_heater_P_out_sensor.C_in, HP_heater.C_cold_out)
    annotation (Line(points={{-623,-60},{-576,-60}}, color={28,108,200}));
  connect(HP_heater_P_out_sensor.P_sensor, HP_heater_P_out)
    annotation (Line(points={{-630,-53},{-630,-44}}, color={0,0,127}));
  connect(HP_heater_T_out_sensor.T_sensor, HP_heater_T_out)
    annotation (Line(points={{-650,-53},{-650,-44}}, color={0,0,127}));
  connect(Q_feedwater_sensor.Q_sensor, Q_feedwater)
    annotation (Line(points={{-670,-53},{-670,-44}}, color={0,0,127}));
  connect(HP_heater.C_hot_out, HP_heater_T_drains_sensor.C_in)
    annotation (Line(points={{-560,-68},{-560,-79}}, color={28,108,200}));
  connect(HP_heater_T_drains_sensor.T_sensor, HP_heater_T_drains)
    annotation (Line(points={{-553,-86},{-544,-86}}, color={0,0,127}));
  connect(HP_reheater_drains_control_valve.Opening,
    HP_reheater_drains_control_valve_opening_sensor.                                                Opening) annotation (Line(points={{-480,
          -91.2729},{-480,-91.1}},                                                                                                                                color={0,0,127}));
  connect(HP_reheater_drains_control_valve.Cv_max,
    HP_heater_drains_control_valve_Cvmax) annotation (Line(points={{-482,
          -94.0002},{-490,-94.0002},{-490,-80}},
                                       color={0,0,127}));
  connect(HP_reheater_drains_control_valve_opening_sensor.opening_sensor,
    HP_reheater_drains_control_valve_opening)
    annotation (Line(points={{-480,-80.9},{-480,-74}}, color={0,0,127}));
  connect(HP_heater_T_drains_sensor.C_out, HP_reheater_drains_control_valve.C_in)
    annotation (Line(points={{-560,-93},{-560,-100},{-485,-100}}, color={28,108,
          200}));
  connect(HP_reheater_drains_control_valve.C_out, deaerator_outlet_pipe.C_out)
    annotation (Line(points={{-475,-100},{-340,-100},{-340,-60},{-366,-60}},
        color={28,108,200}));
  connect(steam_generator.purge_outlet,Q_purge_sensor. C_in) annotation (Line(
        points={{-740,-105.233},{-740,-115}}, color={28,108,200}));
  connect(Q_purge_sensor.C_out,sink. C_in)
    annotation (Line(points={{-740,-129},{-740,-135}}, color={28,108,200}));
  connect(source.C_out,thermal_power_sensor. C_in)
    annotation (Line(points={{-793.2,-60},{-782,-60}}, color={244,125,35}));
  connect(thermal_power_sensor.C_out,steam_generator. C_thermal_power)
    annotation (Line(points={{-766.16,-60},{-751,-60}},
        color={244,125,35}));
  connect(thermal_power_sensor.W_sensor,thermal_power)
    annotation (Line(points={{-774,-52},{-774,-42}}, color={0,0,127}));
  connect(P_steam_sensor.C_in,steam_generator. steam_outlet)
    annotation (Line(points={{-740,13},{-740,-14}}, color={28,108,200}));
  connect(steam_generator.feedwater_inlet, Q_feedwater_sensor.C_out)
    annotation (Line(points={{-729,-60},{-677,-60}}, color={28,108,200}));
  connect(P_steam_sensor.P_sensor, P_steam)
    annotation (Line(points={{-733,20},{-724,20}}, color={0,0,127}));
  connect(Q_purge_sensor.Q_sensor,Q_purge)
    annotation (Line(points={{-733,-122},{-722,-122}}, color={0,0,127}));
  connect(P_steam_sensor.C_out, sink2.C_in)
    annotation (Line(points={{-740,27},{-740,43}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-740,-120},
            {140,180}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-740,-120},{140,180}})));
end MetroscopiaNPP_reverse_EG_3;
