within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model MetroscopiaNPP_reverse_EG_bis
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
        origin={4,54}), iconTransformation(extent={{-160,20},{-140,40}})));
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
        origin={-144,134}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealOutput turbines_eta_is annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-370,154}), iconTransformation(extent={{-174,14},{-154,34}})));
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
        origin={-280,134}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealInput superheater_T_out(start=228) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-340,136}), iconTransformation(extent={{-170,22},{-130,62}})));
  WaterSteam.Pipes.HeightVariationPipe deaerator_inlet_pipe
    annotation (Placement(transformation(extent={{-380,-70},{-400,-50}})));
  WaterSteam.Pipes.HeightVariationPipe deaerator_outlet_pipe
    annotation (Placement(transformation(extent={{-446,-70},{-466,-50}})));
    WaterSteam.Machines.FixedSpeedPump                 feedwater_pump(
    T_in_0=350.05,
    T_out_0=353.15,
    P_in_0=600000,
    P_out_0=5900000,
    h_in_0=322e3,
    h_out_0=340e3,
    Q_0=1500)                                                                                         annotation (Placement(transformation(extent={{-506,
            -68},{-522,-52}})));
  Utilities.Interfaces.RealOutput feedwater_pump_hn annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-506,-42}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealOutput feedwater_pump_rh annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-502,-40}), iconTransformation(extent={{-174,14},{-154,34}})));
  Sensors_Control.WaterSteam.PressureSensor                   HP_pump_P_out_sensor(
    Q_0=1500,
    P_0=5900000,
    h_0=340e3,
    sensor_function="Calibration",
    causality="feedwater_pump_hn")                                                 annotation (Placement(transformation(extent={{7,-7},{-7,7}}, origin={-570,-60})));
  Sensors_Control.WaterSteam.TemperatureSensor                   HP_pump_T_out_sensor(
    Q_0=1500,
    P_0=5900000,
    h_0=340e3,
    sensor_function="Calibration",
    causality="feedwater_pump_rh",
    T_0=353.15)                                                                       annotation (Placement(transformation(extent={{-533,
            -67},{-547,-53}})));
  Utilities.Interfaces.RealInput HP_pump_P_out(start=59) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-570,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput HP_pump_T_out(start=80) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-540,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
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
    h_hot_out_0=379e3)                                                                                annotation (Placement(transformation(extent={{-624,
            -68},{-656,-52}})));
  Utilities.Interfaces.RealOutput HP_heater_Kfr_cold annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-646,-42}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealOutput HP_heater_Kth_subc annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-628,-38}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealOutput HP_heater_Kth_cond annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-634,-40}), iconTransformation(extent={{-174,14},{-154,34}})));
  Sensors_Control.WaterSteam.FlowSensor                   Q_feedwater_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6)                                                               annotation (Placement(transformation(extent={{-743,
            -67},{-757,-53}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   HP_heater_T_out_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6,
    sensor_function="Calibration",
    causality="HP_heater_Kth_subc")                                                     annotation (Placement(transformation(extent={{-723,
            -67},{-737,-53}})));
  Sensors_Control.WaterSteam.PressureSensor                   HP_heater_P_out_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6,
    sensor_function="Calibration",
    causality="HP_heater_Kfr_cold")                                                  annotation (Placement(transformation(extent={{-703,
            -67},{-717,-53}})));
  Utilities.Interfaces.RealInput HP_heater_P_out(start=50) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-710,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput HP_heater_T_out(start=100) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-730,-44}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealOutput Q_feedwater(start=1200)
                                              annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-750,-44}), iconTransformation(extent={{-734,-82},{-714,-62}})));
  Sensors_Control.WaterSteam.TemperatureSensor                   HP_heater_T_drains_sensor(
    Q_0=387,
    P_0=3100000,
    h_0=379e3,
    sensor_function="Calibration",
    causality="HP_heater_Kth_cond",
    T_0=363.15)                                                                            annotation (Placement(transformation(
        extent={{7,7},{-7,-7}},
        rotation=90,
        origin={-640,-86})));
  Utilities.Interfaces.RealInput HP_heater_T_drains(start=90)  annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-624,-86}), iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.Pipes.ControlValve                           HP_reheater_drains_control_valve annotation (Placement(transformation(extent={{-565,
            -102.182},{-555,-90.182}})));
  Sensors_Control.Outline.OpeningSensor                   HP_reheater_drains_control_valve_opening_sensor(
      output_signal_unit="%")                                                                             annotation (Placement(transformation(extent={{-565,
            -91},{-555,-81}})));
  Utilities.Interfaces.RealOutput HP_heater_drains_control_valve_Cvmax
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-570,-80}), iconTransformation(extent={{-328,-88},{-308,-68}})));
  Utilities.Interfaces.RealInput HP_reheater_drains_control_valve_opening(start=15)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-560,-74}), iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.HeatExchangers.SteamGenerator                           steam_generator annotation (Placement(transformation(extent={{-842,
            -106},{-798,-14}})));
  Sensors_Control.WaterSteam.FlowSensor                   Q_purge_sensor(
    Q_0=5,
    h_0=1154502,
    sensor_function="BC")                                                annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-820,-122})));
  Sensors_Control.Power.PowerSensor                   thermal_power_sensor(
      sensor_function="BC")                                                annotation (Placement(transformation(extent={{-862,
            -68},{-846,-52}})));
  Sensors_Control.WaterSteam.PressureSensor                   P_steam_sensor(
    Q_0=1500,
    P_0=5000000,
    h_0=2.778e6,
    sensor_function="BC")                                                    annotation (Placement(transformation(
        extent={{-7,7},{7,-7}},
        rotation=90,
        origin={-820,10})));
  Power.BoundaryConditions.Source                           source annotation (Placement(transformation(extent={{-888,
            -70},{-868,-50}})));
  WaterSteam.BoundaryConditions.Sink                           sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-820,-140})));
  Utilities.Interfaces.RealInput thermal_power(start=2820, nominal=1e3)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-854,-42}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput P_steam(start=50) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-804,10}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput Q_purge(start=5) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-802,-122}), iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.Pipes.ControlValve                           HP_control_valve(
    T_out_0=535.15,
    P_in_0=5000000,
    P_out_0=4850000,
    h_in_0=2.8e6,
    h_out_0=2.8e6,
    Q_0=1455,
    T_0=536.15,
    h_0=2.8e6)                                                                                            annotation (Placement(transformation(extent={{-785,
            37.818},{-775,49.818}})));
  Sensors_Control.Outline.OpeningSensor                   HP_control_valve_opening_sensor(
      sensor_function="Calibration", causality="HP_control_valve_Cvmax")                  annotation (Placement(transformation(extent={{-785,51},
            {-775,61}})));
  Sensors_Control.WaterSteam.PressureSensor                   HPT_P_in_sensor(
    Q_0=1455,
    P_0=4850000,
    h_0=2.8e6,
    sensor_function="Calibration",
    causality="HPT1_Cst")                                                     annotation (Placement(transformation(extent={{-734,34},
            {-722,46}})));
  Utilities.Interfaces.RealOutput HP_control_valve_Cvmax annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-790,56}),  iconTransformation(extent={{-328,-88},{-308,-68}})));
  Utilities.Interfaces.RealInput HP_control_valve_opening(start=15) annotation
    (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-780,68}),  iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput HPT_P_in(start=48.5) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-728,56}),  iconTransformation(extent={{-170,22},{-130,62}})));
  WaterSteam.Machines.SteamTurbine                           HPT_1(
    T_in_0=535.15,
    T_out_0=608.85,
    P_in_0=4850000,
    P_out_0=3100000,
    h_in_0=2.8e6,
    h_out_0=2.7e6,
    Q_0=1455) annotation (Placement(transformation(extent={{-697,32},{-679,48}})));
  Utilities.Interfaces.RealOutput HPT1_Cst annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-696,56}),  iconTransformation(extent={{-328,-88},{-308,-68}})));
  Sensors_Control.WaterSteam.PressureSensor                   HP_extract_P_sensor(
    Q_0=340,
    P_0=3100000,
    h_0=2.73e6,
    sensor_function="Calibration",
    causality="HPT2_Cst")                                                         annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-640,8})));
  Utilities.Interfaces.RealInput HPT_extract_P(start=31) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-624,8}),  iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.Pipes.SteamExtractionSplitter                           HP_extract(
    Q_in_0=1455,
    Q_ext_0=340,
    P_0=3100000,
    T_0=508.85,
    h_0=2.73e6)                                                                   annotation (Placement(transformation(extent={{-650,30},
            {-630,48}})));
  WaterSteam.Machines.SteamTurbine                           HPT_2(
    T_in_0=508.85,
    T_out_0=484.15,
    P_in_0=3100000,
    P_out_0=1940000,
    h_in_0=2.73e6,
    h_out_0=2.68e6,
    Q_0=1113) annotation (Placement(transformation(extent={{-587,32},{-569,48}})));
  Sensors_Control.WaterSteam.PressureSensor                   HPT_P_out_sensor(
    Q_0=1113,
    P_0=1940000,
    h_0=2.68e6,
    sensor_function="Calibration",
    causality="LPT1_Cst")                                                      annotation (Placement(transformation(extent={{-466,34},
            {-454,46}})));
  Utilities.Interfaces.RealInput HPT_extract_P1(start=19.4)
                                                         annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-460,56}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealOutput HPT2_Cst annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-586,58}),  iconTransformation(extent={{-328,-88},{-308,-68}})));
    WaterSteam.Volumes.SteamDryer                           steam_dryer(
    P_0=1940000,
    T_0=483.95,
    h_in_0=2.68e6,
    Q_in_0=1113,
    Q_liq_0=50)                                                         annotation (Placement(transformation(extent={{-436,
            27.8182},{-420,45.8182}})));
    WaterSteam.Pipes.PressureCut steam_dryer_liq_out_pipe(
    P_in_0=4000000,
    P_out_0=3100000,
    Q_0=44,
    T_0=525.15,
    h_0=1.09e6) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-420,0})));
  WaterSteam.HeatExchangers.Superheater superheater
    annotation (Placement(transformation(extent={{-416,72},{-384,88}})));
  Utilities.Interfaces.RealOutput superheater_Kth annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-412,102}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealOutput superheater_Kfr_hot annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-394,102}), iconTransformation(extent={{-174,14},{-154,34}})));
  Sensors_Control.WaterSteam.PressureSensor                   superheater_bleed_P_sensor(
    Q_0=45,
    P_0=4100000,
    h_0=2.778e6,
    sensor_function="Calibration",
    causality="superheater_control_valve_Cv_max")                                        annotation (Placement(transformation(extent={{-746,74},
            {-734,86}})));
  Utilities.Interfaces.RealInput superheater_bleed_P(start=41) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-740,98}), iconTransformation(extent={{-170,22},{-130,62}})));
  WaterSteam.Pipes.LoopBreaker                           loopBreaker annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-782,-60})));
  Sensors_Control.WaterSteam.PressureSensor                   superheater_drains_P_sensor(
    Q_0=44,
    P_0=4000000,
    h_0=1.09e6,
    sensor_function="Calibration",
    causality="supearheater_Kfr_cold")                                                    annotation (Placement(transformation(extent={{-368,74},
            {-356,86}})));
  Utilities.Interfaces.RealInput superheater_drains_P(start=40) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-362,94}), iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.Pipes.PressureCut pressureCut(
    P_in_0=4000000,
    P_out_0=3100000,
    Q_0=44,
    T_0=525.15,
    h_0=1.09e6) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-362,40})));
    WaterSteam.Pipes.PressureCut superheater_drains_pipe(
    P_in_0=4000000,
    P_out_0=3100000,
    Q_0=44,
    T_0=525.15,
    h_0=1.09e6) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-340,60})));
  Utilities.Interfaces.RealInput steam_generator_vapor_fraction(start=0.99)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-790,-32}), iconTransformation(extent={{-174,14},{-154,34}})));
  Utilities.Interfaces.RealInput superheater_Kfr_cold(start=0) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-406,102}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput deaerator_outlet_pipe_delta_z(start=-5)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-456,-46}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput deaerator_inlet_pipe_delta_z(start=-5)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-390,-46}), iconTransformation(extent={{-170,22},{-130,62}})));
  Utilities.Interfaces.RealInput condenser_Kfr_cold(start=0) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={0,52}), iconTransformation(extent={{-170,22},{-130,62}})));
    WaterSteam.Pipes.ControlValve                           superheater_control_valve(
    P_in_0=5000000,
    P_out_0=4100000,
    h_in_0=2.778e6,
    h_out_0=2.778e6,
    Q_0=45,
    T_0=537.15,
    h_0=2.8e6)                                                                        annotation (Placement(transformation(extent={{-777,
            77.8182},{-767,89.8182}})));
  Utilities.Interfaces.RealOutput superheater_control_valve_Cv_max annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-780,94}), iconTransformation(extent={{-328,-88},{-308,-68}})));
equation
  // Parameters
  condenser.C_incond = 0;

  // Hypotheses
  superheater_control_valve.Opening = 1;

  connect(superheater_control_valve.C_in, P_steam_sensor.C_out) annotation (
      Line(points={{-777,80},{-800,80},{-800,40},{-820,40},{-820,17}}, color={28,
          108,200}));
  connect(LP_reheater_drains_control_valve.C_out, condenser.C_hot_in)
    annotation (Line(points={{-135,-100},{120,-100},{120,100},{20,100},{20,
          34.6074}},
        color={28,108,200}));
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
  connect(condenser_Kth, condenser.Kth) annotation (Line(points={{4,54},{4,35.7531},
          {7.6,35.7531}}, color={0,0,127}));
  connect(condenser.C_hot_in, P_cond_sensor.C_out) annotation (Line(points={{20,
          34.6074},{20,120},{-76,120}}, color={28,108,200},
      thickness=1));
  connect(P_cond_sensor.P_sensor, P_cond)
    annotation (Line(points={{-82,126},{-82,136}}, color={0,0,127}));
  connect(extraction_pump.C_out,extraction_pump_T_out_sensor. C_in) annotation (Line(points={{-40,-60},
          {-53,-60}},                                                                                              color={28,108,200},
      thickness=1));
  connect(extraction_pump_T_out_sensor.C_out,extraction_pump_P_out_sensor. C_in) annotation (Line(points={{-67,-60},
          {-73,-60}},                                                                                                           color={28,108,200},
      thickness=1));
  connect(extraction_pump.C_in, condenser.C_hot_out) annotation (Line(points={{-24,
          -60},{20,-60},{20,8.5432}}, color={28,108,200},
      thickness=1));
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
    annotation (Line(points={{-88,120},{-127,120}}, color={28,108,200},
      thickness=1));
  connect(LPT2.Cst, LPT2_Cst) annotation (Line(points={{-144.1,125.6},{-144,125.6},
          {-144,134}}, color={0,0,127}));
  connect(LPT2_Cst, LPT2_Cst)
    annotation (Line(points={{-144,134},{-144,134}}, color={0,0,127}));
  connect(turbines_eta_is, LPT2.eta_is) annotation (Line(points={{-370,154},{-370,
          140},{-140,140},{-140,126.08},{-142.3,126.08}},
                                    color={0,0,127}));
  connect(powerSink.C_in,W_elec_sensor. C_out) annotation (Line(points={{-49,160},
          {-56.12,160}},                                                                      color={244,125,35}));
  connect(W_elec_sensor.C_in,generator. C_out) annotation (Line(points={{-68,160},
          {-74,160}},                                                                      color={244,125,35}));
  connect(LPT2.C_W_out, generator.C_in) annotation (Line(points={{-127,126.72},{
          -120,126.72},{-120,160},{-100.4,160}}, color={244,125,35},
      smooth=Smooth.Bezier));
  connect(W_elec_sensor.W_sensor, W_elec)
    annotation (Line(points={{-62,166},{-62,174}}, color={0,0,127}));
  connect(LPT2.C_in,LP_extract. C_main_out)
    annotation (Line(points={{-145,120},{-189.4,120}}, color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(LP_extract_P_sensor.P_sensor,LP_extract_P)
    annotation (Line(points={{-193,92},{-180,92}}, color={0,0,127}));
  connect(LP_extract.C_ext_out,LP_extract_P_sensor. C_in)
    annotation (Line(points={{-200,113.2},{-200,99}}, color={28,108,200}));
  connect(turbines_eta_is, turbines_eta_is)
    annotation (Line(points={{-370,154},{-370,154}}, color={0,0,127}));
  connect(LP_heater.C_cold_out,LP_heater_P_out_sensor. C_in)
    annotation (Line(points={{-216,-60},{-223,-60}}, color={28,108,200},
      thickness=1));
  connect(LP_heater_P_out_sensor.C_out,LP_heater_T_out_sensor. C_in)
    annotation (Line(points={{-237,-60},{-253,-60}}, color={28,108,200},
      thickness=1));
  connect(LP_heater_P_out_sensor.P_sensor,LP_heater_P_out)
    annotation (Line(points={{-230,-53},{-230,-44}}, color={0,0,127}));
  connect(LP_heater_T_out_sensor.T_sensor,LP_heater_T_out)
    annotation (Line(points={{-260,-53},{-260,-44}}, color={0,0,127}));
  connect(LP_heater.C_cold_in, extraction_pump_P_out_sensor.C_out)
    annotation (Line(points={{-183.8,-60},{-87,-60}}, color={28,108,200},
      thickness=1));
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
  connect(LP_extract_P_sensor.C_out, LP_heater.C_hot_in)
    annotation (Line(points={{-200,85},{-200,-52}}, color={28,108,200}));
  connect(superheater_T_out_sensor.C_out, LPT1.C_in)
    annotation (Line(points={{-334,120},{-281,120}}, color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(LPT1.C_out, LP_extract.C_in)
    annotation (Line(points={{-263,120},{-210.6,120}}, color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(LPT1.C_W_out, generator.C_in) annotation (Line(points={{-263,126.72},{
          -252,126.72},{-252,160},{-100.4,160}}, color={244,125,35},
      smooth=Smooth.Bezier));
  connect(LPT1.Cst, LPT1_Cst) annotation (Line(points={{-280.1,125.6},{-280.1,126},
          {-280,126},{-280,134}}, color={0,0,127}));
  connect(superheater_T_out_sensor.T_sensor, superheater_T_out)
    annotation (Line(points={{-340,126},{-340,136}}, color={0,0,127}));
  connect(LPT1.eta_is, turbines_eta_is) annotation (Line(points={{-278.3,126.08},
          {-278,126.08},{-278,140},{-370,140},{-370,154}},
                                                      color={0,0,127}));
  connect(feedwater_pump_hn, feedwater_pump_hn)
    annotation (Line(points={{-506,-42},{-506,-42}}, color={0,0,127}));
  connect(feedwater_pump.rh, feedwater_pump_rh) annotation (Line(points={{-507.12,
          -55.2},{-502,-55.2},{-502,-40}}, color={0,0,127}));
  connect(feedwater_pump_rh, feedwater_pump_rh)
    annotation (Line(points={{-502,-40},{-502,-40}}, color={0,0,127}));
  connect(feedwater_pump.hn, feedwater_pump_hn) annotation (Line(points={{-508.88,
          -53.6},{-506,-53.6},{-506,-42}}, color={0,0,127}));
  connect(feedwater_pump.C_out, HP_pump_T_out_sensor.C_in)
    annotation (Line(points={{-522,-60},{-533,-60}}, color={28,108,200},
      thickness=1));
  connect(HP_pump_T_out_sensor.C_out, HP_pump_P_out_sensor.C_in)
    annotation (Line(points={{-547,-60},{-563,-60}}, color={28,108,200},
      thickness=1));
  connect(HP_pump_P_out_sensor.P_sensor, HP_pump_P_out)
    annotation (Line(points={{-570,-53},{-570,-44}}, color={0,0,127}));
  connect(HP_pump_T_out_sensor.T_sensor, HP_pump_T_out)
    annotation (Line(points={{-540,-53},{-540,-44}}, color={0,0,127}));
  connect(HP_pump_P_out_sensor.C_out, HP_heater.C_cold_in)
    annotation (Line(points={{-577,-60},{-623.8,-60}}, color={28,108,200},
      thickness=1));
  connect(HP_heater.Kfr_cold, HP_heater_Kfr_cold)
    annotation (Line(points={{-646,-50},{-646,-42}}, color={0,0,127}));
  connect(HP_heater.Kth_subc, HP_heater_Kth_subc)
    annotation (Line(points={{-628,-50},{-628,-38}}, color={0,0,127}));
  connect(HP_heater.Kth_cond, HP_heater_Kth_cond)
    annotation (Line(points={{-634,-50},{-634,-40}}, color={0,0,127}));
  connect(HP_heater_Kfr_cold, HP_heater_Kfr_cold)
    annotation (Line(points={{-646,-42},{-646,-42}}, color={0,0,127}));
  connect(HP_heater_P_out_sensor.C_out,HP_heater_T_out_sensor. C_in) annotation (Line(points={{-717,
          -60},{-723,-60}},                                                                                                                     color={28,108,200},
      thickness=1));
  connect(HP_heater_T_out_sensor.C_out,Q_feedwater_sensor. C_in) annotation (Line(points={{-737,
          -60},{-743,-60}},                                                                                      color={28,108,200},
      thickness=1));
  connect(HP_heater_P_out_sensor.C_in, HP_heater.C_cold_out)
    annotation (Line(points={{-703,-60},{-656,-60}}, color={28,108,200},
      thickness=1));
  connect(HP_heater_P_out_sensor.P_sensor, HP_heater_P_out)
    annotation (Line(points={{-710,-53},{-710,-44}}, color={0,0,127}));
  connect(HP_heater_T_out_sensor.T_sensor, HP_heater_T_out)
    annotation (Line(points={{-730,-53},{-730,-44}}, color={0,0,127}));
  connect(Q_feedwater_sensor.Q_sensor, Q_feedwater)
    annotation (Line(points={{-750,-53},{-750,-44}}, color={0,0,127}));
  connect(HP_heater.C_hot_out, HP_heater_T_drains_sensor.C_in)
    annotation (Line(points={{-640,-68},{-640,-79}}, color={28,108,200}));
  connect(HP_heater_T_drains_sensor.T_sensor, HP_heater_T_drains)
    annotation (Line(points={{-633,-86},{-624,-86}}, color={0,0,127}));
  connect(HP_reheater_drains_control_valve.Opening,
    HP_reheater_drains_control_valve_opening_sensor.                                                Opening) annotation (Line(points={{-560,
          -91.2729},{-560,-91.1}},                                                                                                                                color={0,0,127}));
  connect(HP_reheater_drains_control_valve.Cv_max,
    HP_heater_drains_control_valve_Cvmax) annotation (Line(points={{-562,
          -94.0002},{-570,-94.0002},{-570,-80}},
                                       color={0,0,127}));
  connect(HP_reheater_drains_control_valve_opening_sensor.opening_sensor,
    HP_reheater_drains_control_valve_opening)
    annotation (Line(points={{-560,-80.9},{-560,-74}}, color={0,0,127}));
  connect(HP_heater_T_drains_sensor.C_out, HP_reheater_drains_control_valve.C_in)
    annotation (Line(points={{-640,-93},{-640,-100},{-565,-100}}, color={28,108,
          200}));
  connect(steam_generator.purge_outlet,Q_purge_sensor. C_in) annotation (Line(
        points={{-820,-105.233},{-820,-115}}, color={28,108,200}));
  connect(Q_purge_sensor.C_out,sink. C_in)
    annotation (Line(points={{-820,-129},{-820,-135}}, color={28,108,200}));
  connect(source.C_out,thermal_power_sensor. C_in)
    annotation (Line(points={{-873.2,-60},{-862,-60}}, color={244,125,35}));
  connect(thermal_power_sensor.C_out,steam_generator. C_thermal_power)
    annotation (Line(points={{-846.16,-60},{-831,-60}},
        color={244,125,35}));
  connect(thermal_power_sensor.W_sensor,thermal_power)
    annotation (Line(points={{-854,-52},{-854,-42}}, color={0,0,127}));
  connect(P_steam_sensor.C_in,steam_generator. steam_outlet)
    annotation (Line(points={{-820,3},{-820,-14}},  color={255,0,0},
      thickness=1));
  connect(P_steam_sensor.P_sensor, P_steam)
    annotation (Line(points={{-813,10},{-804,10}}, color={0,0,127}));
  connect(Q_purge_sensor.Q_sensor,Q_purge)
    annotation (Line(points={{-813,-122},{-802,-122}}, color={0,0,127}));
  connect(HP_control_valve.Opening,HP_control_valve_opening_sensor. Opening)
    annotation (Line(points={{-780,48.7271},{-780,50.9}},  color={0,0,127}));
  connect(HP_control_valve_opening_sensor.opening_sensor,
    HP_control_valve_opening)
    annotation (Line(points={{-780,61.1},{-780,68}},   color={0,0,127}));
  connect(HP_control_valve_Cvmax,HP_control_valve. Cv_max) annotation (Line(
        points={{-790,56},{-790,45.9998},{-782,45.9998}},
                                                   color={0,0,127}));
  connect(HPT_P_in_sensor.P_sensor,HPT_P_in)
    annotation (Line(points={{-728,46},{-728,56}},   color={0,0,127}));
  connect(HP_control_valve_Cvmax,HP_control_valve_Cvmax)
    annotation (Line(points={{-790,56},{-790,56}},   color={0,0,127}));
  connect(HP_control_valve.C_in, P_steam_sensor.C_out) annotation (Line(points={{-785,
          39.9998},{-820,39.9998},{-820,17}},       color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(HPT_P_in_sensor.C_in,HP_control_valve. C_out) annotation (Line(points={{-734,40},
          {-754,40},{-754,39.9998},{-775,39.9998}},                 color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(HPT_1.C_in, HPT_P_in_sensor.C_out) annotation (Line(points={{-697,40},
          {-722,40}},                            color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(HPT1_Cst,HPT1_Cst)
    annotation (Line(points={{-696,56},{-696,56}},   color={0,0,127}));
  connect(HPT_1.Cst,HPT1_Cst)  annotation (Line(points={{-696.1,45.6},{-696.1,56},
          {-696,56}},             color={0,0,127}));
  connect(HPT_1.eta_is, turbines_eta_is) annotation (Line(points={{-694.3,46.08},
          {-694.3,56},{-694,56},{-694,140},{-370,140},{-370,154}},
                                               color={0,0,127}));
  connect(HPT_1.C_W_out, generator.C_in) annotation (Line(points={{-679,46.72},{
          -662,46.72},{-662,160},{-100.4,160}},   color={244,125,35},
      smooth=Smooth.Bezier));
  connect(HP_extract_P_sensor.P_sensor,HPT_extract_P)
    annotation (Line(points={{-633,8},{-624,8}},   color={0,0,127}));
  connect(HPT_1.C_out, HP_extract.C_in)
    annotation (Line(points={{-679,40},{-650.6,40}},   color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(HP_extract.C_ext_out, HP_extract_P_sensor.C_in)
    annotation (Line(points={{-640,33.2},{-640,15}},  color={28,108,200}));
  connect(HP_heater.C_hot_in, HP_extract_P_sensor.C_out) annotation (Line(
        points={{-640,-52},{-640,1}},            color={28,108,200}));
  connect(HP_extract.C_main_out,HPT_2. C_in)
    annotation (Line(points={{-629.4,40},{-587,40}},   color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(HPT_2.C_out,HPT_P_out_sensor. C_in)
    annotation (Line(points={{-569,40},{-466,40}},   color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(HPT_P_out_sensor.P_sensor,HPT_extract_P1)
    annotation (Line(points={{-460,46},{-460,56}},   color={0,0,127}));
  connect(HPT_2.Cst,HPT2_Cst)  annotation (Line(points={{-586.1,45.6},{-586.1,46},
          {-586,46},{-586,58}},            color={0,0,127}));
  connect(HPT_2.eta_is, turbines_eta_is) annotation (Line(points={{-584.3,46.08},
          {-584.3,46},{-584,46},{-584,140},{-370,140},{-370,154}},
                                                       color={0,0,127}));
  connect(HPT_2.C_W_out, generator.C_in) annotation (Line(points={{-569,46.72},{
          -542,46.72},{-542,160},{-100.4,160}},   color={244,125,35},
      smooth=Smooth.Bezier));
  connect(HPT_P_out_sensor.C_out, steam_dryer.C_in) annotation (Line(points={{-454,40},
          {-436,40},{-436,39.2727}},            color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(steam_dryer.C_hot_liquid, steam_dryer_liq_out_pipe.C_in)
    annotation (Line(points={{-420,32.7273},{-420,10}}, color={28,108,200}));
  connect(steam_dryer.C_hot_steam, superheater.C_cold_in) annotation (Line(
        points={{-420,39.2727},{-420,40},{-400,40},{-400,72}},
        color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(superheater_Kth, superheater.Kth)
    annotation (Line(points={{-412,102},{-412,90}},  color={0,0,127}));
  connect(superheater.Kfr_hot, superheater_Kfr_hot)
    annotation (Line(points={{-394,90},{-394,102}},  color={0,0,127}));
  connect(HPT2_Cst, HPT2_Cst)
    annotation (Line(points={{-586,58},{-586,58}}, color={0,0,127}));
  connect(superheater_bleed_P_sensor.P_sensor, superheater_bleed_P)
    annotation (Line(points={{-740,86},{-740,98}}, color={0,0,127}));
  connect(superheater_bleed_P_sensor.C_out, superheater.C_hot_in)
    annotation (Line(points={{-734,80},{-416,80}}, color={28,108,200}));
  connect(superheater.C_cold_out, superheater_T_out_sensor.C_in) annotation (
      Line(points={{-400,88},{-400,120},{-346,120}}, color={255,0,0},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(steam_generator.feedwater_inlet, loopBreaker.C_out)
    annotation (Line(points={{-809,-60},{-792,-60}}, color={28,108,200},
      thickness=1));
  connect(loopBreaker.C_in, Q_feedwater_sensor.C_out)
    annotation (Line(points={{-772,-60},{-757,-60}}, color={28,108,200},
      thickness=1));
  connect(superheater_drains_P_sensor.P_sensor, superheater_drains_P)
    annotation (Line(points={{-362,86},{-362,94}}, color={0,0,127}));
  connect(superheater.C_hot_out, superheater_drains_P_sensor.C_in)
    annotation (Line(points={{-384,80},{-368,80}}, color={28,108,200}));
  connect(superheater.C_vent, pressureCut.C_in) annotation (Line(points={{-384,72.2},
          {-384,40},{-372,40}}, color={28,108,200}));
  connect(pressureCut.C_out, HP_heater.C_hot_in) annotation (Line(points={{-352,
          40},{-340,40},{-340,-20},{-640,-20},{-640,-52}}, color={28,108,200}));
  connect(superheater_drains_P_sensor.C_out, superheater_drains_pipe.C_in)
    annotation (Line(points={{-356,80},{-340,80},{-340,70}}, color={28,108,200}));
  connect(superheater_drains_pipe.C_out, HP_heater.C_hot_in) annotation (Line(
        points={{-340,50},{-340,-20},{-640,-20},{-640,-52}}, color={28,108,200}));
  connect(deaerator_outlet_pipe.C_out, feedwater_pump.C_in)
    annotation (Line(points={{-466,-60},{-506,-60}}, color={28,108,200},
      thickness=1));
  connect(HP_reheater_drains_control_valve.C_out, deaerator_outlet_pipe.C_in)
    annotation (Line(points={{-555,-100},{-420,-100},{-420,-60},{-446,-60}},
        color={28,108,200}));
  connect(steam_dryer_liq_out_pipe.C_out, deaerator_outlet_pipe.C_in)
    annotation (Line(points={{-420,-10},{-420,-60},{-446,-60}}, color={28,108,200}));
  connect(deaerator_inlet_pipe.C_out, deaerator_outlet_pipe.C_in)
    annotation (Line(points={{-400,-60},{-446,-60}}, color={28,108,200},
      thickness=1));
  connect(deaerator_inlet_pipe.C_in, LP_heater_T_out_sensor.C_out)
    annotation (Line(points={{-380,-60},{-267,-60}}, color={28,108,200},
      thickness=1));
  connect(steam_generator.vapor_fraction, steam_generator_vapor_fraction)
    annotation (Line(points={{-801.3,-32.0167},{-797.466,-32.0167},{-797.466,
          -32},{-790,-32}},
                       color={0,0,127}));
  connect(superheater.Kfr_cold, superheater_Kfr_cold)
    annotation (Line(points={{-406,90},{-406,102}}, color={0,0,127}));
  connect(deaerator_outlet_pipe.delta_z, deaerator_outlet_pipe_delta_z)
    annotation (Line(points={{-456,-56},{-456,-46}}, color={0,0,127}));
  connect(deaerator_inlet_pipe.delta_z, deaerator_inlet_pipe_delta_z)
    annotation (Line(points={{-390,-56},{-390,-46}}, color={0,0,127}));
  connect(condenser_Kth, condenser_Kth)
    annotation (Line(points={{4,54},{4,54}}, color={0,0,127}));
  connect(condenser.Kfr_cold, condenser_Kfr_cold) annotation (Line(points={{2.95,
          31.4568},{0,31.4568},{0,52}}, color={0,0,127}));
  connect(superheater_control_valve.C_out, superheater_bleed_P_sensor.C_in)
    annotation (Line(points={{-767,80},{-756,80},{-756,80},{-746,80}},
                                                   color={28,108,200}));
  connect(superheater_control_valve_Cv_max, superheater_control_valve.Cv_max)
    annotation (Line(points={{-780,94},{-780,86},{-774,86}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-900,-160},
            {140,180}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-900,-160},{140,180}})));
end MetroscopiaNPP_reverse_EG_bis;
