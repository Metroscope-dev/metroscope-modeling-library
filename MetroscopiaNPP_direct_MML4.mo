within MetroscopiaNPPModel;
model MetroscopiaNPP_direct_MML4 "Direct model in version MML4"
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-900,-20},{-860,20}})));
  MetroscopeModelingLibrary.Sensors_Control.Power.PowerSensor thermal_power_sensor(
      sensor_function="BC")
    annotation (Placement(transformation(extent={{-852,-10},{-832,10}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition thermal_power(start=
        2820) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-840,20}), iconTransformation(extent={{-970,-88},{-930,-48}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor HP_heater_T_out_sensor(
    sensor_function="Calibration",
    causality="HP_heater_Kth_subc",
    T_start=210)
    annotation (Placement(transformation(extent={{-666,-10},{-686,10}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor HP_heater_P_out_sensor(
    sensor_function="Calibration",
    causality="HP_heater_Kfr_cold",
    P_start=58)
    annotation (Placement(transformation(extent={{-636,-10},{-656,10}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.FlowSensor Q_feedwater_sensor(Q_start=
        1500)
    annotation (Placement(transformation(extent={{-606,-10},{-626,10}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable Q_feedwater
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-616,20}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_heater_P_out
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-646,20}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_heater_T_out
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-676,20}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_purge(Q_in(
        start=5), Qv_in(start=0.0064317323)) annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-780,-144})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.FlowSensor Q_purge_sensor(
      sensor_function="BC", Q_start=5) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-780,-114})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition Q_purge
    annotation (Placement(transformation(extent={{-814,-118},{-806,-110}}),
        iconTransformation(extent={{-1222,-72},{-1182,-32}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor P_steam_sensor(
      sensor_function="BC", P_start=50) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-780,118})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition P_steam
    annotation (Placement(transformation(extent={{-806,114},{-798,122}}),
        iconTransformation(extent={{-1208,44},{-1168,84}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealInput steam_generator_vapor_fraction(start=
        0.99) annotation (Placement(transformation(extent={{-840,52},{-832,60}}),
        iconTransformation(extent={{-1058,78},{-1018,118}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine HPT_1(
    T_in_0=535.15,
    T_out_0=608.85,
    P_in_0=4850000,
    P_out_0=3100000,
    DP_0=-1750000,
    h_in_0=2800000.0,
    h_out_0=2700000.0,
    Q_0=1455.0,
    W(start=-64914788),
    xm_0=0.9710897,
    x_in_0=1.0)
    annotation (Placement(transformation(extent={{-600,240},{-520,320}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.SteamGenerator steamGenerator
    annotation (Placement(transformation(extent={{-822,-84},{-738,84}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine HPT_2(
    T_in_0=508.85,
    T_out_0=484.15,
    P_in_0=3100000,
    P_out_0=1940000,
    DP_0=-1160000,
    h_in_0=2730000.0,
    h_out_0=2680000.0,
    Q_0=1113.0,
    W(start=-49711764),
    xm_0=0.94844204,
    x_in_0=0.9589742)
    annotation (Placement(transformation(extent={{-398,240},{-318,320}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter HP_extract(
    Q_in_0=1455.0,
    Q_ext_0=340.0,
    Q_main_0=1115.0,
    P_0=3100000,
    T_0=508.85,
    h_0=2730000.0,
    x_0=0.9589742)
    annotation (Placement(transformation(extent={{-480,260},{-440,300}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink2
    annotation (Placement(transformation(extent={{464,380},{504,420}})));
  MetroscopeModelingLibrary.Power.Machines.Generator generator(eta=0.99)
    annotation (Placement(transformation(extent={{382,376},{462,424}})));
  MetroscopeModelingLibrary.Sensors_Control.Power.PowerSensor W_elec_sensor(
    sensor_function="Calibration",
    causality="turbines_eta_is",
    W_start=570)
    annotation (Placement(transformation(extent={{358,390},{378,410}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor HP_extract_P_sensor(
    sensor_function="Calibration",
    causality="HPT2_Cst",
    P_start=31) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-460,110})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealExpression realExpression(y=1)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-444,298}), iconTransformation(extent={{-656,182},{-616,222}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_extract_P
    annotation (Placement(transformation(extent={{-416,106},{-424,114}}),
        iconTransformation(extent={{-814,98},{-774,138}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput W_elec(start=570)
    annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=90,
        origin={368,428}), iconTransformation(extent={{-814,98},{-774,138}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor HPT_extract_P1_sensor(
    sensor_function="Calibration",
    causality="LPT1_Cst",
    P_start=19.4) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-260,280})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HPT_extract_P1
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-260,310}), iconTransformation(extent={{-804,180},{-764,220}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater reheater(
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
    h_cold_in_0=340000.0,
    h_cold_out_0=900000.0,
    h_hot_in_0=2550000.0,
    h_hot_out_0=379000.0)
    annotation (Placement(transformation(extent={{-412,-24},{-508,24}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor HP_pump_T_out_sensor(
    sensor_function="Calibration",
    causality="feedwater_pump_rh",
    T_start=80)
    annotation (Placement(transformation(extent={{-360,-10},{-380,10}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor HP_pump_P_out_sensor(
    sensor_function="Calibration",
    causality="feedwater_pump_hn",
    P_start=59)
    annotation (Placement(transformation(extent={{-330,-10},{-350,10}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_pump_P_out
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-340,20}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_pump_T_out
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-370,20}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor HP_heater_T_drains_sensor(
    sensor_function="Calibration",
    causality="HP_heater_Kth_cond",
    T_start=90) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-460,-60})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_heater_T_drains
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-486,-60}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.FixedSpeedPump feedwater_pump(hn(start=
          554), rh(start=0.17))
    annotation (Placement(transformation(extent={{-260,-20},{-300,20}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor LP_heater_T_out_sensor(
    sensor_function="Calibration",
    causality="LP_heater_Kth",
    T_start=65) annotation (Placement(transformation(extent={{26,-10},{6,10}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor LP_heater_P_out_sensor(
    sensor_function="Calibration",
    causality="LP_heater_Kfr_cold",
    P_start=5.9999995)
    annotation (Placement(transformation(extent={{56,-10},{36,10}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput LP_heater_P_out
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={46,20}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput LP_heater_T_out
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={16,20}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.WaterSteam.Volumes.SteamDryer steamDryer(
    x_steam_out=0.99,
    P_0=1940000,
    T_0=483.95,
    h_in_0=2680000.0,
    Q_in_0=1113.0,
    Q_liq_0=50.0,
    Q_vap_0=1063.0) annotation (Placement(transformation(extent={{-223,85.4545},
            {-173,139.455}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.PressureCut pressureCut
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-160,28})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.HeightVariationPipe deaerator_outlet_pipe(
    T_in_0=350.05,
    T_out_0=350.05,
    P_in_0=550000,
    P_out_0=600000,
    DP_0=50000,
    h_in_0=322000.0,
    h_out_0=322000.0,
    Q_0=1500.0,
    W(start=0),
    T_0=350.05,
    h_0=322000.0,
    delta_z(start=-5))
    annotation (Placement(transformation(extent={{-190,-10},{-210,10}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.HeightVariationPipe deaerator_inlet_pipe(
    T_in_0=338.15,
    T_out_0=338.15,
    P_in_0=600000,
    P_out_0=550000,
    DP_0=-50000,
    h_in_0=272000.0,
    h_out_0=272000.0,
    Q_0=1060.0,
    W(start=0),
    T_0=338.15,
    h_0=272000.0,
    delta_z(start=5))
    annotation (Placement(transformation(extent={{-110,-10},{-130,10}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealExpression realExpression1(y=-5)
    annotation (Placement(transformation(extent={{-210,24},{-190,44}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealExpression realExpression2(y=5)
    annotation (Placement(transformation(extent={{-130,24},{-110,44}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve controlValve1(
    T_in_0=300,
    T_out_0=300,
    P_in_0=100000,
    P_out_0=100000,
    DP_0=0,
    h_in_0=500000.0,
    h_out_0=500000.0,
    Q_0=8000.0,
    W(start=0),
    T_0=300,
    h_0=500000.0,
    Cv(start=330.2112)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-300,-114})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor superheater_T_out_sensor(
    sensor_function="Calibration",
    causality="superheater_Kth",
    T_start=228) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-80,230})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput superheater_T_out
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-100,230}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine LPT_1(
    T_in_0=501.15,
    T_out_0=425.15,
    P_in_0=1940000,
    P_out_0=500000,
    DP_0=-1440000,
    h_in_0=2850000.0,
    h_out_0=2700000.0,
    Q_0=1060.0,
    W(start=-143588900),
    xm_0=0.98858887,
    x_in_0=1.0)
    annotation (Placement(transformation(extent={{-22,240},{58,320}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter LP_extract(
    Q_in_0=1060.0,
    Q_ext_0=55,
    Q_main_0=1005.0,
    P_0=500000,
    T_0=425.15,
    h_0=2700000.0,
    x_0=0.97717774)
    annotation (Placement(transformation(extent={{100,260},{140,300}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealExpression realExpression3(y=1)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={136,300}), iconTransformation(extent={{-656,182},{-616,222}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine LPT_2(
    T_in_0=425.15,
    T_out_0=312.05,
    P_in_0=500000,
    P_out_0=6900,
    DP_0=-493100,
    h_in_0=2700000.0,
    h_out_0=2400000.0,
    Q_0=1000.0,
    W(start=-317542100),
    xm_0=0.9530399,
    x_in_0=0.97717774)
    annotation (Placement(transformation(extent={{180,240},{260,320}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor P_cond_sensor(
    sensor_function="Calibration",
    causality="condenser_Kth",
    P_start=0.0698)
    annotation (Placement(transformation(extent={{350,270},{370,290}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor LP_extract_P_sensor(
    sensor_function="Calibration",
    causality="LPT2_Cst",
    P_start=5) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={120,170})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput LP_extract_P
    annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=0,
        origin={148,170}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput P_cond annotation (
      Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=90,
        origin={360,312}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.DryReheater dryReheater(
    Q_cold_0=1060.0,
    Q_hot_0=55.0,
    P_cold_in_0=700000,
    P_cold_out_0=600000,
    P_hot_in_0=500000,
    P_hot_out_0=500000,
    T_cold_in_0=312.15,
    T_cold_out_0=338.15,
    T_hot_in_0=425.15,
    T_hot_out_0=425.15,
    h_cold_in_0=164000.0,
    h_cold_out_0=272000.0,
    h_hot_in_0=2700000.0,
    h_hot_out_0=640000.0)
    annotation (Placement(transformation(extent={{168,-24},{72,24}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor extraction_pump_T_out_sensor(
    sensor_function="Calibration",
    causality="extraction_pump_hn",
    P_start=6.9999995)
    annotation (Placement(transformation(extent={{250,-10},{230,10}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor extraction_pump_P_out_sensor(
    sensor_function="Calibration",
    causality="extraction_pump_rh",
    T_start=39)
    annotation (Placement(transformation(extent={{218,-10},{198,10}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput extraction_pump_P_out
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={208,20}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput extraction_pump_T_out
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={238,20}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve LP_reheater_drains_control_valve(
    T_in_0=425.15,
    T_out_0=425.15,
    P_in_0=500000,
    P_out_0=6900,
    DP_0=-493100,
    h_in_0=640000.0,
    h_out_0=640000.0,
    Q_0=55.0,
    W(start=0),
    T_0=425.15,
    h_0=640000.0,
    Cv(start=227.55151))
    annotation (Placement(transformation(extent={{210,-124},{230,-104}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser(
    water_height=1.0,
    S=100.0,
    Q_cold_0=54000.0,
    Q_hot_0=1000.0,
    Psat_0=6980,
    P_cold_in_0=300000,
    P_cold_out_0=300000,
    T_cold_in_0=288.15,
    T_cold_out_0=298.15,
    T_hot_in_0=312.0976,
    T_hot_out_0=312.0976,
    h_cold_in_0=63000.0,
    h_cold_out_0=105000.0,
    h_hot_in_0=2400000.0,
    h_liq_sat_0=163142.9,
    Tsat_0=312.0976)
    annotation (Placement(transformation(extent={{572,40},{672,140}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source4
    annotation (Placement(transformation(extent={{814,78},{774,118}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink4(Q_in(start
        =54232.63), Qv_in(start=54.388317))
    annotation (Placement(transformation(extent={{780,22},{820,62}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor CW_P_in_sensor(
      sensor_function="BC", P_start=3)
    annotation (Placement(transformation(extent={{754,90},{734,110}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor CW_T_in_sensor(
      sensor_function="BC", T_start=15)
    annotation (Placement(transformation(extent={{724,90},{704,110}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition CW_T_in
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={714,120}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition CW_P_in
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={744,120}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.TemperatureSensor CW_T_out_sensor(
    sensor_function="Calibration",
    causality="condenser_Qv_cold",
    T_start=25)
    annotation (Placement(transformation(extent={{704,80},{724,60}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput CW_T_out
    annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=270,
        origin={714,40}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealExpression condenser_Kfr_cold(y=0)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={524,62})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealExpression condenser_C_incond(y=0)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={516,136})));
  MetroscopeModelingLibrary.WaterSteam.Machines.FixedSpeedPump extraction_pump(
    T_in_0=312.05,
    T_out_0=312.15,
    P_in_0=6980,
    P_out_0=700000,
    DP_0=693020,
    h_in_0=163000.0,
    h_out_0=164000.0,
    Q_0=1060.0,
    W(start=0))
    annotation (Placement(transformation(extent={{422,-20},{382,20}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.LoopBreaker loopBreaker
    annotation (Placement(transformation(extent={{-710,-10},{-730,10}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Superheater superheater(
    Q_cold_0=1059.0,
    Q_hot_0=44.0,
    Q_vent_0=1.0,
    P_cold_in_0=1940000,
    P_cold_out_0=1940000,
    P_hot_in_0=4100000,
    P_hot_out_0=4000000,
    T_cold_in_0=483.95,
    T_cold_out_0=501.15,
    T_hot_in_0=525.15,
    T_hot_out_0=525.15,
    h_cold_in_0=2780000.0,
    h_cold_out_0=2850000.0,
    h_hot_in_0=2800000.0,
    h_hot_out_0=1090000.0)
    annotation (Placement(transformation(extent={{-128,136},{-32,184}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.PressureCut pressureCut3(
    T_in_0=300,
    T_out_0=300,
    P_in_0=100000,
    P_out_0=100000,
    DP_0=0,
    h_in_0=500000.0,
    h_out_0=500000.0,
    Q_0=8000.0,
    W(start=0),
    T_0=300,
    h_0=500000.0,
    DP_input(start=-899999.94))
    annotation (Placement(transformation(extent={{-10,150},{10,170}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.PressureCut pressureCut4(
    T_in_0=300,
    T_out_0=300,
    P_in_0=100000,
    P_out_0=100000,
    DP_0=0,
    h_in_0=500000.0,
    h_out_0=500000.0,
    Q_0=8000.0,
    W(start=0),
    T_0=300,
    h_0=500000.0,
    DP_input(start=0))
    annotation (Placement(transformation(extent={{-10,126},{10,146}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve controlValve(
    T_in_0=536.15,
    T_out_0=536.15,
    P_in_0=5000000,
    P_out_0=4850000,
    DP_0=-150000,
    h_in_0=2800000.0,
    h_out_0=2800000.0,
    rho_0=998,
    Q_0=1455.0,
    W(start=0),
    T_0=536.15,
    h_0=2800000.0,
    Cv(start=196329.05))
    annotation (Placement(transformation(extent={{-710,276},{-690,296}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.SlideValve slideValve(
    T_in_0=537.15,
    T_out_0=537.15,
    P_in_0=5000000,
    P_out_0=4100000,
    DP_0=-900000,
    h_in_0=2800000.0,
    h_out_0=2800000.0,
    rho_0=998,
    Q_0=45.0,
    W(start=0),
    T_0=537.15,
    h_0=2800000.0,
    Cv(start=2646.496))
    annotation (Placement(transformation(extent={{-710,156},{-690,178}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor HPT_P_in_sensor(
    sensor_function="Calibration",
    causality="HPT1_Cst",
    P_start=48.5)
    annotation (Placement(transformation(extent={{-650,270},{-630,290}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HPT_P_in
    annotation (Placement(transformation(
        extent={{-4,4},{4,-4}},
        rotation=270,
        origin={-640,300}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Sensors_Control.WaterSteam.PressureSensor superheater_bleed_P_sensor(
    sensor_function="Calibration",
    causality="superheater_control_valve_Cv_max",
    P_start=41)
    annotation (Placement(transformation(extent={{-652,150},{-632,170}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput superheater_bleed_P
    annotation (Placement(transformation(
        extent={{-4,4},{4,-4}},
        rotation=270,
        origin={-642,180}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Sensors_Control.Outline.OpeningSensor HP_control_valve_opening_sensor(
    sensor_function="Calibration",
    causality="HP_control_valve_Cvmax",
    Opening_start=14.999998,
    signal_unit="%")
    annotation (Placement(transformation(extent={{-710,310},{-690,330}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_control_valve_opening(start=15)
    annotation (Placement(transformation(
        extent={{-4,4},{4,-4}},
        rotation=270,
        origin={-700,342}), iconTransformation(extent={{-978,-52},{-938,-12}})));
  MetroscopeModelingLibrary.Sensors_Control.Outline.OpeningSensor LP_reheater_drains_control_valve_opening_sensor(
    sensor_function="Calibration",
    causality="LP_heater_drains_control_valve_Cvmax",
    Opening_start=14.999999,
    signal_unit="%")
    annotation (Placement(transformation(extent={{210,-90},{230,-70}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput LP_reheater_drains_control_valve_opening(start=15)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={220,-60}), iconTransformation(extent={{82,-132},{122,-92}})));
  MetroscopeModelingLibrary.Sensors_Control.Outline.OpeningSensor HP_reheater_drains_control_valve_opening_sensor(
    sensor_function="Calibration",
    causality="HP_heater_drains_control_valve_Cvmax",
    Opening_start=15,
    signal_unit="%")
    annotation (Placement(transformation(extent={{-310,-86},{-290,-66}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_reheater_drains_control_valve_opening(start=15)
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-300,-58}), iconTransformation(extent={{82,-132},{122,-92}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_control_valve_Cvmax
    annotation (Placement(transformation(extent={{-728,288},{-720,296}}),
        iconTransformation(extent={{-788,294},{-768,314}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HPT1_Cst
    annotation (Placement(transformation(extent={{-604,216},{-596,224}}),
        iconTransformation(extent={{-788,294},{-768,314}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput superheater_control_valve_Cv_max
    annotation (Placement(transformation(extent={{-726,168},{-718,176}}),
        iconTransformation(extent={{-788,294},{-768,314}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput turbines_eta_is
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-160,220}), iconTransformation(extent={{-534,258},{-514,278}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HPT2_Cst
    annotation (Placement(transformation(extent={{-404,216},{-396,224}}),
        iconTransformation(extent={{-788,294},{-768,314}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput LPT1_Cst
    annotation (Placement(transformation(extent={{-24,216},{-16,224}}),
        iconTransformation(extent={{-788,294},{-768,314}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput LPT2_Cst
    annotation (Placement(transformation(extent={{176,216},{184,224}}),
        iconTransformation(extent={{-788,294},{-768,314}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput condenser_Kth
    annotation (Placement(transformation(extent={{496,96},{504,104}}),
        iconTransformation(extent={{226,112},{246,132}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput condenser_Qv_cold
    annotation (Placement(transformation(extent={{496,86},{504,94}}),
        iconTransformation(extent={{226,112},{246,132}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput extraction_pump_rh
    annotation (Placement(transformation(extent={{356,-44},{364,-36}}),
        iconTransformation(extent={{122,-30},{142,-10}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput extraction_pump_hn
    annotation (Placement(transformation(extent={{356,-64},{364,-56}}),
        iconTransformation(extent={{122,-30},{142,-10}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput LP_heater_Kth
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={150,60}), iconTransformation(extent={{0,24},{20,44}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput LP_heater_Kfr_cold
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={180,60}), iconTransformation(extent={{0,24},{20,44}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_heater_Kth_subc
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-430,60}), iconTransformation(extent={{-688,10},{-668,30}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_heater_Kfr_cold
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-400,60}), iconTransformation(extent={{-688,10},{-668,30}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_heater_Kth_cond
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-430,-60}), iconTransformation(extent={{-688,10},{-668,30}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput feedwater_pump_rh
    annotation (Placement(transformation(extent={{-314,-44},{-306,-36}}),
        iconTransformation(extent={{-524,-14},{-504,6}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput feedwater_pump_hn
    annotation (Placement(transformation(extent={{-246,-44},{-254,-36}}),
        iconTransformation(extent={{-524,-14},{-504,6}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput HP_heater_drains_control_valve_Cvmax
    annotation (Placement(transformation(extent={{-326,-112},{-318,-104}}),
        iconTransformation(extent={{-524,-14},{-504,6}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput LP_heater_drains_control_valve_Cvmax
    annotation (Placement(transformation(extent={{196,-112},{204,-104}}),
        iconTransformation(extent={{-524,-14},{-504,6}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.RealOutput superheater_Kth
    annotation (Placement(transformation(extent={{-138,188},{-130,196}}),
        iconTransformation(extent={{-294,202},{-274,222}})));
equation
//   sink3.Q_in = 70;
//   source2.h_out = 1745904.9;
//   source2.P_out = 31e5;
//   sink1.Q_in = 56;
//   source3.h_out = 2711154;
  //   source3.P_out = 5e5;

  //HPT1_Cst = 19794.209;
  //superheater_control_valve_Cv_max = 2370.9817;
  //HP_control_valve_Cvmax= 1019899.5;

//The values simulated from the reverse after fixing the start values
  HPT1_Cst = 12398.202;
  HP_control_valve_Cvmax = 1288686;
  HP_heater_Kth_subc = 184783.44;
  HP_heater_Kfr_cold = 43.287144;
  HP_heater_Kth_cond =146484.64;
  feedwater_pump_rh =0.31297985;
  feedwater_pump_hn =554.81726;
  HP_heater_drains_control_valve_Cvmax = 2199.8484;
  LP_heater_Kth = 11611.088;
  LP_heater_Kfr_cold = 88.25726;
  extraction_pump_hn = 70.18661;
  extraction_pump_rh = 0.89422244;
  LP_heater_drains_control_valve_Cvmax = 758.7734;
  condenser_Kth = 1230111.5;
  turbines_eta_is = 0.5266621;
  condenser_Qv_cold = 54.493572;
  superheater_control_valve_Cv_max = 2414.415;
  LPT1_Cst = 6232.2524;
  LPT2_Cst = 591.8084;
  HPT2_Cst = 9641.745;
  superheater_Kth= 16408.51;

  connect(LP_reheater_drains_control_valve.C_out, condenser.C_hot_in) annotation (Line(
      points={{230,-120},{460,-120},{460,280},{622,280},{622,140}},
      color={28,108,200},
      thickness=0.5));
  connect(slideValve.C_in, controlValve.C_in) annotation (Line(
      points={{-710,160},{-760,160},{-760,280},{-710,280}},
      color={238,46,47},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(thermal_power_sensor.C_in, source.C_out) annotation (Line(points={{-852,0},
          {-870.4,0}},                                                                            color={244,125,35}));
  connect(thermal_power, thermal_power_sensor.W_sensor) annotation (Line(points={{-840,20},
          {-840,10},{-842,10}},                                                                        color={28,108,200}));
  connect(HP_heater_T_out_sensor.C_in, HP_heater_P_out_sensor.C_out) annotation (Line(points={{-666,0},{-656,0}}, color={28,108,200},
      thickness=1));
  connect(HP_heater_P_out_sensor.C_in, Q_feedwater_sensor.C_out) annotation (Line(points={{-636,0},{-626,0}}, color={28,108,200},
      thickness=1));
  connect(Q_feedwater, Q_feedwater_sensor.Q_sensor) annotation (Line(points={{-616,20},{-616,10}}, color={0,0,127}));
  connect(HP_heater_P_out_sensor.P_sensor, HP_heater_P_out) annotation (Line(points={{-646,10},{-646,20}}, color={0,0,127}));
  connect(HP_heater_T_out_sensor.T_sensor, HP_heater_T_out) annotation (Line(points={{-676,10},{-676,20}}, color={0,0,127}));
  connect(Q_purge_sensor.C_out, sink_purge.C_in) annotation (Line(points={{-780,-124},{-780,-134}}, color={28,108,200}));
  connect(Q_purge_sensor.Q_sensor, Q_purge) annotation (Line(points={{-790,-114},{-810,-114}}, color={0,0,127}));
  connect(P_steam_sensor.P_sensor, P_steam) annotation (Line(points={{-790,118},{-802,118}}, color={0,0,127}));
  connect(Q_purge_sensor.C_in, steamGenerator.purge_outlet) annotation (Line(points={{-780,-104},{-780,-82.6}}, color={28,108,200}));
  connect(P_steam_sensor.C_in, steamGenerator.steam_outlet) annotation (Line(points={{-780,108},{-780,84}}, color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(steamGenerator.vapor_fraction, steam_generator_vapor_fraction) annotation (Line(points={{-812.2,56},{-836,56}}, color={0,0,127}));
  connect(thermal_power_sensor.C_out, steamGenerator.C_thermal_power) annotation (Line(points={{-832.2,
          0},{-801,0}},                                                                                              color={244,125,35}));
  connect(HPT_1.C_out, HP_extract.C_in) annotation (Line(points={{-520,280},{-481.2,280}}, color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(HP_extract.C_main_out, HPT_2.C_in) annotation (Line(points={{-438.8,280},
          {-398,280}},                                                                          color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(sink2.C_in, generator.C_out) annotation (Line(points={{474,400},{450,400}},color={244,125,35},
      smooth=Smooth.Bezier));
  connect(generator.C_in, W_elec_sensor.C_out) annotation (Line(points={{397.2,400},{377.8,400}},
                                                                                               color={244,125,35},
      smooth=Smooth.Bezier));
  connect(HP_extract.C_ext_out, HP_extract_P_sensor.C_in) annotation (Line(points={{-460,266.4},{-460,120}}, color={238,46,47},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(HPT_1.eta_is, HPT_2.eta_is) annotation (Line(points={{-568,248},{-568,
          200},{-366,200},{-366,248}},                                                                       color={0,0,127}));
  connect(HP_extract_P_sensor.P_sensor, HP_extract_P) annotation (Line(points={{-450,110},{-420,110}}, color={0,0,127}));
  connect(W_elec, W_elec_sensor.W_sensor) annotation (Line(points={{368,428},{368,410}},
                                                                                     color={0,0,127}));
  connect(HPT_2.C_out, HPT_extract_P1_sensor.C_in) annotation (Line(points={{-318,
          280},{-270,280}},                                                                         color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(HPT_extract_P1_sensor.P_sensor, HPT_extract_P1) annotation (Line(points={{-260,290},{-260,310}}, color={0,0,127}));
  connect(HP_extract.alpha, realExpression.y) annotation (Line(points={{-444,284.8},{-444,296}}, color={0,0,127}));
  connect(Q_feedwater_sensor.C_in, reheater.C_cold_out) annotation (Line(points={{-606,0},{-508,0}}, color={28,108,200},
      thickness=1));
  connect(HP_pump_P_out_sensor.P_sensor, HP_pump_P_out) annotation (Line(points={{-340,10},{-340,20}}, color={0,0,127}));
  connect(HP_pump_T_out_sensor.T_sensor, HP_pump_T_out) annotation (Line(points={{-370,10},{-370,20}}, color={0,0,127}));
  connect(reheater.C_cold_in, HP_pump_T_out_sensor.C_out) annotation (Line(points={{-411.4,0},{-380,0}}, color={28,108,200},
      thickness=1));
  connect(HP_pump_T_out_sensor.C_in, HP_pump_P_out_sensor.C_out) annotation (Line(points={{-360,0},{-350,0}}, color={28,108,200},
      thickness=1));
  connect(reheater.C_hot_out, HP_heater_T_drains_sensor.C_in) annotation (Line(points={{-460,-24},{-460,-50}}, color={238,46,47},
      thickness=0.5));
  connect(HP_heater_T_drains_sensor.T_sensor, HP_heater_T_drains) annotation (Line(points={{-470,-60},{-486,-60}}, color={0,0,127}));
  connect(HP_pump_P_out_sensor.C_in,feedwater_pump. C_out) annotation (Line(points={{-330,0},{-300,0}}, color={28,108,200},
      thickness=1));
  connect(LP_heater_P_out_sensor.P_sensor, LP_heater_P_out) annotation (Line(points={{46,10},{46,20}}, color={0,0,127}));
  connect(LP_heater_T_out_sensor.T_sensor, LP_heater_T_out) annotation (Line(points={{16,10},{16,20}}, color={0,0,127}));
  connect(LP_heater_T_out_sensor.C_in, LP_heater_P_out_sensor.C_out) annotation (Line(points={{26,0},{36,0}}, color={28,108,200},
      thickness=1));
  connect(HPT_extract_P1_sensor.C_out, steamDryer.C_in) annotation (Line(points={{-250,
          280},{-240,280},{-240,119.818},{-223,119.818}},                                                                              color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(steamDryer.C_hot_liquid, pressureCut.C_in) annotation (Line(points={{-173,
          100.182},{-160,100.182},{-160,38}},                                                                                color={244,125,35},
      thickness=0.5));
  connect(feedwater_pump.C_in, deaerator_outlet_pipe.C_out) annotation (Line(points={{-260,0},{-210,0}}, color={28,108,200},
      thickness=1));
  connect(pressureCut.C_out, deaerator_inlet_pipe.C_out) annotation (Line(points={{-160,18},{-160,0},{-130,0}}, color={28,108,200},
      thickness=0.5));
  connect(LP_heater_T_out_sensor.C_out, deaerator_inlet_pipe.C_in) annotation (Line(points={{6,0},{-110,0}},color={28,108,200},
      thickness=1));
  connect(deaerator_outlet_pipe.delta_z, realExpression1.y) annotation (Line(points={{-200,4.8},{-200,29}}, color={0,0,127}));
  connect(deaerator_inlet_pipe.delta_z, realExpression2.y) annotation (Line(points={{-120,4.8},{-120,29}}, color={0,0,127}));
  connect(controlValve1.C_out, deaerator_inlet_pipe.C_out) annotation (Line(
      points={{-290,-120},{-160,-120},{-160,0},{-130,0}},
      color={28,108,200},
      thickness=0.5));
  connect(HP_heater_T_drains_sensor.C_out, controlValve1.C_in) annotation (Line(
      points={{-460,-70},{-460,-120},{-310,-120}},
      color={238,46,47},
      thickness=0.5));
  connect(superheater_T_out_sensor.T_sensor, superheater_T_out) annotation (Line(points={{-90,230},{-100,230}},  color={0,0,127}));
  connect(LP_extract.alpha, realExpression3.y) annotation (Line(points={{136,284.8},{136,298}},                     color={0,0,127}));
  connect(LPT_1.eta_is,LPT_2. eta_is) annotation (Line(points={{10,248},{10,200},
          {212,200},{212,248}},                                                                              color={0,0,127}));
  connect(LPT_1.C_in, superheater_T_out_sensor.C_out) annotation (Line(points={{-22,280},
          {-80,280},{-80,240}},                                                                                                      color={244,125,35},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(LPT_1.C_out, LP_extract.C_in) annotation (Line(points={{58,280},{98.8,
          280}},                                                                       color={244,125,35},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(LP_extract.C_main_out, LPT_2.C_in) annotation (Line(points={{141.2,280},{180,280}}, color={244,125,35},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(LPT_2.C_out, P_cond_sensor.C_in) annotation (Line(points={{260,280},{350,280}}, color={28,108,200},
      thickness=1));
  connect(LP_extract.C_ext_out, LP_extract_P_sensor.C_in) annotation (Line(points={{120,266.4},{120,180}}, color={244,125,35},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(LP_extract_P_sensor.P_sensor, LP_extract_P) annotation (Line(points={{130,170},{148,170}}, color={0,0,127}));
  connect(P_cond_sensor.P_sensor, P_cond) annotation (Line(points={{360,290},{360,312}},           color={0,0,127}));
  connect(HPT_2.eta_is, LPT_2.eta_is) annotation (Line(points={{-366,248},{-366,
          200},{212,200},{212,248}},                                                                       color={0,0,127}));
  connect(LP_heater_P_out_sensor.C_in, dryReheater.C_cold_out) annotation (Line(points={{56,0},{72,0}}, color={28,108,200},
      thickness=1));
  connect(extraction_pump_P_out_sensor.T_sensor, extraction_pump_P_out) annotation (Line(points={{208,10},{208,20}}, color={0,0,127}));
  connect(extraction_pump_T_out_sensor.P_sensor, extraction_pump_T_out) annotation (Line(points={{240,10},
          {240,16},{238,16},{238,20}},                                                                               color={0,0,127}));
  connect(dryReheater.C_cold_in, extraction_pump_P_out_sensor.C_out) annotation (Line(points={{168.6,0},{198,0}}, color={28,108,200},
      thickness=1));
  connect(extraction_pump_P_out_sensor.C_in, extraction_pump_T_out_sensor.C_out) annotation (Line(points={{218,0},
          {230,0}},                                                                                                         color={28,108,200},
      thickness=1));
  connect(P_cond_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{370,280},
          {622,280},{622,140}},                                                                            color={28,108,200},
      thickness=1));
  connect(CW_T_in_sensor.T_sensor, CW_T_in) annotation (Line(points={{714,110},{714,120}},
                                                                                         color={0,0,127}));
  connect(CW_P_in_sensor.P_sensor, CW_P_in) annotation (Line(points={{744,110},{744,120}},
                                                                                         color={0,0,127}));
  connect(CW_P_in_sensor.C_in, source4.C_out) annotation (Line(points={{754,100},
          {770,100},{770,98},{784,98}},                                                    color={28,108,200}));
  connect(condenser.C_cold_in, CW_T_in_sensor.C_out) annotation (Line(points={{672,100},
          {704,100}},                                                                             color={28,108,200}));
  connect(CW_P_in_sensor.C_out, CW_T_in_sensor.C_in) annotation (Line(points={{734,100},{724,100}},
                                                                                                  color={28,108,200}));
  connect(CW_T_out_sensor.T_sensor, CW_T_out) annotation (Line(points={{714,60},{714,40}}, color={0,0,127}));
  connect(condenser.Kfr_cold, condenser_Kfr_cold.y) annotation (Line(points={{570,80},
          {544,80},{544,62},{529,62}},                                                                             color={0,0,127}));
  connect(condenser.C_incond, condenser_C_incond.y) annotation (Line(points={{570,110},
          {536,110},{536,136},{521,136}},                                                                          color={0,0,127}));
  connect(condenser.C_cold_out, CW_T_out_sensor.C_in) annotation (Line(points={{672,70},
          {704,70}},                                                                               color={28,108,200}));
  connect(CW_T_out_sensor.C_out, sink4.C_in) annotation (Line(points={{724,70},{790,70},{790,42}},                  color={28,108,200}));
  connect(LP_extract_P_sensor.C_out, dryReheater.C_hot_in) annotation (Line(points={{120,160},{120,24}},                    color={244,125,35},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(HP_extract_P_sensor.C_out, reheater.C_hot_in) annotation (Line(points={{-460,100},{-460,24},{-460,24}}, color={238,46,47},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(extraction_pump_T_out_sensor.C_in, extraction_pump.C_out) annotation (Line(
      points={{250,0},{382,0}},
      color={28,108,200},
      thickness=1));
  connect(condenser.C_hot_out, extraction_pump.C_in) annotation (Line(
      points={{622,40},{622,0},{422,0}},
      color={28,108,200},
      thickness=1));
  connect(HP_heater_T_out_sensor.C_out, loopBreaker.C_in) annotation (Line(
      points={{-686,0},{-710,0}},
      color={28,108,200},
      thickness=1));
  connect(loopBreaker.C_out, steamGenerator.feedwater_inlet) annotation (Line(
      points={{-730,0},{-759,0}},
      color={28,108,200},
      thickness=1));
  connect(steamDryer.C_hot_steam, superheater.C_cold_in) annotation (Line(
      points={{-173,119.818},{-80,119.818},{-80,136}},
      color={244,125,35},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(superheater.C_cold_out, superheater_T_out_sensor.C_in) annotation (Line(
      points={{-80,184},{-80,220}},
      color={244,125,35},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(superheater.C_vent, pressureCut4.C_in) annotation (Line(
      points={{-32,136},{-10,136}},
      color={238,46,47},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(superheater.C_hot_out, pressureCut3.C_in) annotation (Line(
      points={{-32,160},{-10,160}},
      color={238,46,47},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(pressureCut3.C_out, pressureCut4.C_out) annotation (Line(
      points={{10,160},{10,136}},
      color={238,46,47},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(pressureCut4.C_out, reheater.C_hot_in) annotation (Line(
      points={{10,136},{10,148},{20,148},{20,80},{-460,80},{-460,24}},
      color={238,46,47},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(P_steam_sensor.C_out, controlValve.C_in) annotation (Line(
      points={{-780,128},{-780,280},{-710,280}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(HPT_P_in_sensor.P_sensor, HPT_P_in) annotation (Line(points={{-640,290},{-640,300}}, color={0,0,127}));
  connect(controlValve.C_out, HPT_P_in_sensor.C_in) annotation (Line(
      points={{-690,280},{-650,280}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(HPT_P_in_sensor.C_out, HPT_1.C_in) annotation (Line(
      points={{-630,280},{-600,280}},
      color={238,46,47},
      thickness=1,
      pattern=LinePattern.Dash));
  connect(superheater_bleed_P_sensor.P_sensor, superheater_bleed_P) annotation (Line(points={{-642,170},{-642,180}}, color={0,0,127}));
  connect(superheater.C_hot_in, superheater_bleed_P_sensor.C_out) annotation (Line(
      points={{-128,160},{-632,160}},
      color={238,46,47},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(superheater_bleed_P_sensor.C_in, slideValve.C_out) annotation (Line(
      points={{-652,160},{-690,160}},
      color={238,46,47},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(controlValve.Opening, HP_control_valve_opening_sensor.Opening) annotation (Line(points={{-700,296},{-700,309.8}}, color={0,0,127}));
  connect(HP_control_valve_opening_sensor.opening_sensor, HP_control_valve_opening) annotation (Line(points={{-700,330.2},{-700,342}}, color={0,0,127}));
  connect(deaerator_outlet_pipe.C_in, deaerator_inlet_pipe.C_out) annotation (Line(points={{-190,0},{-130,0}}, color={28,108,200},
      thickness=1));
  connect(LP_reheater_drains_control_valve.Opening, LP_reheater_drains_control_valve_opening_sensor.Opening) annotation (Line(points={{220,-104},{220,-90.2}}, color={0,0,127}));
  connect(LP_reheater_drains_control_valve_opening_sensor.opening_sensor, LP_reheater_drains_control_valve_opening) annotation (Line(points={{220,-69.8},{220,-60}}, color={0,0,127}));
  connect(dryReheater.C_hot_out, LP_reheater_drains_control_valve.C_in) annotation (Line(
      points={{120,-24},{120,-120},{210,-120}},
      color={244,125,35},
      thickness=0.5));
  connect(HP_reheater_drains_control_valve_opening_sensor.opening_sensor, HP_reheater_drains_control_valve_opening) annotation (Line(points={{-300,
          -65.8},{-300,-58}},                                                                                                                                          color={0,0,127}));
  connect(HP_reheater_drains_control_valve_opening_sensor.Opening, controlValve1.Opening) annotation (Line(points={{-300,
          -86.2},{-300,-104}},                                                                                                                color={0,0,127}));
  connect(HPT_1.C_W_out, W_elec_sensor.C_in) annotation (Line(
      points={{-520,313.6},{-520,400},{358,400}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(HPT_2.C_W_out, W_elec_sensor.C_in) annotation (Line(
      points={{-318,313.6},{-318,400},{358,400}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(LPT_1.C_W_out, W_elec_sensor.C_in) annotation (Line(
      points={{58,313.6},{66,313.6},{66,334},{358,334},{358,400}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(LPT_2.C_W_out, W_elec_sensor.C_in) annotation (Line(
      points={{260,313.6},{268,313.6},{268,314},{358,314},{358,400}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(controlValve.Cv_max, HP_control_valve_Cvmax) annotation (Line(points={{-704,292},{-724,292}}, color={0,0,127}));
  connect(HPT_1.Cst, HPT1_Cst) annotation (Line(points={{-584,252},{-584,220},{-600,220}}, color={0,0,127}));
  connect(slideValve.Cv, superheater_control_valve_Cv_max) annotation (Line(
        points={{-704,172},{-704,172},{-722,172}}, color={0,0,127}));
  connect(superheater_control_valve_Cv_max, superheater_control_valve_Cv_max)
    annotation (Line(
      points={{-722,172},{-722,172}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(turbines_eta_is, LPT_2.eta_is) annotation (Line(points={{-160,220},{-160,200},{212,200},{212,248}}, color={0,0,127}));
  connect(HPT_2.Cst, HPT2_Cst) annotation (Line(points={{-382,252},{-382,220},{-400,
          220}},                                                                           color={0,0,127}));
  connect(LPT_1.Cst, LPT1_Cst) annotation (Line(points={{-6,252},{-6,220},{-20,220}}, color={0,0,127}));
  connect(LPT_2.Cst, LPT2_Cst) annotation (Line(points={{196,252},{196,220},{180,220}}, color={0,0,127}));
  connect(condenser.Kth, condenser_Kth) annotation (Line(points={{570,100},{500,
          100}},                                                                       color={0,0,127}));
  connect(condenser.Qv_cold_in, condenser_Qv_cold) annotation (Line(points={{570,90},
          {500,90}},                                                                            color={0,0,127}));
  connect(extraction_pump.rh, extraction_pump_rh) annotation (Line(points={{390,-16},{390,-40},{360,-40}}, color={0,0,127}));
  connect(extraction_pump.hn, extraction_pump_hn) annotation (Line(points={{414,-16},{414,-60},{360,-60}}, color={0,0,127}));
  connect(dryReheater.Kth, LP_heater_Kth) annotation (Line(points={{150,25.2},{150,60}}, color={0,0,127}));
  connect(dryReheater.Kfr_cold, LP_heater_Kfr_cold) annotation (Line(points={{169.2,12},{180,12},{180,60}}, color={0,0,127}));
  connect(reheater.Kth_subc, HP_heater_Kth_subc) annotation (Line(points={{-430,25.2},{-430,60}}, color={0,0,127}));
  connect(reheater.Kfr_cold, HP_heater_Kfr_cold) annotation (Line(points={{-410.8,12},{-400,12},{-400,60}}, color={0,0,127}));
  connect(reheater.Kth_cond, HP_heater_Kth_cond) annotation (Line(points={{-430,-25.2},{-430,-60}}, color={0,0,127}));
  connect(HP_heater_Kth_cond, HP_heater_Kth_cond) annotation (Line(points={{-430,-60},{-430,-60}}, color={0,0,127}));
  connect(feedwater_pump.rh, feedwater_pump_rh) annotation (Line(points={{-292,-16},{-292,-40},{-310,-40}}, color={0,0,127}));
  connect(feedwater_pump.hn, feedwater_pump_hn) annotation (Line(points={{-268,-16},{-268,-40},{-250,-40}}, color={0,0,127}));
  connect(controlValve1.Cv_max, HP_heater_drains_control_valve_Cvmax) annotation (Line(points={{-304,-108},{-322,-108}}, color={0,0,127}));
  connect(LP_reheater_drains_control_valve.Cv_max, LP_heater_drains_control_valve_Cvmax) annotation (Line(points={{216,-108},{200,-108}}, color={0,0,127}));
  connect(superheater_Kth, superheater.Kth) annotation (Line(points={{-134,
          192},{-128,192},{-128,185.2},{-116,185.2}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-900,-180},{820,460}})),
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-900,-180},{820,460}})),
    experiment(__Dymola_fixedstepsize=10, __Dymola_Algorithm="Euler"));
end MetroscopiaNPP_direct_MML4;
