within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model MetroscopiaNPP_reverse


  // Boundary Conditions

    input Real P_steam(start = 50, unit="bar", min=0, nominal=50) "barA"; // Steam generator steam pressure
    input Real CW_P_in(start = 3, unit="bar", min=0, nominal=5) "barA"; // Circulating Water inlet pressure
    input Real CW_T_in(start = 15, unit="degC", min=0, nominal=15) "degC"; // Circulating Water inlet temperature
    input Real Q_purge(start=5, unit = "kg/s", min=0) "kg/s"; // Steam generator blowdown flow
    input Real thermal_power(start=2820, unit="MW", nominal = 1e3) "MW"; // Core thermal power


  // Observables used for calibration

    // HP Control Valve
    input Real HP_control_valve_opening(start=0.15);
    // HP turbines
    input Real HPT_P_in(start=48.5, unit="bar", min=0, nominal=50) "barA";
    input Real HP_extract_P(start=31, unit="bar", min=0, nominal=50) "barA";
    input Real HPT_P_out(start=19.4, unit="bar", min=0, nominal=50) "barA";
    // Superheater
    input Real superheater_bleed_P(start=41, unit="bar", min=0, nominal=50) "barA";
    input Real superheater_drains_P(start=40, unit="bar", min=0, nominal=50) "barA";
    input Real superheater_T_out(start=228) "°C"; // superheater_Kth
    // LP turbines
    input Real LP_extract_P(start=5, unit="bar", min=0, nominal=5) "barA";
    // Condenser
    input Real P_cond(start=69.8, unit="mbar", min=0, nominal=70) "mbar";
    input Real CW_T_out(start=25, unit= "degC", min=0, nominal = 25) "degC";
    // Generator
    input Real W_elec(start=570) "MW"; //
    // Extraction pump
    input Real extraction_pump_P_out(start=7, unit="bar", min=0, nominal=70) "barA";
    input Real extraction_pump_T_out(start=39, unit="degC", min=0, nominal=20) "degC";
    // LP Heater
    input Real LP_heater_P_out(start=6, min=0, nominal=50) "bar"; // LP_heater_Kfr_cold
    input Real LP_heater_T_out(start=65, min=0, nominal=100) "degC"; // LP_heater_Kth
    // LP reheater drains Control Valve
    input Real LP_reheater_drains_control_valve_opening(start=0.15);
    // Feedwater pump
    input Real HP_pump_P_out(start=59, unit="bar", min=0, nominal=70) "barA";
    input Real HP_pump_T_out(start=80, unit="degC", min=0, nominal=20) "degC";
    // HP Reheater
    input Real HP_heater_P_out(start=58, min=0, nominal=50) "bar";
    input Real HP_heater_T_out(start=210, min=0, nominal=100) "degC";
    input Real HP_heater_T_drains(start=90, min=0, nominal=100) "degC";
    // HP reheater drains Control Valve
    input Real HP_reheater_drains_control_valve_opening(start=0.15);

  // Other observables
    output Real Q_feedwater(start=1500, unit="kg/s", min=0, nominal=1e3) "kg/s"; // Feedwater flow rate

  // Calibrated parameters (input used for calibration in comment)
    // HP turbines inlet control valve
    output Utilities.Units.Cv HP_control_valve_Cvmax; // HP valve opening
    // HP Turbines
    output Utilities.Units.Cst HPT1_Cst; // HP turbine inlet pressure
    output Utilities.Units.Cst HPT2_Cst;  // HP extract pressure
    output Utilities.Units.Yield turbines_eta_is; // Welec
    // Superheater inlet control valve
    output Utilities.Units.Cv superheater_control_valve_Cvmax; // Superheater bleed pressure
    // Superheater
    output Utilities.Units.FrictionCoefficient superheater_Kfr_hot; // Superheater drains pressure
    output Utilities.Units.HeatExchangeCoefficient superheater_Kth; // Superheater steam outlet temperature
    // LP Turbines
    output Utilities.Units.Cst LPT1_Cst; // HP turbine outlet pressure
    output Utilities.Units.Cst LPT2_Cst; // LP extract pressure
    // Condenser
    output Utilities.Units.HeatExchangeCoefficient condenser_Kth; // P cond
    output Utilities.Units.PositiveMassFlowRate condenser_Q_cold; // Circulating water outlet temperature
    // Extraction pump
    output Real extraction_pump_hn; // Extraction pump outlet pressure
    output Real extraction_pump_rh; // Extraction pump outlet temperature
    // LP Heater
    output Utilities.Units.HeatExchangeCoefficient LP_heater_Kth; // LP heater outlet temperature
    output Utilities.Units.FrictionCoefficient LP_heater_Kfr_cold; // LP heater outlet pressure
    output Utilities.Units.Cv LP_heater_drains_control_valve_Cvmax; // LP heater drains valve opening
    // Feedwater pump
    output Real feedwater_pump_hn; // Feedwater pump outlet pressure
    output Real feedwater_pump_rh; // Feedwater pump outlet temperature
    // HP Heater
    output Utilities.Units.HeatExchangeCoefficient HP_heater_Kth_cond; // HP heater outlet temperature
    output Utilities.Units.HeatExchangeCoefficient HP_heater_Kth_subc; // HP heater drains temperature
    output Utilities.Units.FrictionCoefficient HP_heater_Kfr_cold; // HP heater outlet pressure
    output Utilities.Units.Cv HP_heater_drains_control_valve_Cvmax; // HP heater drains valve opening

    MetroscopeModelingLibrary.WaterSteam.HeatExchangers.SteamGenerator steam_generator annotation (Placement(transformation(extent={{-202,-126},{-158,-34}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_feedwater_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6)                                                               annotation (Placement(transformation(extent={{-104,-87},{-118,-73}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve HP_control_valve(
    T_out_0=535.15,
    P_in_0=5000000,
    P_out_0=4850000,
    h_in_0=2.8e6,
    h_out_0=2.8e6,
    Q_0=1455,
    T_0=536.15,
    h_0=2.8e6)                                                                                            annotation (Placement(transformation(extent={{-145,77.8182},{-135,89.8182}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor HP_control_valve_opening_sensor annotation (Placement(transformation(extent={{-145,95},{-135,105}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor HPT_P_in_sensor(
    Q_0=1455,
    P_0=4850000,
    h_0=2.8e6)                                                                annotation (Placement(transformation(extent={{-116,74},{-104,86}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine HPT_1(
    T_in_0=535.15,
    T_out_0=608.85,
    P_in_0=4850000,
    P_out_0=3100000,
    h_in_0=2.8e6,
    h_out_0=2.7e6,
    Q_0=1455) annotation (Placement(transformation(extent={{-89,72},{-71,88}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine HPT_2(
    T_in_0=508.85,
    T_out_0=484.15,
    P_in_0=3100000,
    P_out_0=1940000,
    h_in_0=2.73e6,
    h_out_0=2.68e6,
    Q_0=1113) annotation (Placement(transformation(extent={{-9,72},{9,88}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter HP_extract(
    Q_in_0=1455,
    Q_ext_0=340,
    P_0=3100000,
    T_0=508.85,
    h_0=2.73e6)                                                                   annotation (Placement(transformation(extent={{-50,70},{-30,88}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor HP_extract_P_sensor(
    Q_0=340,
    P_0=3100000,
    h_0=2.73e6)                                                                   annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-40,60})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor HPT_P_out_sensor(
    Q_0=1113,
    P_0=1940000,
    h_0=2.68e6)                                                                annotation (Placement(transformation(extent={{26,74},{38,86}})));
    MetroscopeModelingLibrary.WaterSteam.Volumes.SteamDryer steam_dryer(
    P_0=1940000,
    T_0=483.95,
    h_in_0=2.68e6,
    Q_in_0=1113,
    Q_liq_0=50)                                                         annotation (Placement(transformation(extent={{56,87.8182},{72,105.818}})));
    MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Superheater superheater(
    Q_cold_0=1059,
    Q_hot_0=44,
    P_cold_in_0=1940000,
    P_cold_out_0=1940000,
    P_hot_in_0=4100000,
    P_hot_out_0=4000000,
    T_cold_in_0=483.95,
    T_cold_out_0=501.15,
    T_hot_in_0=525.15,
    T_hot_out_0=525.15,
    h_cold_in_0=2.78e6,
    h_cold_out_0=2.85e6,
    h_hot_in_0=2.8e6,
    h_hot_out_0=1.09e6)                                                         annotation (Placement(transformation(extent={{56,112},{88,128}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor superheater_drains_P_sensor(
    Q_0=44,
    P_0=4000000,
    h_0=1.09e6)                                                                           annotation (Placement(transformation(extent={{100,114},{112,126}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor superheater_T_out_sensor(
    Q_0=1060,
    P_0=1940000,
    h_0=2.85e6,
    T_0=501.15)                                                                           annotation (Placement(transformation(extent={{88,134},{100,146}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve superheater_control_valve(
    P_in_0=5000000,
    P_out_0=4100000,
    h_in_0=2.778e6,
    h_out_0=2.778e6,
    Q_0=45,
    T_0=537.15,
    h_0=2.8e6)                                                                        annotation (Placement(transformation(extent={{-145,117.818},{-135,129.818}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor superheater_bleed_P_sensor(
    Q_0=45,
    P_0=4100000,
    h_0=2.778e6)                                                                         annotation (Placement(transformation(extent={{-116,114},{-104,126}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.PressureCut superheater_drains_pipe(
    P_in_0=4000000,
    P_out_0=3100000,
    Q_0=44,
    T_0=525.15,
    h_0=1.09e6)                                                                    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={120,38})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine LPT1(
    T_in_0=501.15,
    T_out_0=425.15,
    P_in_0=1940000,
    P_out_0=500000,
    h_in_0=2.85e6,
    h_out_0=2.7e6,
    Q_0=1060) annotation (Placement(transformation(extent={{151,132},{169,148}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine LPT2(
    T_in_0=425.15,
    T_out_0=312.05,
    P_in_0=500000,
    P_out_0=6900,
    h_in_0=2.7e6,
    h_out_0=2.4e6,
    Q_0=1000) annotation (Placement(transformation(extent={{231,132},{249,148}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor LP_extract_P_sensor(
    Q_0=55,
    P_0=500000,
    h_0=2.7e6)                                                                    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={200,110})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter LP_extract(
    Q_in_0=1060,
    Q_ext_0=55,
    P_0=500000,
    T_0=425.15,
    h_0=2.7e6)                                                                    annotation (Placement(transformation(extent={{190,130},{210,148}})));
    MetroscopeModelingLibrary.Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{364,170},{384,190}})));
    MetroscopeModelingLibrary.Power.Machines.Generator generator annotation (Placement(transformation(extent={{310,168},{350,192}})));
    MetroscopeModelingLibrary.Sensors.Power.PowerSensor W_elec_sensor annotation (Placement(transformation(extent={{350,174},{362,186}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cond_sensor(
    Q_0=1000,
    P_0=6900,
    h_0=2.4e6)                                                              annotation (Placement(transformation(extent={{286,134},{298,146}})));
    MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser(
    Q_cold_0=54000,
    Q_hot_0=1000,
    Psat_0=6980,
    P_cold_in_0=300000,
    P_cold_out_0=300000,
    T_cold_in_0=288.15,
    T_cold_out_0=298.15,
    h_cold_in_0=63e3,
    h_cold_out_0=105e3,
    h_hot_in_0=2.4e6)                                                                      annotation (Placement(transformation(extent={{385,48.2222},{416,74}})));
    MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={468,59.679})));
    MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{298,49.679},{318,69.679}})));
    MetroscopeModelingLibrary.WaterSteam.Machines.Pump extraction_pump(
    T_in_0=312.05,
    T_out_0=312.15,
    P_in_0=6980,
    P_out_0=700000,
    h_in_0=163e3,
    h_out_0=164e3,
    Q_0=1060)                                                                       annotation (Placement(transformation(extent={{368,-88},{352,-72}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor extraction_pump_T_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=164e3,
    T_0=312.15)                                                                               annotation (Placement(transformation(extent={{338,-87},{324,-73}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor extraction_pump_P_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=164e3)                                                                             annotation (Placement(transformation(extent={{7,-7},{-7,7}}, origin={301,-80})));
    MetroscopeModelingLibrary.WaterSteam.HeatExchangers.DryReheater LP_heater(
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
    h_hot_out_0=640e3)                                                        annotation (Placement(transformation(extent={{276,-88},{244,-72}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor LP_heater_T_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=272e3,
    T_0=338.15)                                                                         annotation (Placement(transformation(extent={{207,-87},{193,-73}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor LP_heater_P_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=272e3)                                                                       annotation (Placement(transformation(extent={{237,-87},{223,-73}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve LP_reheater_drains_control_valve(
    P_in_0=500000,
    P_out_0=6900,
    Q_0=55,
    T_0=425.15,
    h_0=640e3)                                                                               annotation (Placement(transformation(extent={{288,-122},{298,-110}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor LP_reheater_drains_control_valve_opening_sensor annotation (Placement(transformation(extent={{288,-106},{298,-96}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe deaerator_inlet_pipe(
    P_in_0=600000,
    P_out_0=550000,
    Q_0=1060,
    T_0=338.15,
    h_0=272e3)                                                           annotation (Placement(transformation(extent={{180,-90},{160,-70}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.PressureCut steam_dryer_liq_out_pipe(
    P_in_0=1940000,
    P_out_0=1940000,
    Q_0=50,
    T_0=483.95,
    h_0=901606.56)                                                                  annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={140,-60})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe deaerator_outlet_pipe(
    P_in_0=550000,
    P_out_0=600000,
    Q_0=1500,
    T_0=350.05,
    h_0=322e3)                                                            annotation (Placement(transformation(extent={{114,-90},{94,-70}})));
    MetroscopeModelingLibrary.WaterSteam.Machines.Pump feedwater_pump(
    T_in_0=350.05,
    T_out_0=353.15,
    P_in_0=600000,
    P_out_0=5900000,
    h_in_0=322e3,
    h_out_0=340e3,
    Q_0=1500)                                                                                         annotation (Placement(transformation(extent={{62,-88},{46,-72}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor HP_pump_T_out_sensor(
    Q_0=1500,
    P_0=5900000,
    h_0=340e3,
    T_0=353.15)                                                                       annotation (Placement(transformation(extent={{30,-87},{16,-73}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor HP_pump_P_out_sensor(
    Q_0=1500,
    P_0=5900000,
    h_0=340e3)                                                                     annotation (Placement(transformation(extent={{7,-7},{-7,7}}, origin={1,-80})));
    MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater HP_heater(Q_cold_0=1500,
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
    h_hot_out_0=379e3)                                                                                annotation (Placement(transformation(extent={{-24,-88},{-56,-72}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor HP_heater_T_out_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6)                                                                          annotation (Placement(transformation(extent={{-84,-87},{-98,-73}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor HP_heater_P_out_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6)                                                                       annotation (Placement(transformation(extent={{-64,-87},{-78,-73}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor HP_heater_T_drains_sensor(
    Q_0=387,
    P_0=3100000,
    h_0=379e3,
    T_0=363.15)                                                                            annotation (Placement(transformation(
        extent={{7,-7},{-7,7}},
        rotation=90,
        origin={-40,-100})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve HP_reheater_drains_control_valve annotation (Placement(transformation(extent={{6,-122},{16,-110}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor HP_reheater_drains_control_valve_opening_sensor annotation (Placement(transformation(extent={{6,-106},{16,-96}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_steam_sensor(
    Q_0=1500,
    P_0=5000000,
    h_0=2.778e6)                                                             annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=90,
        origin={-180,10})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.PressureCut pressureCut(
    P_in_0=4000000,
    P_out_0=4000000,
    Q_0=2,
    T_0=525.15,
    h_0=2.8e6)                                                       annotation (Placement(transformation(extent={{94,84},{114,104}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor CW_T_in_sensor(
    Q_0=54000,
    P_0=300000,
    h_0=63e3,
    T_0=288.15)                                                                 annotation (Placement(transformation(extent={{326,52.679},{340,66.679}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor CW_P_in_sensor(
    Q_0=54000,
    P_0=300000,
    h_0=63e3)                                                                annotation (Placement(transformation(extent={{354,52.679},{368,66.679}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor CW_T_out_sensor(
    Q_0=54000,
    P_0=300000,
    h_0=105e3,
    T_0=298.15)                                                                  annotation (Placement(transformation(extent={{431,52.679},{445,66.679}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_purge_sensor(Q_0=5, h_0=1154502)
                                                                         annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-180,-142})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-180,-160})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.LoopBreaker loopBreaker annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-142,-70})));
  Power.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-238,-80},{-218,-60}})));
  Sensors.Power.PowerSensor thermal_power_sensor annotation (Placement(transformation(extent={{-212,-78},{-196,-62}})));
equation

  // SteamGenerator

    // Quantities definitions
    P_steam_sensor.P_barA = P_steam;
    Q_feedwater_sensor.Q = Q_feedwater;
    Q_purge_sensor.Q = Q_purge;
    thermal_power_sensor.W_MW = thermal_power;

    // Parameters
    steam_generator.vapor_fraction = 0.99;

    // Hypothesis
    steam_generator.P_purge = P_steam * 1e5;

  // HP Turbines

    // Quantities definitions
    HPT_P_in_sensor.P_barA = HPT_P_in;
    HP_control_valve_opening_sensor.Opening = HP_control_valve_opening;
    HP_extract_P_sensor.P_barA = HP_extract_P;
    HPT_P_out_sensor.P_barA = HPT_P_out;

    // Calibrated parameters
  HP_control_valve.Cv_max = HP_control_valve_Cvmax;
    HPT_1.Cst = HPT1_Cst;
    HPT_1.eta_is = turbines_eta_is;
    HPT_2.Cst = HPT2_Cst;
    HPT_2.eta_is = turbines_eta_is;

    // Parameter
    HP_extract.alpha = 1;

  // Dryer - Superheater

    // Quantities definitions
    superheater_bleed_P_sensor.P_barA = superheater_bleed_P;
    superheater_drains_P_sensor.P_barA = superheater_drains_P;
    superheater_T_out_sensor.T_degC = superheater_T_out;

    // Parameters
    superheater.S = 100;
    superheater.Kfr_cold = 0;
    superheater.Q_vent = 1;
    steam_dryer.x_steam_out = 0.99;

    // Calibrated parameters
    superheater.Kfr_hot = superheater_Kfr_hot;
    superheater.Kth = superheater_Kth;
  superheater_control_valve.Cv_max = superheater_control_valve_Cvmax;

    // Hypothesis
    superheater_control_valve.Opening = 1;


  // LP Turbines and extraction

    // Quantities definitions
    LP_extract_P_sensor.P_barA = LP_extract_P;

    // Calibrated parameters
    LPT1.Cst = LPT1_Cst;
    LPT1.eta_is = turbines_eta_is;
    LPT2.Cst = LPT2_Cst;
    LPT2.eta_is = turbines_eta_is;

    // Parameters
    LP_extract.alpha = 1;

  // Generator

    // Quantities definitions
    W_elec_sensor.W_MW = W_elec;

    // Fixed parameter
    generator.eta = 0.99;

  // Condenser

    // Quantities definitions
    CW_P_in_sensor.P_barA = CW_P_in; // BC
    CW_T_in_sensor.T_degC = CW_T_in; // BC
    P_cond_sensor.P_mbar = P_cond;
    CW_T_out_sensor.T_degC = CW_T_out;

    // Parameters
    condenser.S = 100;
    condenser.water_height = 1;
    condenser.C_incond = 0;
    condenser.P_offset = 0;
    condenser.Kfr_cold = 0;

    // Calibrated Parameters
    condenser.Kth = condenser_Kth;
    condenser.Q_cold = condenser_Q_cold;

  // Extraction pump

    // Quantities defintions
    extraction_pump_T_out_sensor.T_degC = extraction_pump_T_out;
    extraction_pump_P_out_sensor.P_barA = extraction_pump_P_out;

    // Parameters
    extraction_pump.VRotn = 4000;
    extraction_pump.VRot = 3950;
    extraction_pump.rm = 0.85;
    extraction_pump.a1 = 0;
    extraction_pump.a2 = 0;
    extraction_pump.b1 = 0;
    extraction_pump.b2 = 0;
  extraction_pump.rh_min = 0.2;

    // Calibrated parameters
    extraction_pump.hn = extraction_pump_hn;
    extraction_pump.rh = extraction_pump_rh;

  // LP Heater and drains

    // Quantities definitions
    LP_heater_P_out_sensor.P_barA = LP_heater_P_out;
    LP_heater_T_out_sensor.T_degC = LP_heater_T_out;
    LP_reheater_drains_control_valve_opening_sensor.Opening = LP_reheater_drains_control_valve_opening;

    // Parameters
    LP_heater.S = 100;
    LP_heater.Kfr_hot = 0;

    // Calibrated parameters
    LP_heater.Kth = LP_heater_Kth;
    LP_heater.Kfr_cold = LP_heater_Kfr_cold;
  LP_reheater_drains_control_valve.Cv_max = LP_heater_drains_control_valve_Cvmax;


  // Deaerator

    deaerator_inlet_pipe.Kfr = 0;
    deaerator_inlet_pipe.delta_z = 5;
    deaerator_outlet_pipe.Kfr = 0;
    deaerator_outlet_pipe.delta_z = -5;

  // Feedwater pump

    // Quantities definitions
    HP_pump_T_out_sensor.T_degC = HP_pump_T_out;
    HP_pump_P_out_sensor.P_barA = HP_pump_P_out;

    // Parameters
    feedwater_pump.VRotn = 4000;
    feedwater_pump.VRot = 4000;
    feedwater_pump.rm = 0.85;
    feedwater_pump.a1 = 0;
    feedwater_pump.a2 = 0;
    feedwater_pump.b1 = 0;
    feedwater_pump.b2 = 0;
  feedwater_pump.rh_min = 0.2;

    // Calibrated parameters
    feedwater_pump.hn = feedwater_pump_hn;
    feedwater_pump.rh = feedwater_pump_rh;


  // HP Reheater and drains

    // Quantities definitions
    HP_heater_P_out_sensor.P_barA = HP_heater_P_out;
    HP_heater_T_out_sensor.T_degC = HP_heater_T_out;
    HP_heater_T_drains_sensor.T_degC = HP_heater_T_drains;
    HP_reheater_drains_control_valve_opening_sensor.Opening = HP_reheater_drains_control_valve_opening;

    // Parameters
    HP_heater.S = 100;
    HP_heater.Kfr_hot = 0;
    HP_heater.level = 0.3;

    // Calibrated parameters
    HP_heater.Kth_subc = HP_heater_Kth_subc;
    HP_heater.Kth_cond = HP_heater_Kth_cond;
    HP_heater.Kfr_cold = HP_heater_Kfr_cold;
  HP_reheater_drains_control_valve.Cv_max = HP_heater_drains_control_valve_Cvmax;


  connect(HP_control_valve.C_out, HPT_P_in_sensor.C_in) annotation (Line(points={{-135,80},{-126,80},{-126,80},{-116,80}},                            color={28,108,200}));
  connect(HP_control_valve.Opening, HP_control_valve_opening_sensor.Opening) annotation (Line(points={{-140,88.7273},{-140,94.9}}, color={0,0,127}));
  connect(HPT_1.C_out, HP_extract.C_in) annotation (Line(points={{-71,80},{-50.6,80}},           color={28,108,200}));
  connect(HP_extract.C_main_out, HPT_2.C_in) annotation (Line(points={{-29.4,80},{-9,80}},           color={28,108,200}));
  connect(HP_extract.C_ext_out, HP_extract_P_sensor.C_in) annotation (Line(points={{-40,73.2},{-40,67}},             color={28,108,200}));
  connect(powerSink.C_in,W_elec_sensor. C_out) annotation (Line(points={{369,180},{361.88,180}},
                                                                                              color={244,125,35}));
  connect(W_elec_sensor.C_in,generator. C_out) annotation (Line(points={{350,180},{344,180}},
                                                                                           color={244,125,35}));
  connect(HPT_2.C_W_out, generator.C_in) annotation (Line(points={{9,86.72},{20,86.72},{20,180},{317.6,180}},      color={244,125,35}));
  connect(HPT_1.C_W_out, generator.C_in) annotation (Line(points={{-71,86.72},{-60,86.72},{-60,180},{317.6,180}},     color={244,125,35}));
  connect(HPT_1.C_in, HPT_P_in_sensor.C_out) annotation (Line(points={{-89,80},{-104,80}},          color={28,108,200}));
  connect(HPT_P_out_sensor.C_in, HPT_2.C_out) annotation (Line(points={{26,80},{9,80}},            color={28,108,200}));
  connect(steam_dryer.C_in, HPT_P_out_sensor.C_out) annotation (Line(points={{56,99.2726},{56,100},{46,100},{46,80},{38,80}},      color={28,108,200}));
  connect(superheater.C_cold_in, steam_dryer.C_hot_steam) annotation (Line(points={{72,112},{72,99.2726}},              color={28,108,200}));
  connect(superheater.C_hot_out, superheater_drains_P_sensor.C_in) annotation (Line(points={{88,120},{100,120}}, color={28,108,200}));
  connect(superheater_T_out_sensor.C_in,superheater. C_cold_out) annotation (Line(points={{88,140},{72,140},{72,128}},         color={28,108,200}));
  connect(superheater_control_valve.C_in, HP_control_valve.C_in) annotation (Line(points={{-145,120},{-180,120},{-180,80},{-145,80}},                   color={28,108,200}));
  connect(superheater.C_hot_in, superheater_bleed_P_sensor.C_out) annotation (Line(points={{56,120},{-104,120}},                                color={28,108,200}));
  connect(superheater_bleed_P_sensor.C_in, superheater_control_valve.C_out) annotation (Line(points={{-116,120},{-126,120},{-126,120},{-135,120}},                 color={28,108,200}));
  connect(LPT1.C_W_out, generator.C_in) annotation (Line(points={{169,146.72},{180,146.72},{180,180},{317.6,180}},   color={244,125,35}));
  connect(LPT2.C_W_out, generator.C_in) annotation (Line(points={{249,146.72},{260,146.72},{260,180},{317.6,180}},   color={244,125,35}));
  connect(LPT2.C_out, P_cond_sensor.C_in) annotation (Line(points={{249,140},{286,140}},                             color={28,108,200}));
  connect(P_cond_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{298,140},{400.5,140},{400.5,74.2864}},
                                                                                                           color={28,108,200}));
  connect(superheater_T_out_sensor.C_out, LPT1.C_in) annotation (Line(points={{100,140},{151,140}},                             color={28,108,200}));
  connect(extraction_pump.C_out, extraction_pump_T_out_sensor.C_in) annotation (Line(points={{352,-80},{338,-80}}, color={28,108,200}));
  connect(extraction_pump_T_out_sensor.C_out,extraction_pump_P_out_sensor. C_in) annotation (Line(points={{324,-80},{308,-80}}, color={28,108,200}));
  connect(condenser.C_hot_out, extraction_pump.C_in) annotation (Line(points={{400.5,48.2222},{400.5,-80},{368,-80}},
                                                                                                                  color={28,108,200}));
  connect(LP_heater.C_cold_out, LP_heater_P_out_sensor.C_in) annotation (Line(points={{244,-80},{237,-80}}, color={28,108,200}));
  connect(LP_heater_P_out_sensor.C_out,LP_heater_T_out_sensor. C_in) annotation (Line(points={{223,-80},{210,-80},{210,-80.125},{207,-80.125},{207,-80}},
                                                                                                      color={28,108,200}));
  connect(extraction_pump_P_out_sensor.C_out, LP_heater.C_cold_in) annotation (Line(points={{294,-80},{276.2,-80}}, color={28,108,200}));
  connect(LP_extract_P_sensor.C_out, LP_heater.C_hot_in) annotation (Line(points={{200,103},{200,40},{260,40},{260,-72}}, color={28,108,200}));
  connect(LP_extract_P_sensor.C_in, LP_extract.C_ext_out) annotation (Line(points={{200,117},{200,133.2}}, color={28,108,200}));
  connect(LPT1.C_out, LP_extract.C_in) annotation (Line(points={{169,140},{189.4,140}},                                 color={28,108,200}));
  connect(LPT2.C_in, LP_extract.C_main_out) annotation (Line(points={{231,140},{210.6,140}},                                 color={28,108,200}));
  connect(deaerator_inlet_pipe.C_in, LP_heater_T_out_sensor.C_out) annotation (Line(points={{180,-80},{193,-80}}, color={28,108,200}));
  connect(steam_dryer.C_hot_liquid, steam_dryer_liq_out_pipe.C_in) annotation (Line(points={{72,92.7272},{72,-26},{140,-26},{140,-50}},
                                                                                                                                     color={28,108,200}));
  connect(deaerator_inlet_pipe.C_out, deaerator_outlet_pipe.C_in) annotation (Line(points={{160,-80},{114,-80}}, color={28,108,200}));
  connect(steam_dryer_liq_out_pipe.C_out, deaerator_outlet_pipe.C_in) annotation (Line(points={{140,-70},{140,-80},{114,-80}}, color={28,108,200}));
  connect(feedwater_pump.C_out, HP_pump_T_out_sensor.C_in) annotation (Line(points={{46,-80},{30,-80}}, color={28,108,200}));
  connect(HP_pump_T_out_sensor.C_out, HP_pump_P_out_sensor.C_in) annotation (Line(points={{16,-80},{8,-80}},  color={28,108,200}));
  connect(feedwater_pump.C_in, deaerator_outlet_pipe.C_out) annotation (Line(points={{62,-80},{94,-80}}, color={28,108,200}));
  connect(HP_pump_P_out_sensor.C_out, HP_heater.C_cold_in) annotation (Line(points={{-6,-80},{-23.8,-80}},  color={28,108,200}));
  connect(HP_heater.C_cold_out, HP_heater_P_out_sensor.C_in) annotation (Line(points={{-56,-80},{-64,-80}}, color={28,108,200}));
  connect(HP_heater_P_out_sensor.C_out, HP_heater_T_out_sensor.C_in) annotation (Line(points={{-78,-80},{-84,-80}},                             color={28,108,200}));
  connect(HP_heater_T_drains_sensor.C_in, HP_heater.C_hot_out) annotation (Line(points={{-40,-93},{-40,-88}}, color={28,108,200}));
  connect(HP_extract_P_sensor.C_out, HP_heater.C_hot_in) annotation (Line(points={{-40,53},{-40,-72}}, color={28,108,200}));
  connect(superheater_drains_P_sensor.C_out, superheater_drains_pipe.C_in) annotation (Line(points={{112,120},{120,120},{120,48}}, color={28,108,200}));
  connect(superheater_drains_pipe.C_out, HP_heater.C_hot_in) annotation (Line(points={{120,28},{120,10},{-40,10},{-40,-72}}, color={28,108,200}));
  connect(HP_heater_T_out_sensor.C_out, Q_feedwater_sensor.C_in) annotation (Line(points={{-98,-80},{-104,-80}}, color={28,108,200}));
  connect(HP_reheater_drains_control_valve.Opening, HP_reheater_drains_control_valve_opening_sensor.Opening) annotation (Line(points={{11,-111.091},{11,-106.1}}, color={0,0,127}));
  connect(HP_heater_T_drains_sensor.C_out, HP_reheater_drains_control_valve.C_in) annotation (Line(points={{-40,-107},{-40,-119.818},{6,-119.818}}, color={28,108,200}));
  connect(LP_reheater_drains_control_valve.Opening, LP_reheater_drains_control_valve_opening_sensor.Opening) annotation (Line(points={{293,-111.091},{293,-106.1}}, color={0,0,127}));
  connect(LP_heater.C_hot_out, LP_reheater_drains_control_valve.C_in) annotation (Line(points={{260,-88},{260,-119.818},{288,-119.818}}, color={28,108,200}));
  connect(steam_generator.steam_outlet, P_steam_sensor.C_in) annotation (Line(points={{-180,-34},{-180,3}},  color={28,108,200}));
  connect(P_steam_sensor.C_out, HP_control_valve.C_in) annotation (Line(points={{-180,17},{-180,80},{-145,80}},           color={28,108,200}));
  connect(superheater.C_vent, pressureCut.C_in) annotation (Line(points={{88,112.2},{88,94},{94,94}}, color={28,108,200}));
  connect(pressureCut.C_out, superheater_drains_pipe.C_in) annotation (Line(points={{114,94},{120,94},{120,48}}, color={28,108,200}));
  connect(cold_source.C_out, CW_T_in_sensor.C_in) annotation (Line(points={{313,59.679},{326,59.679}},                         color={28,108,200}));
  connect(CW_T_in_sensor.C_out, CW_P_in_sensor.C_in) annotation (Line(points={{340,59.679},{354,59.679}},
                                                                                                  color={28,108,200}));
  connect(CW_P_in_sensor.C_out, condenser.C_cold_in) annotation (Line(points={{368,59.679},{385,59.679}},                        color={28,108,200}));
  connect(CW_T_out_sensor.C_out, cold_sink.C_in) annotation (Line(points={{445,59.679},{463,59.679}},                         color={28,108,200}));
  connect(condenser.C_cold_out, CW_T_out_sensor.C_in) annotation (Line(points={{415.69,59.679},{431,59.679}},                      color={28,108,200}));
  connect(LP_reheater_drains_control_valve.C_out, condenser.C_hot_in) annotation (Line(points={{298,-119.818},{400,-119.818},{400,-120},{500,-120},{500,108},{400.5,108},{400.5,74.2864}},
                                                                                                                                                                                  color={28,108,200}));
  connect(HP_reheater_drains_control_valve.C_out, deaerator_outlet_pipe.C_in) annotation (Line(points={{16,-121.818},{78,-121.818},{78,-122},{142,-122},{142,-70},{114,-70}}, color={28,108,200}));
  connect(steam_generator.purge_outlet, Q_purge_sensor.C_in) annotation (Line(points={{-170,-115.233},{-170,-125}},             color={28,108,200}));
  connect(Q_feedwater_sensor.C_out, loopBreaker.C_in) annotation (Line(points={{-118,-70},{-132,-70}}, color={28,108,200}));
  connect(loopBreaker.C_out, steam_generator.feedwater_inlet) annotation (Line(points={{-152,-70},{-159,-70}}, color={28,108,200}));
  connect(Q_purge_sensor.C_out, sink.C_in) annotation (Line(points={{-170,-139},{-170,-145}}, color={28,108,200}));
  connect(steam_generator.C_thermal_power, thermal_power_sensor.C_out) annotation (Line(points={{-181,-70},{-196.16,-70}}, color={244,125,35}));
  connect(thermal_power_sensor.C_in, source.C_out) annotation (Line(points={{-212,-70},{-223.2,-70}}, color={244,125,35}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,-160},{520,200}})), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-240,-160},{520,200}})));
end MetroscopiaNPP_reverse;
