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

    MetroscopeModelingLibrary.WaterSteam.HeatExchangers.SteamGenerator steam_generator annotation (Placement(transformation(extent={{-192,-116},{-148,-24}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_feedwater_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6)                                                               annotation (Placement(transformation(extent={{-104,-77},{-118,-63}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve HP_control_valve(
    T_out_0=535.15,
    P_in_0=5000000,
    P_out_0=4850000,
    h_in_0=2.8e6,
    h_out_0=2.8e6,
    Q_0=1455,
    T_0=536.15,
    h_0=2.8e6)                                                                                            annotation (Placement(transformation(extent={{-135,69.8182},{-125,81.8182}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor HP_control_valve_opening_sensor annotation (Placement(transformation(extent={{-136,86},{-126,96}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor HPT_P_in_sensor(
    Q_0=1455,
    P_0=4850000,
    h_0=2.8e6)                                                                annotation (Placement(transformation(extent={{-106,66},{-94,78}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine HPT_1(
    T_in_0=535.15,
    T_out_0=608.85,
    P_in_0=4850000,
    P_out_0=3100000,
    h_in_0=2.8e6,
    h_out_0=2.7e6,
    Q_0=1455) annotation (Placement(transformation(extent={{-79,64},{-61,80}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine HPT_2(
    T_in_0=508.85,
    T_out_0=484.15,
    P_in_0=3100000,
    P_out_0=1940000,
    h_in_0=2.73e6,
    h_out_0=2.68e6,
    Q_0=1113) annotation (Placement(transformation(extent={{-9,64},{9,80}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter HP_extract(
    Q_in_0=1455,
    Q_ext_0=340,
    P_0=3100000,
    T_0=508.85,
    h_0=2.73e6)                                                                   annotation (Placement(transformation(extent={{-50,62},{-30,80}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor HP_extract_P_sensor(
    Q_0=340,
    P_0=3100000,
    h_0=2.73e6)                                                                   annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-40,47})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor HPT_P_out_sensor(
    Q_0=1113,
    P_0=1940000,
    h_0=2.68e6)                                                                annotation (Placement(transformation(extent={{26,66},{38,78}})));
    MetroscopeModelingLibrary.WaterSteam.Volumes.SteamDryer steam_dryer(
    P_0=1940000,
    T_0=483.95,
    h_in_0=2.68e6,
    Q_in_0=1113,
    Q_liq_0=50)                                                         annotation (Placement(transformation(extent={{56,79.8182},{72,97.8182}})));
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
    h_hot_out_0=1.09e6)                                                         annotation (Placement(transformation(extent={{56,104},{88,120}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor superheater_drains_P_sensor(
    Q_0=44,
    P_0=4000000,
    h_0=1.09e6)                                                                           annotation (Placement(transformation(extent={{100,106},{112,118}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor superheater_T_out_sensor(
    Q_0=1060,
    P_0=1940000,
    h_0=2.85e6,
    T_0=501.15)                                                                           annotation (Placement(transformation(extent={{88,124},{100,136}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve superheater_control_valve(
    P_in_0=5000000,
    P_out_0=4100000,
    h_in_0=2.778e6,
    h_out_0=2.778e6,
    Q_0=45,
    T_0=537.15,
    h_0=2.8e6)                                                                        annotation (Placement(transformation(extent={{-136,110},{-126,122}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor superheater_bleed_P_sensor(
    Q_0=45,
    P_0=4100000,
    h_0=2.778e6)                                                                         annotation (Placement(transformation(extent={{-106,106.182},{-94,118.182}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.PressureCut superheater_drains_pipe(
    P_in_0=4000000,
    P_out_0=3100000,
    Q_0=44,
    T_0=525.15,
    h_0=1.09e6)                                                                    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={122,30})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine LPT1(
    T_in_0=501.15,
    T_out_0=425.15,
    P_in_0=1940000,
    P_out_0=500000,
    h_in_0=2.85e6,
    h_out_0=2.7e6,
    Q_0=1060) annotation (Placement(transformation(extent={{151,122},{169,138}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine LPT2(
    T_in_0=425.15,
    T_out_0=312.05,
    P_in_0=500000,
    P_out_0=6900,
    h_in_0=2.7e6,
    h_out_0=2.4e6,
    Q_0=1000) annotation (Placement(transformation(extent={{221,122},{239,138}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor LP_extract_P_sensor(
    Q_0=55,
    P_0=500000,
    h_0=2.7e6)                                                                    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={196,111})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter LP_extract(
    Q_in_0=1060,
    Q_ext_0=55,
    P_0=500000,
    T_0=425.15,
    h_0=2.7e6)                                                                    annotation (Placement(transformation(extent={{186,120},{206,138}})));
    MetroscopeModelingLibrary.Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{362,158},{382,178}})));
    MetroscopeModelingLibrary.Power.Machines.Generator generator annotation (Placement(transformation(extent={{308,156},{348,180}})));
    MetroscopeModelingLibrary.Sensors.Power.PowerSensor W_elec_sensor annotation (Placement(transformation(extent={{348,162},{360,174}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cond_sensor(
    Q_0=1000,
    P_0=6900,
    h_0=2.4e6)                                                              annotation (Placement(transformation(extent={{286,124},{298,136}})));
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
    h_hot_in_0=2.4e6)                                                                      annotation (Placement(transformation(extent={{377,48.2222},{408,74}})));
    MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={460,60})));
    MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{290,57.7778},{310,77.7778}})));
    WaterSteam.Machines.FixedSpeedPump                 extraction_pump(
    T_in_0=312.05,
    T_out_0=312.15,
    P_in_0=6980,
    P_out_0=700000,
    h_in_0=163e3,
    h_out_0=164e3,
    Q_0=1060)                                                                       annotation (Placement(transformation(extent={{380,-78},{364,-62}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor extraction_pump_T_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=164e3,
    T_0=312.15)                                                                               annotation (Placement(transformation(extent={{350,-77},{336,-63}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor extraction_pump_P_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=164e3)                                                                             annotation (Placement(transformation(extent={{7,-7},{-7,7}}, origin={315,-70})));
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
    h_hot_out_0=640e3)                                                        annotation (Placement(transformation(extent={{284,-78},{252,-62}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor LP_heater_T_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=272e3,
    T_0=338.15)                                                                         annotation (Placement(transformation(extent={{220,-77},{206,-63}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor LP_heater_P_out_sensor(
    Q_0=1060,
    P_0=700000,
    h_0=272e3)                                                                       annotation (Placement(transformation(extent={{242,-77},{228,-63}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve LP_reheater_drains_control_valve(
    P_in_0=500000,
    P_out_0=6900,
    Q_0=55,
    T_0=425.15,
    h_0=640e3)                                                                               annotation (Placement(transformation(extent={{288,-122},{298,-110}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor LP_reheater_drains_control_valve_opening_sensor annotation (Placement(transformation(extent={{288,-106},{298,-96}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.FrictionPipe deaerator_inlet_pipe(
    P_in_0=600000,
    P_out_0=550000,
    Q_0=1060,
    T_0=338.15,
    h_0=272e3) annotation (Placement(transformation(extent={{186,-80},{166,-60}})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.PressureCut steam_dryer_liq_out_pipe(
    P_in_0=1940000,
    P_out_0=1940000,
    Q_0=50,
    T_0=483.95,
    h_0=901606.56)                                                                  annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={142,-50})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.FrictionPipe deaerator_outlet_pipe(
    P_in_0=550000,
    P_out_0=600000,
    Q_0=1500,
    T_0=350.05,
    h_0=322e3) annotation (Placement(transformation(extent={{114,-80},{94,-60}})));
    WaterSteam.Machines.FixedSpeedPump                 feedwater_pump(
    T_in_0=350.05,
    T_out_0=353.15,
    P_in_0=600000,
    P_out_0=5900000,
    h_in_0=322e3,
    h_out_0=340e3,
    Q_0=1500)                                                                                         annotation (Placement(transformation(extent={{62,-78},{46,-62}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor HP_pump_T_out_sensor(
    Q_0=1500,
    P_0=5900000,
    h_0=340e3,
    T_0=353.15)                                                                       annotation (Placement(transformation(extent={{30,-77},{16,-63}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor HP_pump_P_out_sensor(
    Q_0=1500,
    P_0=5900000,
    h_0=340e3)                                                                     annotation (Placement(transformation(extent={{7,-7},{-7,7}}, origin={-3,-70})));
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
    h_hot_out_0=379e3)                                                                                annotation (Placement(transformation(extent={{-24,-78},{-56,-62}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor HP_heater_T_out_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6)                                                                          annotation (Placement(transformation(extent={{-84,-77},{-98,-63}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor HP_heater_P_out_sensor(
    Q_0=1500,
    P_0=5800000,
    h_0=0.9e6)                                                                       annotation (Placement(transformation(extent={{-64,-77},{-78,-63}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor HP_heater_T_drains_sensor(
    Q_0=387,
    P_0=3100000,
    h_0=379e3,
    T_0=363.15)                                                                            annotation (Placement(transformation(
        extent={{7,-7},{-7,7}},
        rotation=90,
        origin={-40,-98})));
    MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve HP_reheater_drains_control_valve annotation (Placement(transformation(extent={{6,-124},{16,-112}})));
  MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor HP_reheater_drains_control_valve_opening_sensor annotation (Placement(transformation(extent={{6,-108},{16,-98}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_steam_sensor(
    Q_0=1500,
    P_0=5000000,
    h_0=2.778e6)                                                             annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=90,
        origin={-170,10})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.PressureCut pressureCut(
    P_in_0=4000000,
    P_out_0=4000000,
    Q_0=2,
    T_0=525.15,
    h_0=2.8e6)                                                       annotation (Placement(transformation(extent={{94,76},{114,96}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor CW_T_in_sensor(
    Q_0=54000,
    P_0=300000,
    h_0=63e3,
    T_0=288.15)                                                                 annotation (Placement(transformation(extent={{318,60},{332,74}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor CW_P_in_sensor(
    Q_0=54000,
    P_0=300000,
    h_0=63e3)                                                                annotation (Placement(transformation(extent={{346,60},{360,74}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor CW_T_out_sensor(
    Q_0=54000,
    P_0=300000,
    h_0=105e3,
    T_0=298.15)                                                                  annotation (Placement(transformation(extent={{423,53},{437,67}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_purge_sensor(Q_0=5, h_0=1154502)
                                                                         annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-170,-132})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-170,-150})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.LoopBreaker loopBreaker annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-142,-70})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-238,-80},{-218,-60}})));
  MetroscopeModelingLibrary.Sensors.Power.PowerSensor thermal_power_sensor annotation (Placement(transformation(extent={{-212,-78},{-196,-62}})));
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

  connect(HP_control_valve.C_out, HPT_P_in_sensor.C_in) annotation (Line(points={{-125,72},{-116,72},{-116,72},{-106,72}},                            color={28,108,200}));
  connect(HP_control_valve.Opening, HP_control_valve_opening_sensor.Opening) annotation (Line(points={{-130,80.7273},{-130,82},{-131,82},{-131,85.9}},
                                                                                                                                   color={0,0,127}));
  connect(HPT_1.C_out, HP_extract.C_in) annotation (Line(points={{-61,72},{-50.6,72}},           color={28,108,200}));
  connect(HP_extract.C_main_out, HPT_2.C_in) annotation (Line(points={{-29.4,72},{-9,72}},           color={28,108,200}));
  connect(HP_extract.C_ext_out, HP_extract_P_sensor.C_in) annotation (Line(points={{-40,65.2},{-40,54}},             color={28,108,200}));
  connect(powerSink.C_in,W_elec_sensor. C_out) annotation (Line(points={{367,168},{359.88,168}},
                                                                                              color={244,125,35}));
  connect(W_elec_sensor.C_in,generator. C_out) annotation (Line(points={{348,168},{342,168}},
                                                                                           color={244,125,35}));
  connect(HPT_2.C_W_out, generator.C_in) annotation (Line(points={{9,78.72},{18,78.72},{18,168},{315.6,168}},      color={244,125,35}));
  connect(HPT_1.C_W_out, generator.C_in) annotation (Line(points={{-61,78.72},{-54,78.72},{-54,168},{315.6,168}},     color={244,125,35}));
  connect(HPT_1.C_in, HPT_P_in_sensor.C_out) annotation (Line(points={{-79,72},{-94,72}},           color={28,108,200}));
  connect(HPT_P_out_sensor.C_in, HPT_2.C_out) annotation (Line(points={{26,72},{9,72}},            color={28,108,200}));
  connect(steam_dryer.C_in, HPT_P_out_sensor.C_out) annotation (Line(points={{56,91.2727},{56,92},{46,92},{46,72},{38,72}},        color={28,108,200}));
  connect(superheater.C_cold_in, steam_dryer.C_hot_steam) annotation (Line(points={{72,104},{72,91.2727}},              color={28,108,200}));
  connect(superheater.C_hot_out, superheater_drains_P_sensor.C_in) annotation (Line(points={{88,112},{100,112}}, color={28,108,200}));
  connect(superheater_T_out_sensor.C_in,superheater. C_cold_out) annotation (Line(points={{88,130},{72,130},{72,120}},         color={28,108,200}));
  connect(superheater_control_valve.C_in, HP_control_valve.C_in) annotation (Line(points={{-136,112.182},{-136,112},{-158,112},{-158,72},{-135,72}},    color={28,108,200}));
  connect(superheater.C_hot_in, superheater_bleed_P_sensor.C_out) annotation (Line(points={{56,112},{-35,112},{-35,112.182},{-94,112.182}},     color={28,108,200}));
  connect(superheater_bleed_P_sensor.C_in, superheater_control_valve.C_out) annotation (Line(points={{-106,112.182},{-116,112.182},{-116,112.182},{-126,112.182}}, color={28,108,200}));
  connect(LPT1.C_W_out, generator.C_in) annotation (Line(points={{169,136.72},{188,136.72},{188,168},{315.6,168}},   color={244,125,35}));
  connect(LPT2.C_W_out, generator.C_in) annotation (Line(points={{239,136.72},{262,136.72},{262,168},{315.6,168}},   color={244,125,35}));
  connect(LPT2.C_out, P_cond_sensor.C_in) annotation (Line(points={{239,130},{286,130}},                             color={28,108,200}));
  connect(P_cond_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{298,130},{392.5,130},{392.5,74.2864}},
                                                                                                           color={28,108,200}));
  connect(superheater_T_out_sensor.C_out, LPT1.C_in) annotation (Line(points={{100,130},{151,130}},                             color={28,108,200}));
  connect(extraction_pump.C_out, extraction_pump_T_out_sensor.C_in) annotation (Line(points={{364,-70},{350,-70}}, color={28,108,200}));
  connect(extraction_pump_T_out_sensor.C_out,extraction_pump_P_out_sensor. C_in) annotation (Line(points={{336,-70},{322,-70}}, color={28,108,200}));
  connect(condenser.C_hot_out, extraction_pump.C_in) annotation (Line(points={{392.5,48.2222},{392.5,-70},{380,-70}},
                                                                                                                  color={28,108,200}));
  connect(LP_heater.C_cold_out, LP_heater_P_out_sensor.C_in) annotation (Line(points={{252,-70},{242,-70}}, color={28,108,200}));
  connect(LP_heater_P_out_sensor.C_out,LP_heater_T_out_sensor. C_in) annotation (Line(points={{228,-70},{224,-70},{224,-70.5},{220,-70.5},{220,-70}},
                                                                                                      color={28,108,200}));
  connect(extraction_pump_P_out_sensor.C_out, LP_heater.C_cold_in) annotation (Line(points={{308,-70},{284.2,-70}}, color={28,108,200}));
  connect(LP_extract_P_sensor.C_out, LP_heater.C_hot_in) annotation (Line(points={{196,104},{196,44},{268,44},{268,-62}}, color={28,108,200}));
  connect(LP_extract_P_sensor.C_in, LP_extract.C_ext_out) annotation (Line(points={{196,118},{196,123.2}}, color={28,108,200}));
  connect(LPT1.C_out, LP_extract.C_in) annotation (Line(points={{169,130},{185.4,130}},                                 color={28,108,200}));
  connect(LPT2.C_in, LP_extract.C_main_out) annotation (Line(points={{221,130},{206.6,130}},                                 color={28,108,200}));
  connect(deaerator_inlet_pipe.C_in, LP_heater_T_out_sensor.C_out) annotation (Line(points={{186,-70},{206,-70}}, color={28,108,200}));
  connect(steam_dryer.C_hot_liquid, steam_dryer_liq_out_pipe.C_in) annotation (Line(points={{72,84.7273},{72,-34},{142,-34},{142,-40}},
                                                                                                                                     color={28,108,200}));
  connect(deaerator_inlet_pipe.C_out, deaerator_outlet_pipe.C_in) annotation (Line(points={{166,-70},{114,-70}}, color={28,108,200}));
  connect(steam_dryer_liq_out_pipe.C_out, deaerator_outlet_pipe.C_in) annotation (Line(points={{142,-60},{142,-70},{114,-70}}, color={28,108,200}));
  connect(feedwater_pump.C_out, HP_pump_T_out_sensor.C_in) annotation (Line(points={{46,-70},{30,-70}}, color={28,108,200}));
  connect(HP_pump_T_out_sensor.C_out, HP_pump_P_out_sensor.C_in) annotation (Line(points={{16,-70},{4,-70}},  color={28,108,200}));
  connect(feedwater_pump.C_in, deaerator_outlet_pipe.C_out) annotation (Line(points={{62,-70},{94,-70}}, color={28,108,200}));
  connect(HP_pump_P_out_sensor.C_out, HP_heater.C_cold_in) annotation (Line(points={{-10,-70},{-23.8,-70}}, color={28,108,200}));
  connect(HP_heater.C_cold_out, HP_heater_P_out_sensor.C_in) annotation (Line(points={{-56,-70},{-64,-70}}, color={28,108,200}));
  connect(HP_heater_P_out_sensor.C_out, HP_heater_T_out_sensor.C_in) annotation (Line(points={{-78,-70},{-84,-70}},                             color={28,108,200}));
  connect(HP_heater_T_drains_sensor.C_in, HP_heater.C_hot_out) annotation (Line(points={{-40,-91},{-40,-78}}, color={28,108,200}));
  connect(HP_extract_P_sensor.C_out, HP_heater.C_hot_in) annotation (Line(points={{-40,40},{-40,-62}}, color={28,108,200}));
  connect(superheater_drains_P_sensor.C_out, superheater_drains_pipe.C_in) annotation (Line(points={{112,112},{122,112},{122,40}}, color={28,108,200}));
  connect(superheater_drains_pipe.C_out, HP_heater.C_hot_in) annotation (Line(points={{122,20},{122,16},{-40,16},{-40,-62}}, color={28,108,200}));
  connect(HP_heater_T_out_sensor.C_out, Q_feedwater_sensor.C_in) annotation (Line(points={{-98,-70},{-104,-70}}, color={28,108,200}));
  connect(HP_reheater_drains_control_valve.Opening, HP_reheater_drains_control_valve_opening_sensor.Opening) annotation (Line(points={{11,-113.091},{11,-108.1}}, color={0,0,127}));
  connect(HP_heater_T_drains_sensor.C_out, HP_reheater_drains_control_valve.C_in) annotation (Line(points={{-40,-105},{-40,-121.818},{6,-121.818}}, color={28,108,200}));
  connect(LP_reheater_drains_control_valve.Opening, LP_reheater_drains_control_valve_opening_sensor.Opening) annotation (Line(points={{293,-111.091},{293,-106.1}}, color={0,0,127}));
  connect(LP_heater.C_hot_out, LP_reheater_drains_control_valve.C_in) annotation (Line(points={{268,-78},{268,-119.818},{288,-119.818}}, color={28,108,200}));
  connect(steam_generator.steam_outlet, P_steam_sensor.C_in) annotation (Line(points={{-170,-24},{-170,3}},  color={28,108,200}));
  connect(P_steam_sensor.C_out, HP_control_valve.C_in) annotation (Line(points={{-170,17},{-170,72},{-135,72}},           color={28,108,200}));
  connect(superheater.C_vent, pressureCut.C_in) annotation (Line(points={{88,104.2},{88,86},{94,86}}, color={28,108,200}));
  connect(pressureCut.C_out, superheater_drains_pipe.C_in) annotation (Line(points={{114,86},{122,86},{122,40}}, color={28,108,200}));
  connect(cold_source.C_out, CW_T_in_sensor.C_in) annotation (Line(points={{305,67.7778},{305,67},{318,67}},                   color={28,108,200}));
  connect(CW_T_in_sensor.C_out, CW_P_in_sensor.C_in) annotation (Line(points={{332,67},{346,67}}, color={28,108,200}));
  connect(CW_P_in_sensor.C_out, condenser.C_cold_in) annotation (Line(points={{360,67},{378,67},{378,59.679},{377,59.679}},      color={28,108,200}));
  connect(CW_T_out_sensor.C_out, cold_sink.C_in) annotation (Line(points={{437,60},{455,60}},                                 color={28,108,200}));
  connect(condenser.C_cold_out, CW_T_out_sensor.C_in) annotation (Line(points={{407.69,59.679},{407.69,60},{423,60}},              color={28,108,200}));
  connect(LP_reheater_drains_control_valve.C_out, condenser.C_hot_in) annotation (Line(points={{298,-119.818},{400,-119.818},{400,-120},{500,-120},{500,100},{392.5,100},{392.5,74.2864}},
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
