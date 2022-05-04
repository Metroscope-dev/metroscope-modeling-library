within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model MetroscopiaNPP_reverse


  // Boundary Conditions

    input Real P_steam(start = 50, unit="bar", min=0, nominal=50) "barA"; // Steam generator steam pressure
    input Real CW_P_in(start = 3, unit="bar", min=0, nominal=5) "barA"; // Circulating Water inlet pressure
    input Real CW_T_in(start = 15, unit="degC", min=0, nominal=15) "degC"; // Circulating Water inlet temperature
    input Real Q_feedwater(start=1500, unit="kg/s", min=0, nominal=1e3) "kg/s"; // Feedwater flow rate
    input Real Q_purge(start=5, unit = "kg/s", min=0) "kg/s"; // Steam generator blowdown flow


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

  // Calibrated parameters (input used for calibration in comment)
    // HP turbines inlet control valve
    output Units.Cv HP_control_valve_Cvmax; // HP valve opening
    // HP Turbines
    output Units.Cst HPT1_Cst; // HP turbine inlet pressure
    output Units.Cst HPT2_Cst; // HP extract pressure
    output Units.Yield turbines_eta_is; // Welec
    // Superheater inlet control valve
    output Units.Cv superheater_control_valve_Cvmax; // Superheater bleed pressure
    // Superheater
    output Units.FrictionCoefficient superheater_Kfr_hot; // Superheater drains pressure
    output Units.HeatExchangeCoefficient superheater_Kth; // Superheater steam outlet temperature
    // LP Turbines
    output Units.Cst LPT1_Cst; // HP turbine outlet pressure
    output Units.Cst LPT2_Cst; // LP extract pressure
    // Condenser
    output Units.HeatExchangeCoefficient condenser_Kth; // P cond
    output Units.PositiveMassFlowRate condenser_Q_cold; // Circulating water outlet temperature
    // Extraction pump
    output Real extraction_pump_hn; // Extraction pump outlet pressure
    output Real extraction_pump_rh; // Extraction pump outlet temperature
    // LP Heater
    output Units.HeatExchangeCoefficient LP_heater_Kth; // LP heater outlet temperature
    output Units.FrictionCoefficient LP_heater_Kfr_cold; // LP heater outlet pressure
    output Units.Cv LP_heater_drains_control_valve_Cvmax; // LP heater drains valve opening
    // Feedwater pump
    output Real feedwater_pump_hn; // Feedwater pump outlet pressure
    output Real feedwater_pump_rh; // Feedwater pump outlet temperature
    // HP Heater
    output Units.HeatExchangeCoefficient HP_heater_Kth_cond; // HP heater outlet temperature
    output Units.HeatExchangeCoefficient HP_heater_Kth_subc; // HP heater drains temperature
    output Units.FrictionCoefficient HP_heater_Kfr_cold; // HP heater outlet pressure
    output Units.Cv HP_heater_drains_control_valve_Cvmax; // HP heater drains valve opening

    WaterSteam.HeatExchangers.SteamGenerator steam_generator annotation (Placement(transformation(extent={{-192,-116},{-148,-24}})));
  Sensors.WaterSteam.FlowSensor Q_feedwater_sensor annotation (Placement(transformation(extent={{-104,-77},{-118,-63}})));
    WaterSteam.Pipes.ControlValve HP_control_valve(P_in_0=50e5, P_out_0=48.5e5) annotation (Placement(transformation(extent={{-135,69.8182},{-125,81.8182}})));
  Sensors.Outline.OpeningSensor HP_control_valve_opening_sensor annotation (Placement(transformation(extent={{-136,86},{-126,96}})));
  Sensors.WaterSteam.PressureSensor HPT_P_in_sensor annotation (Placement(transformation(extent={{-106,66},{-94,78}})));
    WaterSteam.Machines.StodolaTurbine HPT_1 annotation (Placement(transformation(extent={{-79,64},{-61,80}})));
    WaterSteam.Machines.StodolaTurbine HPT_2 annotation (Placement(transformation(extent={{-9,64},{9,80}})));
    WaterSteam.Pipes.SteamExtractionSplitter HP_extract annotation (Placement(transformation(extent={{-50,62},{-30,80}})));
  Sensors.WaterSteam.PressureSensor HP_extract_P_sensor annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-40,47})));
  Sensors.WaterSteam.PressureSensor HPT_P_out_sensor annotation (Placement(transformation(extent={{26,66},{38,78}})));
    WaterSteam.Volumes.SteamDryer steam_dryer annotation (Placement(transformation(extent={{56,79.8182},{72,97.8182}})));
    WaterSteam.HeatExchangers.Superheater superheater annotation (Placement(transformation(extent={{56,104},{88,120}})));
  Sensors.WaterSteam.PressureSensor superheater_drains_P_sensor annotation (Placement(transformation(extent={{100,106},{112,118}})));
  Sensors.WaterSteam.TemperatureSensor superheater_T_out_sensor annotation (Placement(transformation(extent={{88,124},{100,136}})));
    WaterSteam.Pipes.ControlValve superheater_control_valve annotation (Placement(transformation(extent={{-136,110},{-126,122}})));
  Sensors.WaterSteam.PressureSensor superheater_bleed_P_sensor annotation (Placement(transformation(extent={{-106,106.182},{-94,118.182}})));
    WaterSteam.Pipes.PressureCut superheater_drains_pipe annotation (Placement(transformation(extent={{10,-10},{-10,10}},rotation=90,origin={122,32})));
    WaterSteam.Machines.StodolaTurbine LPT1 annotation (Placement(transformation(extent={{151,122},{169,138}})));
    WaterSteam.Machines.StodolaTurbine LPT2 annotation (Placement(transformation(extent={{221,122},{239,138}})));
  Sensors.WaterSteam.PressureSensor LP_extract_P_sensor annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={196,111})));
    WaterSteam.Pipes.SteamExtractionSplitter LP_extract annotation (Placement(transformation(extent={{186,120},{206,138}})));
    Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{362,158},{382,178}})));
    Power.Machines.Generator generator annotation (Placement(transformation(extent={{308,156},{348,180}})));
    Sensors.Power.PowerSensor W_elec_sensor annotation (Placement(transformation(extent={{348,162},{360,174}})));
  Sensors.WaterSteam.PressureSensor P_cond_sensor annotation (Placement(transformation(extent={{286,124},{298,136}})));
    WaterSteam.HeatExchangers.Condenser condenser(Psat_0=69.8e2) annotation (Placement(transformation(extent={{377,48.2222},{408,74}})));
    WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=0,origin={460,60})));
    WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{290,57.7778},{310,77.7778}})));
    WaterSteam.Machines.Pump extraction_pump(P_in_0=6980) annotation (Placement(transformation(extent={{380,-78},{364,-62}})));
    Power.BoundaryConditions.Source LP_pump_Wm_source annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=270,origin={372,-46})));
  Sensors.WaterSteam.TemperatureSensor extraction_pump_T_out_sensor annotation (Placement(transformation(extent={{350,-77},{336,-63}})));
  Sensors.WaterSteam.PressureSensor extraction_pump_P_out_sensor annotation (Placement(transformation(extent={{7,-7},{-7,7}}, origin={315,-70})));
    WaterSteam.HeatExchangers.DryReheater LP_heater annotation (Placement(transformation(extent={{284,-78},{252,-62}})));
  Sensors.WaterSteam.TemperatureSensor LP_heater_T_out_sensor annotation (Placement(transformation(extent={{220,-77},{206,-63}})));
  Sensors.WaterSteam.PressureSensor LP_heater_P_out_sensor annotation (Placement(transformation(extent={{242,-77},{228,-63}})));
    WaterSteam.Pipes.ControlValve LP_reheater_drains_control_valve annotation (Placement(transformation(extent={{288,-122},{298,-110}})));
  Sensors.Outline.OpeningSensor LP_reheater_drains_control_valve_opening_sensor annotation (Placement(transformation(extent={{288,-106},{298,-96}})));
    WaterSteam.Pipes.Pipe deaerator_inlet_pipe annotation (Placement(transformation(extent={{186,-80},{166,-60}})));
    WaterSteam.Pipes.PressureCut steam_dryer_liq_out_pipe annotation (Placement(transformation(extent={{10,-10},{-10,10}},rotation=90,origin={142,-50})));
    WaterSteam.Pipes.Pipe deaerator_outlet_pipe annotation (Placement(transformation(extent={{114,-80},{94,-60}})));
    WaterSteam.Machines.Pump feedwater_pump(P_in_0=600000, P_out_0=5900000) annotation (Placement(transformation(extent={{62,-78},{46,-62}})));
    Power.BoundaryConditions.Source HP_pump_Wm_source annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=270,origin={54,-46})));
  Sensors.WaterSteam.TemperatureSensor HP_pump_T_out_sensor annotation (Placement(transformation(extent={{30,-77},{16,-63}})));
  Sensors.WaterSteam.PressureSensor HP_pump_P_out_sensor annotation (Placement(transformation(extent={{7,-7},{-7,7}}, origin={-3,-70})));
    WaterSteam.HeatExchangers.Reheater HP_heater(Q_cold_0=1500, Q_hot_0=50) annotation (Placement(transformation(extent={{-24,-78},{-56,-62}})));
  Sensors.WaterSteam.TemperatureSensor HP_heater_T_out_sensor annotation (Placement(transformation(extent={{-84,-77},{-98,-63}})));
  Sensors.WaterSteam.PressureSensor HP_heater_P_out_sensor annotation (Placement(transformation(extent={{-64,-77},{-78,-63}})));
  Sensors.WaterSteam.TemperatureSensor HP_heater_T_drains_sensor annotation (Placement(transformation(
        extent={{7,-7},{-7,7}},
        rotation=90,
        origin={-40,-98})));
    WaterSteam.Pipes.ControlValve HP_reheater_drains_control_valve annotation (Placement(transformation(extent={{6,-124},{16,-112}})));
  Sensors.Outline.OpeningSensor HP_reheater_drains_control_valve_opening_sensor annotation (Placement(transformation(extent={{6,-108},{16,-98}})));
  Sensors.WaterSteam.PressureSensor P_steam_sensor annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=90,
        origin={-170,10})));
  WaterSteam.Pipes.PressureCut pressureCut annotation (Placement(transformation(extent={{94,76},{114,96}})));
  Sensors.WaterSteam.TemperatureSensor CW_T_in_sensor annotation (Placement(transformation(extent={{318,60},{332,74}})));
  Sensors.WaterSteam.PressureSensor CW_P_in_sensor annotation (Placement(transformation(extent={{346,60},{360,74}})));
  Sensors.WaterSteam.TemperatureSensor CW_T_out_sensor annotation (Placement(transformation(extent={{423,53},{437,67}})));
  Sensors.WaterSteam.FlowSensor Q_purge_sensor annotation (Placement(transformation(extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-170,-132})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-170,-150})));
  WaterSteam.Pipes.LoopBreaker loopBreaker annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-142,-70})));
equation

  // SteamGenerator

    // Quantities definitions
    P_steam_sensor.P_barA = P_steam;
    Q_feedwater_sensor.Q = Q_feedwater;
    Q_purge_sensor.Q = Q_purge;

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
    HP_control_valve.Cvmax = HP_control_valve_Cvmax;
    HPT_1.Cst = HPT1_Cst;
    HPT_1.eta_is = turbines_eta_is;
    HPT_2.Cst = HPT2_Cst;
    HPT_2.eta_is = turbines_eta_is;

    // Parameter
    HPT_1.eta_nz = 1;
    HPT_1.area_nz = 1;
    HPT_2.eta_nz = 1;
    HPT_2.area_nz = 1;
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
    superheater_control_valve.Cvmax = superheater_control_valve_Cvmax;

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
    LPT1.eta_nz = 1;
    LPT1.area_nz = 1;
    LPT2.eta_nz = 1;
    LPT2.area_nz = 1;
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
    extraction_pump.rhmin = 0.2;

    // Calibrated parameters
    extraction_pump.hn = extraction_pump_hn;
    extraction_pump.rh = extraction_pump_rh;

  // LP Heater and drains

    // Quantities definitions
    LP_heater_P_out_sensor.P_barA = LP_heater_P_out;
    LP_heater_T_out_sensor.T_degC = LP_heater_T_out;
    LP_reheater_drains_control_valve_opening_sensor.Opening = LP_reheater_drains_control_valve_opening;

    // Parameters
    LP_heater.S_condensing = 100;
    LP_heater.Kfr_hot = 0;

    // Calibrated parameters
    LP_heater.Kth = LP_heater_Kth;
    LP_heater.Kfr_cold = LP_heater_Kfr_cold;
    LP_reheater_drains_control_valve.Cvmax = LP_heater_drains_control_valve_Cvmax;


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
    feedwater_pump.rhmin = 0.2;

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
    HP_heater.S_tot = 100;
    HP_heater.Kfr_hot = 0;
    HP_heater.level = 0.3;

      // Calibrated parameters
    HP_heater.Kth_subc = HP_heater_Kth_subc;
    HP_heater.Kth_cond = HP_heater_Kth_cond;
    HP_heater.Kfr_cold = HP_heater_Kfr_cold;
    HP_reheater_drains_control_valve.Cvmax = HP_heater_drains_control_valve_Cvmax;


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
  connect(P_cond_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{298,130},{392.5,130},{392.5,74}},
                                                                                                           color={28,108,200}));
  connect(superheater_T_out_sensor.C_out, LPT1.C_in) annotation (Line(points={{100,130},{151,130}},                             color={28,108,200}));
  connect(extraction_pump.C_power, LP_pump_Wm_source.C_out) annotation (Line(points={{372,-61.36},{372,-50.8}}, color={244,125,35}));
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
  connect(feedwater_pump.C_power, HP_pump_Wm_source.C_out) annotation (Line(points={{54,-61.36},{54,-50.8}}, color={244,125,35}));
  connect(feedwater_pump.C_out, HP_pump_T_out_sensor.C_in) annotation (Line(points={{46,-70},{30,-70}}, color={28,108,200}));
  connect(HP_pump_T_out_sensor.C_out, HP_pump_P_out_sensor.C_in) annotation (Line(points={{16,-70},{4,-70}},  color={28,108,200}));
  connect(feedwater_pump.C_in, deaerator_outlet_pipe.C_out) annotation (Line(points={{62,-70},{94,-70}}, color={28,108,200}));
  connect(HP_pump_P_out_sensor.C_out, HP_heater.C_cold_in) annotation (Line(points={{-10,-70},{-23.8,-70}}, color={28,108,200}));
  connect(HP_heater.C_cold_out, HP_heater_P_out_sensor.C_in) annotation (Line(points={{-56,-70},{-64,-70}}, color={28,108,200}));
  connect(HP_heater_P_out_sensor.C_out, HP_heater_T_out_sensor.C_in) annotation (Line(points={{-78,-70},{-84,-70}},                             color={28,108,200}));
  connect(HP_heater_T_drains_sensor.C_in, HP_heater.C_hot_out) annotation (Line(points={{-40,-91},{-40,-78}}, color={28,108,200}));
  connect(HP_extract_P_sensor.C_out, HP_heater.C_hot_in) annotation (Line(points={{-40,40},{-40,-62}}, color={28,108,200}));
  connect(superheater_drains_P_sensor.C_out, superheater_drains_pipe.C_in) annotation (Line(points={{112,112},{122,112},{122,42}}, color={28,108,200}));
  connect(superheater_drains_pipe.C_out, HP_heater.C_hot_in) annotation (Line(points={{122,22},{122,16},{-40,16},{-40,-62}}, color={28,108,200}));
  connect(HP_heater_T_out_sensor.C_out, Q_feedwater_sensor.C_in) annotation (Line(points={{-98,-70},{-104,-70}}, color={28,108,200}));
  connect(HP_reheater_drains_control_valve.Opening, HP_reheater_drains_control_valve_opening_sensor.Opening) annotation (Line(points={{11,-113.091},{11,-108.1}}, color={0,0,127}));
  connect(HP_heater_T_drains_sensor.C_out, HP_reheater_drains_control_valve.C_in) annotation (Line(points={{-40,-105},{-40,-121.818},{6,-121.818}}, color={28,108,200}));
  connect(LP_reheater_drains_control_valve.Opening, LP_reheater_drains_control_valve_opening_sensor.Opening) annotation (Line(points={{293,-111.091},{293,-106.1}}, color={0,0,127}));
  connect(LP_heater.C_hot_out, LP_reheater_drains_control_valve.C_in) annotation (Line(points={{268,-78},{268,-119.818},{288,-119.818}}, color={28,108,200}));
  connect(steam_generator.steam_outlet, P_steam_sensor.C_in) annotation (Line(points={{-170,-24},{-170,3}},  color={28,108,200}));
  connect(P_steam_sensor.C_out, HP_control_valve.C_in) annotation (Line(points={{-170,17},{-170,72},{-135,72}},           color={28,108,200}));
  connect(superheater.C_vent, pressureCut.C_in) annotation (Line(points={{88,104.2},{88,86},{94,86}}, color={28,108,200}));
  connect(pressureCut.C_out, superheater_drains_pipe.C_in) annotation (Line(points={{114,86},{122,86},{122,42}}, color={28,108,200}));
  connect(cold_source.C_out, CW_T_in_sensor.C_in) annotation (Line(points={{305,67.7778},{305,67},{318,67}},                   color={28,108,200}));
  connect(CW_T_in_sensor.C_out, CW_P_in_sensor.C_in) annotation (Line(points={{332,67},{346,67}}, color={28,108,200}));
  connect(CW_P_in_sensor.C_out, condenser.C_cold_in) annotation (Line(points={{360,67},{378,67},{378,65.4074},{377,65.4074}},    color={28,108,200}));
  connect(CW_T_out_sensor.C_out, cold_sink.C_in) annotation (Line(points={{437,60},{455,60}},                                 color={28,108,200}));
  connect(condenser.C_cold_out, CW_T_out_sensor.C_in) annotation (Line(points={{408,59.679},{408,60},{423,60}},                    color={28,108,200}));
  connect(LP_reheater_drains_control_valve.C_out, condenser.C_hot_in) annotation (Line(points={{298,-119.818},{400,-119.818},{400,-120},{500,-120},{500,100},{392.5,100},{392.5,74}},
                                                                                                                                                                                  color={28,108,200}));
  connect(HP_reheater_drains_control_valve.C_out, deaerator_outlet_pipe.C_in) annotation (Line(points={{16,-121.818},{78,-121.818},{78,-122},{142,-122},{142,-70},{114,-70}}, color={28,108,200}));
  connect(steam_generator.purge_outlet, Q_purge_sensor.C_in) annotation (Line(points={{-170,-115.233},{-170,-125}},             color={28,108,200}));
  connect(Q_feedwater_sensor.C_out, loopBreaker.C_in) annotation (Line(points={{-118,-70},{-132,-70}}, color={28,108,200}));
  connect(loopBreaker.C_out, steam_generator.feedwater_inlet) annotation (Line(points={{-152,-70},{-159,-70}}, color={28,108,200}));
  connect(Q_purge_sensor.C_out, sink.C_in) annotation (Line(points={{-170,-139},{-170,-145}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-160},{520,200}})), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-160},{520,200}})));
end MetroscopiaNPP_reverse;
