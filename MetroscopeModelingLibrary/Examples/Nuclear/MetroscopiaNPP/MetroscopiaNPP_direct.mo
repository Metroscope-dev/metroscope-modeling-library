within MetroscopeModelingLibrary.Examples.Nuclear.MetroscopiaNPP;
model MetroscopiaNPP_direct
  // Boundary Conditions
    // Steam generator
    input Real steam_generator_vapor_fraction(start = 0.99);
    input Real steam_generator_steam_P_out(start = 50, unit="bar", min=0, nominal=50) "barA";

    // Condenser cold side
    input Real cold_source_P_out(start = 3, unit="bar", min=0, nominal=5) "barA";
    input Real cold_source_T_out(start = 15, unit="degC", min=0, nominal=15) "degC";

  // Observables used for calibration
    // HP Control Valve
    output Units.Fraction HP_control_valve_opening; // HP_control_valve_Cvmax

    // HP turbines
    output Real HP_turbine_1_P_in; // HP_control_valve_Cv
    output Real HP_turbines_ext_P; // HP_turbine_1_Cst
    output Real HP_turbine_2_P_out; // HP_turbine_2_Cst

    // Superheater Control Valve
    output Units.Fraction superheater_control_valve_opening; // superheater_control_valve_Cvmax

    // Superheater
    output Real superheater_hot_P_in; // superheater_control_valve_Cv
    output Real superheater_drains_P_out; // superheater_Kfr_hot
    output Real superheated_steam_T_out; // superheater_Kth

    // LP turbines
    output Real LP_turbines_ext_P; // LP_turbine_1_Cst

    // Condenser
    output Real cold_source_Qv_out;
    output Real condenser_P_in; // LP_turbine_2_Cst
    output Real cold_sink_P_in; // condenser_Kfr_cold

    // Generator
    output Real generator_W_elec; // HP_LP_turbines_eta_is

    // LP pump
    output Real LP_pump_P_out; // LP_pump_a3
    output Real LP_pump_T_out; // LP_pump_b3

    // LP Reheater
    output Real LP_reheater_P_cold_out; // LP_reheater_Kfr_cold
    output Real LP_reheater_T_cold_out; // LP_reheater_Kth

    // LP reheater drains Control Valve
    output Units.Fraction LP_reheater_drains_control_valve_opening; // LP_control_valve_Cvmax
    output Real LP_reheater_drains_control_valve_P_out;

    // Flash tank : none

    // HP pump
    output Real HP_pump_P_out; // LP_pump_a3
    output Real HP_pump_T_out; // LP_pump_b3

    // HP Reheater
    output Real HP_reheater_P_cold_out; // LP_reheater_Kfr_cold
    output Real HP_reheater_T_cold_out; // LP_reheater_Kth
    output Real HP_reheater_T_drains; // LP_reheater_Kth

    // HP reheater drains Control Valve
    output Units.Fraction HP_reheater_drains_control_valve_opening; // HP_control_valve_Cvmax
    output Real HP_reheater_drains_control_valve_P_out;

  // Calibrated parameters
    // HP turbines inlet control valve
    parameter Units.Cv HP_control_valve_Cvmax = 1309815.4; // HP_control_valve_opening
    parameter Units.Cv HP_control_valve_Cv = 196472.3; // HP_turbine_1_P_in

    // HP Turbines
    parameter Units.Cst HP_turbine_1_Cst = 12381.829; // HP_turbines_ext_P
    parameter Units.Cst HP_turbine_2_Cst = 9671.686; // HP_turbine_2_P_out

    // Superheater inlet control valve
    parameter Units.Cv superheater_control_valve_Cvmax = 2936.443; // superheater_control_valve_opening
    parameter Units.Cv superheater_control_valve_Cv = 2642.7988; // superheater_hot_P_in

    // Superheater
    parameter Units.FrictionCoefficient superheater_Kfr_hot = 1052.634; // superheater_drains_P_out
    parameter Units.HeatExchangeCoefficient superheater_Kth = 17169.125; // superheated_steam_T_out

    // LP Turbines
    parameter Units.Cst LP_turbine_1_Cst = 6260.431; // LP_turbines_ext_P
    parameter Units.Cst LP_turbine_2_Cst = 608.22784; // LP_turbine_2_P_out = condenser_P_in

    parameter Units.Yield HP_LP_turbines_eta_is = 0.5373223; // generator_W_elec

    // Condenser
    parameter Units.HeatExchangeCoefficient condenser_Kth = 1245009.2;
    parameter Units.FrictionCoefficient condenser_Kfr_cold = 0.040031273; // cold_sink_P_in

    // LP pump
    parameter Real LP_pump_a3 = 162.92415; // LP_pump_P_out
    parameter Real LP_pump_b3 = 0.8685336; // LP_pump_T_out

    // LP Reheater
    parameter Units.HeatExchangeCoefficient LP_reheater_Kth = 14213.838;
    parameter Units.FrictionCoefficient LP_reheater_Kfr_cold = 88.654;

    // LP Reheater drains control valve
    parameter Units.Cv LP_reheater_drains_control_valve_Cvmax = 3569.3113; // LP_control_valve_opening
    parameter Units.Cv LP_reheater_drains_control_valve_Cv = 535.39667; //

    // Flash tank : none

    // LP pump
    parameter Real HP_pump_a3 = 757.10095; // HP_pump_P_out
    parameter Real HP_pump_b3 = 14.271142; // HP_pump_T_out

    // HP Reheater
    parameter Units.HeatExchangeCoefficient HP_reheater_Kth_cond = 146704.28;
    parameter Units.HeatExchangeCoefficient HP_reheater_Kth_subc = 403636.0;
    parameter Units.FrictionCoefficient HP_reheater_Kfr_cold = 43.455;

    // HP Reheater drains control valve
    parameter Units.Cv HP_reheater_drains_control_valve_Cvmax = 7775.4106; // HP_control_valve_opening
    parameter Units.Cv HP_reheater_drains_control_valve_Cv = 1166.3116; //

// Components
    // Steam Generator
    // HP Control Valve
    // HP Turbines
    // Steam Dryer
    // Superheater
    // LP Turbines
    // Generator
    // Condenser
    // Extraction Pump
    // LP reheater
    // Flash tank
    // Feedwater pump
    // HP Reheater
equation
// ----- Boundary Conditions ------
public
    WaterSteam.HeatExchangers.SteamGenerator steam_generator annotation (Placement(transformation(extent={{-196,-116},{-152,-24}})));
    WaterSteam.BoundaryConditions.Sink blow_down_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=180,origin={-190,-132})));
    Sensors.WaterSteam.WaterFlowSensor steam_generator_Q_in_sensor annotation (Placement(transformation(extent={{-100,-77},{-114,-63}})));
    WaterSteam.Pipes.ControlValve HP_control_valve(P_in_0=50e5, P_out_0=48.5e5) annotation (Placement(transformation(extent={{-136,70},{-126,82}})));
    Sensors.Other.OpeningSensor HP_control_valve_opening_sensor annotation (Placement(transformation(extent={{-136,86},{-126,96}})));
    Sensors.WaterSteam.WaterPressureSensor HP_turbine_1_P_in_sensor annotation (Placement(transformation(extent={{-106,66.1818},{-94,78.1818}})));
    WaterSteam.Machines.StodolaTurbine HP_turbine_1 annotation (Placement(transformation(extent={{-80,64.1818},{-62,80.1818}})));
    WaterSteam.Machines.StodolaTurbine HP_turbine_2 annotation (Placement(transformation(extent={{-8,64.1818},{10,80.1818}})));
    WaterSteam.Pipes.SteamExtractionSplitter HP_turbines_ext annotation (Placement(transformation(extent={{-46,62.1818},{-26,80.1818}})));
    Sensors.WaterSteam.WaterPressureSensor HP_turbines_ext_P_sensor annotation (Placement(transformation(extent={{-7,-7},{7,7}},rotation=270,origin={-36,53})));
    Sensors.WaterSteam.WaterPressureSensor HP_turbine_2_P_out_sensor annotation (Placement(transformation(extent={{20,66.1818},{32,78.1818}})));
    WaterSteam.Volumes.SteamDryer steam_dryer annotation (Placement(transformation(extent={{48,60.1818},{64,78.1818}})));
    WaterSteam.HeatExchangers.SuperHeater superheater annotation (Placement(transformation(extent={{56,104},{88,120}})));
    WaterSteam.BoundaryConditions.Sink superheater_vent_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=270,origin={88,86.182})));
    Sensors.WaterSteam.WaterPressureSensor superheater_drains_P_out_sensor annotation (Placement(transformation(extent={{100,106},{112,118}})));
    Sensors.WaterSteam.WaterTemperatureSensor superheated_steam_T_out_sensor annotation (Placement(transformation(extent={{88,124},{100,136}})));
    WaterSteam.Pipes.ControlValve superheater_control_valve annotation (Placement(transformation(extent={{-136,110},{-126,122}})));
    Sensors.Other.OpeningSensor superheater_control_valve_opening_sensor annotation (Placement(transformation(extent={{-136,132},{-126,142}})));
    Sensors.WaterSteam.WaterPressureSensor superheater_hot_P_in_sensor annotation (Placement(transformation(extent={{-106,106.182},{-94,118.182}})));
    WaterSteam.Pipes.PressureCut superheater_drains_pipe annotation (Placement(transformation(extent={{10,-10},{-10,10}},rotation=90,origin={122,32})));
    WaterSteam.Machines.StodolaTurbine LP_turbine_1 annotation (Placement(transformation(extent={{152,122.182},{170,138.182}})));
    WaterSteam.Machines.StodolaTurbine LP_turbine_2 annotation (Placement(transformation(extent={{224,122.182},{242,138.182}})));
    Sensors.WaterSteam.WaterPressureSensor LP_turbines_ext_P_sensor annotation (Placement(transformation(extent={{-7,-7},{7,7}},rotation=270,origin={196,111})));
    WaterSteam.Pipes.SteamExtractionSplitter LP_turbines_ext annotation (Placement(transformation(extent={{186,120},{206,138}})));
    Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{362,158},{382,178}})));
    Power.Machines.Generator generator annotation (Placement(transformation(extent={{308,156},{348,180}})));
    Sensors.Power.PowerSensor generator_W_elec_sensor annotation (Placement(transformation(extent={{348,162},{360,174}})));
    Sensors.WaterSteam.WaterPressureSensor condenser_P_in_sensor annotation (Placement(transformation(extent={{286,124},{298,136}})));
    WaterSteam.HeatExchangers.Condenser condenser(Psat_0=69.8e2) annotation (Placement(transformation(extent={{379,54},{405,76}})));
    WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=0,origin={440,63.7778})));
    Sensors.WaterSteam.WaterPressureSensor cold_sink_P_in_sensor annotation (Placement(transformation(extent={{418,57.7778},{430,69.7778}})));
    WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{328,59.7778},{348,79.7778}})));
    Sensors.WaterSteam.WaterFlowSensor cold_source_Qv_out_sensor annotation (Placement(transformation(extent={{352,62.7778},{366,76.7778}})));
    WaterSteam.Machines.Pump LP_pump(P_in_0=69.8e2) annotation (Placement(transformation(extent={{380,-78},{364,-62}})));
    Power.BoundaryConditions.Source LP_pump_Wm_source annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=270,origin={372,-46})));
    Sensors.WaterSteam.WaterTemperatureSensor LP_pump_T_out_sensor annotation (Placement(transformation(extent={{350,-77},{336,-63}})));
    Sensors.WaterSteam.WaterPressureSensor LP_pump_P_out_sensor annotation (Placement(transformation(extent={{7,-7},{-7,7}},     origin={315,-70})));
    WaterSteam.HeatExchangers.DryReheater LP_reheater annotation (Placement(transformation(extent={{284,-78},{252,-62}})));
    Sensors.WaterSteam.WaterTemperatureSensor LP_reheater_T_cold_out_sensor annotation (Placement(transformation(extent={{220,-77},{206,-63}})));
    Sensors.WaterSteam.WaterPressureSensor LP_reheater_P_cold_out_sensor annotation (Placement(transformation(extent={{242,-77},{228,-63}})));
    WaterSteam.Pipes.ControlValve LP_reheater_drains_control_valve annotation (Placement(transformation(extent={{288,-122},{298,-110}})));
    Sensors.Other.OpeningSensor LP_reheater_drains_control_valve_opening_sensor annotation (Placement(transformation(extent={{288,-106},{298,-96}})));
    Sensors.WaterSteam.WaterPressureSensor LP_reheater_drains_control_valve_P_out_sensor annotation (Placement(transformation(extent={{306,-125.818},{318,-113.818}})));
    WaterSteam.Pipes.PressureCut LP_reheater_drains_pipe annotation (Placement(transformation(extent={{346,-130},{366,-110}})));
    WaterSteam.Pipes.Pipe flash_tank_inlet_pipe(delta_z_0=5) annotation (Placement(transformation(extent={{200,-80},{180,-60}})));
    WaterSteam.Pipes.PressureCut steam_dryer_liq_out_pipe annotation (Placement(transformation(extent={{10,-10},{-10,10}},rotation=90,origin={142,-8})));
    WaterSteam.Pipes.Pipe flash_tank_outlet_pipe(delta_z_0=-5) annotation (Placement(transformation(extent={{100,-80},{80,-60}})));
    WaterSteam.Machines.Pump HP_pump(P_in_0=6e5, P_out_0=59e5) annotation (Placement(transformation(extent={{66,-78},{50,-62}})));
    Power.BoundaryConditions.Source HP_pump_Wm_source annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=270,origin={58,-46})));
    Sensors.WaterSteam.WaterTemperatureSensor HP_pump_T_out_sensor annotation (Placement(transformation(extent={{36,-77},{22,-63}})));
    Sensors.WaterSteam.WaterPressureSensor HP_pump_P_out_sensor annotation (Placement(transformation(extent={{7,-7},{-7,7}}, origin={1,-70})));
    WaterSteam.HeatExchangers.Reheater HP_reheater(Q_cold_0=1500, Q_hot_0=50) annotation (Placement(transformation(extent={{-20,-78},{-52,-62}})));
    Sensors.WaterSteam.WaterTemperatureSensor HP_reheater_T_cold_out_sensor annotation (Placement(transformation(extent={{-80,-77},{-94,-63}})));
    Sensors.WaterSteam.WaterPressureSensor HP_reheater_P_cold_out_sensor annotation (Placement(transformation(extent={{-60,-77},{-74,-63}})));
    Sensors.WaterSteam.WaterTemperatureSensor HP_reheater_T_drains_sensor annotation (Placement(transformation(extent={{7,-7},{-7,7}},rotation=90,origin={-36,-98})));
    WaterSteam.Pipes.ControlValve HP_reheater_drains_control_valve annotation (Placement(transformation(extent={{6,-124},{16,-112}})));
    Sensors.Other.OpeningSensor HP_reheater_drains_control_valve_opening_sensor annotation (Placement(transformation(extent={{6,-108},{16,-98}})));
    Sensors.WaterSteam.WaterPressureSensor HP_reheater_drains_control_valve_P_out_sensor annotation (Placement(transformation(extent={{24,-127.818},{36,-115.818}})));
    WaterSteam.Pipes.PressureCut HP_reheater_drains_pipe annotation (Placement(transformation(extent={{64,-132},{84,-112}})));
equation
  steam_generator.vapor_fraction = steam_generator_vapor_fraction;
  steam_generator.steam_pressure = steam_generator_steam_P_out * 1e5;

  cold_source.P_out = cold_source_P_out * 1e5;
  cold_source.T_out = cold_source_T_out + 273.15;

// ----- Components ------
  // SteamGenerator
    // Purge fixed parameters
    steam_generator.P_purge = 50e5;
    steam_generator_Q_in_sensor.Q = 1500;

  // HP systems
    // HP turbines inlet control valve
      // Observables used for calibration (inlet)
      HP_turbine_1_P_in_sensor.P_barA = HP_turbine_1_P_in;
      HP_control_valve_opening_sensor.Opening = HP_control_valve_opening;

      // Calibrated parameters
      HP_control_valve.Cvmax = HP_control_valve_Cvmax;
      HP_control_valve.Cv = HP_control_valve_Cv;

    // HP Turbine 1
      // Calibrated parameters
      HP_turbine_1.Cst = HP_turbine_1_Cst;
      HP_turbine_1.eta_is = HP_LP_turbines_eta_is;

      // Hypothesis : no nozzle
      HP_turbine_1.eta_nz = 1;
      HP_turbine_1.area_nz = 1;

      // HP extraction
        // Fixed parameter
        HP_turbines_ext.alpha = 1;

        // Observable used for calibration
        HP_turbines_ext_P_sensor.P_barA = HP_turbines_ext_P;

    // HP Turbine 2
      // Calibrated parameters
      HP_turbine_2.Cst = HP_turbine_2_Cst;
      HP_turbine_2.eta_is = HP_LP_turbines_eta_is;

      // Hypothesis : no nozzle
      HP_turbine_2.eta_nz = 1;
      HP_turbine_2.area_nz = 1;

      // Observable used for calibration
      HP_turbine_2_P_out_sensor.P_barA = HP_turbine_2_P_out;

  // Steam dryer
  steam_dryer.x_steam_out = 0.99;

  // Superheater
    // Superheater control valve
      // Observables used for calibration (inlet)
      superheater_hot_P_in_sensor.P_barA = superheater_hot_P_in;
      superheater_control_valve_opening_sensor.Opening = superheater_control_valve_opening;

      // Calibrated parameters
      superheater_control_valve.Cvmax = superheater_control_valve_Cvmax;
      superheater_control_valve.Cv = superheater_control_valve_Cv;

    // Fixed parameters
    superheater.S = 100;
    superheater.Kfr_cold = 0;
    superheater.Q_vent = 1;

    // Observables used for calibration
    superheater_drains_P_out_sensor.P_barA = superheater_drains_P_out;
    superheated_steam_T_out_sensor.T_degC = superheated_steam_T_out;

    // Calibrated parameters
    superheater.Kfr_hot = superheater_Kfr_hot;
    superheater.Kth = superheater_Kth;

  // LP systems
    // LP Turbine 1
      // Calibrated parameters
      LP_turbine_1.Cst = LP_turbine_1_Cst;
      LP_turbine_1.eta_is = HP_LP_turbines_eta_is;

      // Hypothesis : no nozzle
      LP_turbine_1.eta_nz = 1;
      LP_turbine_1.area_nz = 1;

      // LP Extraction
        // Fixed parameter
        LP_turbines_ext.alpha = 1;

        // Observable used for calibration
        LP_turbines_ext_P_sensor.P_barA = LP_turbines_ext_P;

    // LP Turbine 2
      // Calibrated parameters
      LP_turbine_2.Cst = LP_turbine_2_Cst;
      LP_turbine_2.eta_is = HP_LP_turbines_eta_is;

      // Hypothesis : no nozzle
      LP_turbine_2.eta_nz = 1;
      LP_turbine_2.area_nz = 1;

  // Generator
    // Observable used for calibration
    generator_W_elec_sensor.W_MW = generator_W_elec; // Calibrates STs_eta_is

    // Fixed parameter
    generator.eta = 0.99;

  // Condenser
    // Fixed parameters
    condenser.S = 100;
    condenser.water_height = 1;
    condenser.C_incond = 0;
    condenser.P_offset = 0;

    // Observable used for calibration
    cold_source_Qv_out_sensor.Qv_out = cold_source_Qv_out;
    condenser_P_in_sensor.P_mbar = condenser_P_in;
    cold_sink_P_in_sensor.P_barA = cold_sink_P_in;

    // Calibrated Parameters
    condenser.Kth = condenser_Kth;
    condenser.Kfr_cold = condenser_Kfr_cold;

  // LP pump
    // Fixed parameters
    LP_pump.VRotn = 4000;
    LP_pump.VRot = 3950;
    LP_pump.rm = 0.85;
    LP_pump.a1 = -78.0237;
    LP_pump.a2 = 0;
    LP_pump.b1 = 0;
    LP_pump.b2 = 0;
    LP_pump.rhmin = 0.2;

    // Calibrated parameters
    LP_pump.a3 = LP_pump_a3;
    LP_pump.b3 = LP_pump_b3;

    // Inputs for calibration
    LP_pump_T_out_sensor.T_degC = LP_pump_T_out;
    LP_pump_P_out_sensor.P_barA = LP_pump_P_out;

  // LP Reheater
    // Component parameters
    LP_reheater.S_condensing = 100;
    LP_reheater.Kfr_hot = 0;

    // Observables for calibration
    LP_reheater_P_cold_out_sensor.P_barA = LP_reheater_P_cold_out;
    LP_reheater_T_cold_out_sensor.T_degC = LP_reheater_T_cold_out;

    // Calibrated parameters
    LP_reheater.Kth = LP_reheater_Kth;
    LP_reheater.Kfr_cold = LP_reheater_Kfr_cold;

    // Drains
      // Observables used for calibration
      LP_reheater_drains_control_valve_P_out_sensor.P_barA = LP_reheater_drains_control_valve_P_out;
      LP_reheater_drains_control_valve_opening_sensor.Opening = LP_reheater_drains_control_valve_opening;

      // Calibrated parameters
      LP_reheater_drains_control_valve.Cvmax = LP_reheater_drains_control_valve_Cvmax;
      LP_reheater_drains_control_valve.Cv = LP_reheater_drains_control_valve_Cv;

  // Flash tank
    // Inlet Pipe
    flash_tank_inlet_pipe.Kfr = 0;
    flash_tank_inlet_pipe.delta_z = 5;

    flash_tank_outlet_pipe.Kfr = 0;
    flash_tank_outlet_pipe.delta_z = -5;

  // HP pump
    // Fixed parameters
    HP_pump.VRotn = 4000;
    HP_pump.VRot = 3950;
    HP_pump.rm = 0.85;
    HP_pump.a1 = -78.0237;
    HP_pump.a2 = 0;
    HP_pump.b1 = 0;
    HP_pump.b2 = 0;
    HP_pump.rhmin = 0.2;

    // Calibrated parameters
    HP_pump.a3 = HP_pump_a3;
    HP_pump.b3 = HP_pump_b3;

    // Inputs for calibration
    HP_pump_T_out_sensor.T_degC = HP_pump_T_out;
    HP_pump_P_out_sensor.P_barA = HP_pump_P_out;

  // HP Reheater
    // Component parameters
    HP_reheater.S_tot = 100;
    HP_reheater.Kfr_hot = 0;
    HP_reheater.level = 0.3;

    // Observables for calibration
    HP_reheater_P_cold_out_sensor.P_barA = HP_reheater_P_cold_out;
    HP_reheater_T_cold_out_sensor.T_degC = HP_reheater_T_cold_out;
    HP_reheater_T_drains_sensor.T_degC = HP_reheater_T_drains;

    // Calibrated parameters
    HP_reheater.Kth_subc = HP_reheater_Kth_subc;
    HP_reheater.Kth_cond = HP_reheater_Kth_cond;
    HP_reheater.Kfr_cold = HP_reheater_Kfr_cold;

    // Drains
      // Observables used for calibration
      HP_reheater_drains_control_valve_P_out_sensor.P_barA = HP_reheater_drains_control_valve_P_out;
      HP_reheater_drains_control_valve_opening_sensor.Opening = HP_reheater_drains_control_valve_opening;

      // Calibrated parameters
      HP_reheater_drains_control_valve.Cvmax = HP_reheater_drains_control_valve_Cvmax;
      HP_reheater_drains_control_valve.Cv = HP_reheater_drains_control_valve_Cv;

  connect(steam_generator.purge_outlet, blow_down_sink.C_in) annotation (Line(points={{-174,-115.233},{-174,-132},{-185,-132}},
                                                                                                                   color={28,108,200}));
  connect(steam_generator.steam_outlet, HP_control_valve.C_in) annotation (Line(points={{-174,-24},{-174,72.1818},{-136,72.1818}},color={28,108,200}));
  connect(HP_control_valve.C_out, HP_turbine_1_P_in_sensor.C_in) annotation (Line(points={{-126,72.1818},{-116,72.1818},{-116,72.1818},{-106,72.1818}},
                                                                                                                                                      color={28,108,200}));
  connect(HP_control_valve.Opening, HP_control_valve_opening_sensor.Opening) annotation (Line(points={{-131,80.9091},{-131,85.9}}, color={0,0,127}));
  connect(HP_turbine_1.C_out, HP_turbines_ext.C_in) annotation (Line(points={{-62,72.1818},{-46.6,72.1818}}, color={28,108,200}));
  connect(HP_turbines_ext.C_main_out, HP_turbine_2.C_in) annotation (Line(points={{-25.4,72.1818},{-8,72.1818}}, color={28,108,200}));
  connect(HP_turbines_ext.C_ext_out, HP_turbines_ext_P_sensor.C_in) annotation (Line(points={{-36,65.3818},{-36,60}}, color={28,108,200}));
  connect(powerSink.C_in,generator_W_elec_sensor. C_out) annotation (Line(points={{367,168},{359.88,168}},
                                                                                              color={244,125,35}));
  connect(generator_W_elec_sensor.C_in,generator. C_out) annotation (Line(points={{348,168},{342,168}},
                                                                                           color={244,125,35}));
  connect(HP_turbine_2.C_W_out,generator. C_in) annotation (Line(points={{10,78.9018},{18,78.9018},{18,168},{315.6,168}},
                                                                                                                   color={244,125,35}));
  connect(HP_turbine_1.C_W_out,generator. C_in) annotation (Line(points={{-62,78.9018},{-54,78.9018},{-54,168},{315.6,168}},
                                                                                                           color={244,125,35}));
  connect(HP_turbine_1.C_in, HP_turbine_1_P_in_sensor.C_out) annotation (Line(points={{-80,72.1818},{-94,72.1818}},                 color={28,108,200}));
  connect(HP_turbine_2_P_out_sensor.C_in, HP_turbine_2.C_out) annotation (Line(points={{20,72.1818},{10,72.1818}},
                                                                                                         color={28,108,200}));
  connect(steam_dryer.C_in, HP_turbine_2_P_out_sensor.C_out) annotation (Line(points={{48,71.6363},{40,71.6363},{40,72.1818},{32,72.1818}}, color={28,108,200}));
  connect(superheater.C_cold_in, steam_dryer.C_hot_steam) annotation (Line(points={{72,104},{72,71.6363},{64,71.6363}}, color={28,108,200}));
  connect(superheater.C_hot_out, superheater_drains_P_out_sensor.C_in) annotation (Line(points={{88,112},{100,112}}, color={28,108,200}));
  connect(superheated_steam_T_out_sensor.C_in,superheater. C_cold_out) annotation (Line(points={{88,130},{71.8,130.182},{71.8,120}}, color={28,108,200}));
  connect(superheater_control_valve.C_in, HP_control_valve.C_in) annotation (Line(points={{-136,112.182},{-174,112.182},{-174,72.1818},{-136,72.1818}}, color={28,108,200}));
  connect(superheater_control_valve.Opening, superheater_control_valve_opening_sensor.Opening) annotation (Line(points={{-131,120.909},{-131,131.9}}, color={0,0,127}));
  connect(superheater_hot_P_in_sensor.C_in, superheater_control_valve.C_out) annotation (Line(points={{-106,112.182},{-116,112.182},{-116,112.182},{-126,112.182}}, color={28,108,200}));
  connect(LP_turbine_1.C_W_out,generator. C_in) annotation (Line(points={{170,136.902},{188,136.902},{188,168},{315.6,168}},
                                                                                                           color={244,125,35}));
  connect(LP_turbine_2.C_W_out, generator.C_in) annotation (Line(points={{242,136.902},{262,136.902},{262,168},{315.6,168}}, color={244,125,35}));
  connect(LP_turbine_2.C_out, condenser_P_in_sensor.C_in) annotation (Line(points={{242,130.182},{264,130.182},{264,130},{286,130}}, color={28,108,200}));
  connect(condenser_P_in_sensor.C_out, condenser.C_hot_in) annotation (Line(points={{298,130},{392,130},{392,76}}, color={28,108,200}));
  connect(cold_sink_P_in_sensor.C_out, cold_sink.C_in) annotation (Line(points={{430,63.7778},{435,63.7778}},                     color={28,108,200}));
  connect(condenser.C_cold_out, cold_sink_P_in_sensor.C_in) annotation (Line(points={{405,63.0444},{411.5,63.0444},{411.5,63.7778},{418,63.7778}}, color={28,108,200}));
  connect(superheater.C_vent, superheater_vent_sink.C_in) annotation (Line(points={{88,104.2},{88,91.182}}, color={28,108,200}));
  connect(superheated_steam_T_out_sensor.C_out, LP_turbine_1.C_in) annotation (Line(points={{100,130},{126,130},{126,130.182},{152,130.182}}, color={28,108,200}));
  connect(condenser.C_cold_in, cold_source_Qv_out_sensor.C_out) annotation (Line(points={{378.48,69.8889},{378.48,69.7778},{366,69.7778}}, color={28,108,200}));
  connect(cold_source_Qv_out_sensor.C_in, cold_source.C_out) annotation (Line(points={{352,69.7778},{343,69.7778}}, color={28,108,200}));
  connect(LP_pump.C_power,LP_pump_Wm_source. C_out) annotation (Line(points={{372,-61.36},{372,-50.8}}, color={244,125,35}));
  connect(LP_pump.C_out,LP_pump_T_out_sensor. C_in) annotation (Line(points={{364,-70},{350,-70}}, color={28,108,200}));
  connect(LP_pump_T_out_sensor.C_out,LP_pump_P_out_sensor. C_in) annotation (Line(points={{336,-70},{322,-70}}, color={28,108,200}));
  connect(condenser.C_hot_out, LP_pump.C_in) annotation (Line(points={{392,53.5111},{392,-70},{380,-70}}, color={28,108,200}));
  connect(LP_reheater.C_cold_out,LP_reheater_P_cold_out_sensor. C_in) annotation (Line(points={{252,-70},{242,-70}},
                                                                                                    color={28,108,200}));
  connect(LP_reheater_P_cold_out_sensor.C_out,LP_reheater_T_cold_out_sensor. C_in) annotation (Line(points={{228,-70},{224,-70},{224,-70.5},{220,-70.5},{220,-70}},
                                                                                                      color={28,108,200}));
  connect(LP_pump_P_out_sensor.C_out,LP_reheater. C_cold_in) annotation (Line(points={{308,-70},{284.2,-70}},                     color={28,108,200}));
  connect(LP_turbines_ext_P_sensor.C_out,LP_reheater. C_hot_in) annotation (Line(points={{196,104},{196,44},{268,44},{268,-62}}, color={28,108,200}));
  connect(LP_turbines_ext_P_sensor.C_in, LP_turbines_ext.C_ext_out) annotation (Line(points={{196,118},{196,123.2}}, color={28,108,200}));
  connect(LP_turbine_1.C_out, LP_turbines_ext.C_in) annotation (Line(points={{170,130.182},{177.7,130.182},{177.7,130},{185.4,130}}, color={28,108,200}));
  connect(LP_turbine_2.C_in, LP_turbines_ext.C_main_out) annotation (Line(points={{224,130.182},{215.3,130.182},{215.3,130},{206.6,130}}, color={28,108,200}));
  connect(flash_tank_inlet_pipe.C_in, LP_reheater_T_cold_out_sensor.C_out) annotation (Line(points={{200,-70},{206,-70}}, color={28,108,200}));
  connect(steam_dryer.C_hot_liquid, steam_dryer_liq_out_pipe.C_in) annotation (Line(points={{64,65.0909},{64,64},{142,64},{142,2}},  color={28,108,200}));
  connect(flash_tank_inlet_pipe.C_out, flash_tank_outlet_pipe.C_in) annotation (Line(points={{180,-70},{100,-70}}, color={28,108,200}));
  connect(steam_dryer_liq_out_pipe.C_out, flash_tank_outlet_pipe.C_in) annotation (Line(points={{142,-18},{142,-70},{100,-70}},
                                                                                                                              color={28,108,200}));
  connect(HP_pump.C_power, HP_pump_Wm_source.C_out) annotation (Line(points={{58,-61.36},{58,-50.8}}, color={244,125,35}));
  connect(HP_pump.C_out, HP_pump_T_out_sensor.C_in) annotation (Line(points={{50,-70},{36,-70}}, color={28,108,200}));
  connect(HP_pump_T_out_sensor.C_out, HP_pump_P_out_sensor.C_in) annotation (Line(points={{22,-70},{8,-70}},  color={28,108,200}));
  connect(HP_pump.C_in, flash_tank_outlet_pipe.C_out) annotation (Line(points={{66,-70},{80,-70}}, color={28,108,200}));
  connect(HP_pump_P_out_sensor.C_out, HP_reheater.C_cold_in) annotation (Line(points={{-6,-70},{-19.8,-70}}, color={28,108,200}));
  connect(HP_reheater.C_cold_out, HP_reheater_P_cold_out_sensor.C_in) annotation (Line(points={{-52,-70},{-60,-70}}, color={28,108,200}));
  connect(HP_reheater_P_cold_out_sensor.C_out, HP_reheater_T_cold_out_sensor.C_in) annotation (Line(points={{-74,-70},{-80,-70}},                             color={28,108,200}));
  connect(HP_reheater_T_drains_sensor.C_in, HP_reheater.C_hot_out) annotation (Line(points={{-36,-91},{-36,-78}}, color={28,108,200}));
  connect(HP_turbines_ext_P_sensor.C_out, HP_reheater.C_hot_in) annotation (Line(points={{-36,46},{-36,-62}}, color={28,108,200}));
  connect(superheater_drains_P_out_sensor.C_out, superheater_drains_pipe.C_in) annotation (Line(points={{112,112},{122,112},{122,42}}, color={28,108,200}));
  connect(superheater_drains_pipe.C_out, HP_reheater.C_hot_in) annotation (Line(points={{122,22},{122,0},{-36,0},{-36,-62}}, color={28,108,200}));
  connect(HP_reheater_T_cold_out_sensor.C_out, steam_generator_Q_in_sensor.C_in) annotation (Line(points={{-94,-70},{-100,-70}},                    color={28,108,200}));
  connect(steam_generator.feedwater_inlet, steam_generator_Q_in_sensor.C_out) annotation (Line(points={{-163,-70},{-114,-70}},                           color={28,108,200}));
  connect(HP_reheater_drains_control_valve.Opening, HP_reheater_drains_control_valve_opening_sensor.Opening) annotation (Line(points={{11,-113.091},{11,-108.1}}, color={0,0,127}));
  connect(HP_reheater_T_drains_sensor.C_out, HP_reheater_drains_control_valve.C_in) annotation (Line(points={{-36,-105},{-36,-121.818},{6,-121.818}}, color={28,108,200}));
  connect(HP_reheater_drains_control_valve.C_out, HP_reheater_drains_control_valve_P_out_sensor.C_in) annotation (Line(points={{16,-121.818},{20,-121.818},{20,-121.818},{24,-121.818}}, color={28,108,200}));
  connect(HP_reheater_drains_control_valve_P_out_sensor.C_out, HP_reheater_drains_pipe.C_in) annotation (Line(points={{36,-121.818},{36,-122},{64,-122}}, color={28,108,200}));
  connect(HP_reheater_drains_pipe.C_out, flash_tank_outlet_pipe.C_in) annotation (Line(points={{84,-122},{142,-122},{142,-70},{100,-70}}, color={28,108,200}));
  connect(LP_reheater_drains_control_valve.Opening, LP_reheater_drains_control_valve_opening_sensor.Opening) annotation (Line(points={{293,-111.091},{293,-106.1}}, color={0,0,127}));
  connect(LP_reheater_drains_control_valve.C_out, LP_reheater_drains_control_valve_P_out_sensor.C_in) annotation (Line(points={{298,-119.818},{302,-119.818},{302,-119.818},{306,-119.818}}, color={28,108,200}));
  connect(LP_reheater_drains_control_valve_P_out_sensor.C_out, LP_reheater_drains_pipe.C_in) annotation (Line(points={{318,-119.818},{318,-120},{346,-120}}, color={28,108,200}));
  connect(LP_reheater.C_hot_out, LP_reheater_drains_control_valve.C_in) annotation (Line(points={{268,-78},{268,-119.818},{288,-119.818}}, color={28,108,200}));
  connect(LP_reheater_drains_pipe.C_out, condenser.C_hot_in) annotation (Line(points={{366,-120},{460,-120},{460,100},{392,100},{392,76}}, color={28,108,200}));
  connect(superheater.C_hot_in, superheater_hot_P_in_sensor.C_out) annotation (Line(points={{56,112.2},{-35,112.2},{-35,112.182},{-94,112.182}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{460,200}})), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{460,200}}), graphics={Rectangle(
          extent={{120,-54},{164,-82}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid), Rectangle(
          extent={{120,-40},{164,-54}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid)}));
end MetroscopiaNPP_direct;
