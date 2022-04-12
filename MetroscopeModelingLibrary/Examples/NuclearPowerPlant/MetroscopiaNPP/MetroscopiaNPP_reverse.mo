within MetroscopeModelingLibrary.Examples.NuclearPowerPlant.MetroscopiaNPP;
model MetroscopiaNPP_reverse
  // Boundary Conditions
    // Steam generator
    input Real steam_generator_vapor_fraction(start = 0.99);
    input Real steam_generator_steam_P_out(start = 50) "barA";
    input Real steam_generator_thermal_power(start = 1880) "MWth";

    // Boundary conditions
    input Real cold_source_P_out(start = 3) "barA";
    input Real cold_source_T_out(start = 15) "°C";
    input Real cold_source_Qv_out(start = 50) "m3/s";

  // Observables used for calibration
    // Steam Generator
    input Real steam_generator_P_in(start=57.782) "barA";

    // HP Control Valve
    input Real HP_control_valve_opening(start=0.15);

    // HP turbines
    input Real HP_turbine_1_P_in(start=48.5305) "barA";
    input Real HP_turbine_2_P_out(start=19.3986) "barA";
    input Real HP_turbines_ext_P(start=31.0032) "barA";

    // Superheater Control Valve
    input Real superheater_control_valve_opening(start=0.9);

    // Superheater
    input Real superheater_hot_P_in(start=41) "barA";
    input Real superheater_drains_P_out(start=40) "barA";
    input Real superheated_steam_temperature(start=228) "°C";

    // LP turbines
    input Real LP_turbine_1_P_in(start=19) "barA";
    input Real LP_turbines_ext_P(start=5) "barA";

    // Condenser
    input Real condenser_P_in(start=69.8) "mbar";

    // Generator
    input Real ActivePower(start=568.78) "MW";

  // Calibrated parameters
    // HP turbines inlet control valve
    output Units.Cv HP_control_valve_Cvmax;
    output Units.Cv HP_control_valve_Cv;

    // HP Turbines
    output Units.Cst HP_turbine_1_Cst;
    output Units.Cst HP_turbine_2_Cst;

    // Superheater inlet control valve
    output Units.Cv superheater_control_valve_Cvmax;
    output Units.Cv superheater_control_valve_Cv;


    // LP Turbines
    output Units.Cst LP_turbine_1_Cst;
    output Units.Cst LP_turbine_2_Cst;

    output Units.Yield HP_LP_turbines_eta_is;

  // Components
    // Steam Generator
    WaterSteam.HeatExchangers.SteamGenerator steam_generator annotation (Placement(transformation(extent={{-196,-116},{-152,-24}})));
    Sensors.WaterSteam.WaterPressureSensor steam_generator_P_in_sensor annotation (Placement(transformation(extent={{-112,-76},{-124,-64}})));
    WaterSteam.BoundaryConditions.Sink blow_down_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=180,origin={-190,-132})));

    // HP
      // HP Control Valve
      WaterSteam.Pipes.ControlValve HP_control_valve annotation (Placement(transformation(extent={{-136,70},{-126,82}})));
      Sensors.Other.OpeningSensor HP_control_valve_opening_sensor annotation (Placement(transformation(extent={{-136,86},{-126,96}})));

      // HP Turbines
      Sensors.WaterSteam.WaterPressureSensor HP_turbine_1_P_in_sensor annotation (Placement(transformation(extent={{-106,66.1818},{-94,78.1818}})));
      WaterSteam.Machines.StodolaTurbine HP_turbine_1 annotation (Placement(transformation(extent={{-80,64.1818},{-62,80.1818}})));
      WaterSteam.Machines.StodolaTurbine HP_turbine_2 annotation (Placement(transformation(extent={{-8,64.1818},{10,80.1818}})));
      WaterSteam.Pipes.SteamExtractionSplitter HP_turbines_ext annotation (Placement(transformation(extent={{-46,62.1818},{-26,80.1818}})));
      Sensors.WaterSteam.WaterPressureSensor HP_turbines_ext_P_sensor annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-36,47})));


    // Generator
    Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{362,158},{382,178}})));
    Power.Machines.Generator generator annotation (Placement(transformation(extent={{308,156},{348,180}})));
    Sensors.Power.PowerSensor W_tot_sensor annotation (Placement(transformation(extent={{348,162},{360,174}})));

    // Temporary components
    WaterSteam.BoundaryConditions.Source temp_feedwater_source annotation (Placement(transformation(extent={{172,-80},{152,-60}})));
    WaterSteam.BoundaryConditions.Sink temp_main_steam_sink annotation (Placement(transformation(extent={{456,120.182},{476,140.182}})));


  // Unclassified components
  WaterSteam.BoundaryConditions.Sink temp_HP_turbines_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-36,22})));


  Sensors.WaterSteam.WaterPressureSensor HP_turbine_2_P_out_sensor annotation (Placement(transformation(extent={{20,66.1818},{32,78.1818}})));
  WaterSteam.Volumes.SteamDryer steam_dryer
    annotation (Placement(transformation(extent={{48,60.1818},{64,78.1818}})));
    WaterSteam.BoundaryConditions.Sink temp_steam_dryer_condensates_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={64,22.1818})));
  WaterSteam.HeatExchangers.SuperHeater superheater annotation (Placement(transformation(extent={{56,104},{88,120}})));
    WaterSteam.BoundaryConditions.Sink temp_superheater_drains_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={126,22.182})));
    WaterSteam.BoundaryConditions.Sink superheater_vent_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={88,90.182})));
  Sensors.WaterSteam.WaterPressureSensor superheater_drains_P_out_sensor annotation (Placement(transformation(extent={{100,106},{112,118}})));
  Sensors.WaterSteam.WaterTemperatureSensor superheated_steam_T_sensor annotation (Placement(transformation(extent={{88,124},{100,136}})));
  WaterSteam.Pipes.ControlValve superheater_control_valve annotation (Placement(transformation(extent={{-136,110},{-126,122}})));
      Sensors.Other.OpeningSensor superheater_control_valve_opening_sensor annotation (Placement(transformation(extent={{-136,132},{-126,142}})));
      Sensors.WaterSteam.WaterPressureSensor superheater_hot_P_in_sensor annotation (Placement(transformation(extent={{-106,106.182},{-94,118.182}})));
      WaterSteam.Machines.StodolaTurbine LP_turbine_1 annotation (Placement(transformation(extent={{152,122.182},{170,138.182}})));
      WaterSteam.Machines.StodolaTurbine LP_turbine_2 annotation (Placement(transformation(extent={{224,122.182},{242,138.182}})));
      WaterSteam.Pipes.SteamExtractionSplitter LP_turbines_ext annotation (Placement(transformation(extent={{186,120.182},{206,138.182}})));
      Sensors.WaterSteam.WaterPressureSensor LP_turbines_ext_P_sensor annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={196,105})));
  WaterSteam.BoundaryConditions.Sink temp_LP_turbines_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={196,80})));
  Sensors.WaterSteam.WaterPressureSensor condenser_P_in_sensor annotation (Placement(transformation(extent={{286,124},{298,136}})));
      Sensors.WaterSteam.WaterPressureSensor LP_turbine_1_P_in_sensor annotation (Placement(transformation(extent={{106,124.182},{118,136.182}})));
equation
  // ----- Boundary Conditions ------
  steam_generator.vapor_fraction = steam_generator_vapor_fraction;
  steam_generator.thermal_power = steam_generator_thermal_power * 1e6;
  steam_generator.steam_pressure = steam_generator_steam_P_out * 1e5;

  // ----- Components ------
  // SteamGenerator
    // Observable used for calibration
    steam_generator_P_in_sensor.P_barA = steam_generator_P_in;

    // Purge fixed parameters
    steam_generator.Q_purge = 1e-2;
    steam_generator.P_purge = 50e5;

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

      // Extraction 1
      HP_turbines_ext_P_sensor.P_barA = HP_turbines_ext_P;
      HP_turbines_ext.alpha = 1;

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
  steam_dryer.x_steam_out = 0.9;

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
    superheated_steam_T_sensor.T_degC = superheated_steam_temperature;

  // LP systems
    // LP Turbine 1
      // Observable used for calibration
      //LP_turbine_1_P_in_sensor.P_barA = LP_turbine_1_P_in;

      // Calibrated parameters
      LP_turbine_1.Cst = LP_turbine_1_Cst;
      LP_turbine_1.eta_is = HP_LP_turbines_eta_is;

      // Hypothesis : no nozzle
      LP_turbine_1.eta_nz = 1;
      LP_turbine_1.area_nz = 1;

      // Extraction 1
      LP_turbines_ext_P_sensor.P_barA = LP_turbines_ext_P;
      LP_turbines_ext.alpha = 1;

    // LP Turbine 2
      // Calibrated parameters
      LP_turbine_2.Cst = LP_turbine_2_Cst;
      LP_turbine_2.eta_is = HP_LP_turbines_eta_is;

      // Hypothesis : no nozzle
      LP_turbine_2.eta_nz = 1;
      LP_turbine_2.area_nz = 1;

      // Observable used for calibration
      condenser_P_in_sensor.P_mbar = condenser_P_in;

  // Generator
    // Observable used for calibration
    W_tot_sensor.W_MW = ActivePower; // Calibrates STs_eta_is

    // Fixed parameter
    generator.eta = 0.99;

  // Temporary components
  temp_feedwater_source.Q_out = -2000;
  temp_HP_turbines_ext_sink.Q_in = 100;
  temp_LP_turbines_ext_sink.Q_in = 100;
  connect(steam_generator_P_in_sensor.C_out, steam_generator.feedwater_inlet) annotation (Line(points={{-124,-70},{-163,-70}},
                                                                                                                             color={28,108,200}));
  connect(steam_generator.purge_outlet, blow_down_sink.C_in) annotation (Line(points={{-174,-115.233},{-174,-132},{-185,-132}},
                                                                                                                   color={28,108,200}));
  connect(steam_generator_P_in_sensor.C_in, temp_feedwater_source.C_out) annotation (Line(points={{-112,-70},{157,-70}},
                                                                                                                       color={28,108,200}));
  connect(steam_generator.steam_outlet, HP_control_valve.C_in) annotation (Line(points={{-174,-24},{-174,72.1818},{-136,72.1818}},color={28,108,200}));
  connect(HP_control_valve.C_out, HP_turbine_1_P_in_sensor.C_in) annotation (Line(points={{-126,72.1818},{-116,72.1818},{-116,72.1818},{-106,72.1818}},
                                                                                                                                                      color={28,108,200}));
  connect(HP_control_valve.Opening, HP_control_valve_opening_sensor.Opening) annotation (Line(points={{-131,80.9091},{-131,85.9}}, color={0,0,127}));
  connect(HP_turbine_1.C_out, HP_turbines_ext.C_in) annotation (Line(points={{-62,72.1818},{-46.6,72.1818}}, color={28,108,200}));
  connect(HP_turbines_ext.C_main_out, HP_turbine_2.C_in) annotation (Line(points={{-25.4,72.1818},{-8,72.1818}}, color={28,108,200}));
  connect(HP_turbines_ext.C_ext_out, HP_turbines_ext_P_sensor.C_in) annotation (Line(points={{-36,65.3818},{-36,54}}, color={28,108,200}));
  connect(HP_turbines_ext_P_sensor.C_out, temp_HP_turbines_ext_sink.C_in) annotation (Line(points={{-36,40},{-36,27}}, color={28,108,200}));
  connect(powerSink.C_in,W_tot_sensor. C_out) annotation (Line(points={{367,168},{359.88,168}},
                                                                                              color={244,125,35}));
  connect(W_tot_sensor.C_in,generator. C_out) annotation (Line(points={{348,168},{342,168}},
                                                                                           color={244,125,35}));
  connect(HP_turbine_2.C_W_out,generator. C_in) annotation (Line(points={{10,78.9018},{18,78.9018},{18,168},{315.6,168}},
                                                                                                                   color={244,125,35}));
  connect(HP_turbine_1.C_W_out,generator. C_in) annotation (Line(points={{-62,78.9018},{-54,78.9018},{-54,168},{315.6,168}},
                                                                                                           color={244,125,35}));
  connect(HP_turbine_1.C_in, HP_turbine_1_P_in_sensor.C_out) annotation (Line(points={{-80,72.1818},{-94,72.1818}},                 color={28,108,200}));
  connect(HP_turbine_2_P_out_sensor.C_in, HP_turbine_2.C_out) annotation (Line(points={{20,72.1818},{10,72.1818}},
                                                                                                         color={28,108,200}));
  connect(steam_dryer.C_in, HP_turbine_2_P_out_sensor.C_out) annotation (Line(points={{48,71.6363},{40,71.6363},{40,72.1818},{32,72.1818}}, color={28,108,200}));
  connect(steam_dryer.C_hot_liquid, temp_steam_dryer_condensates_sink.C_in) annotation (Line(points={{64,65.0909},{64,27.1818}}, color={28,108,200}));
  connect(superheater.C_cold_in, steam_dryer.C_hot_steam) annotation (Line(points={{72,104},{72,71.6363},{64,71.6363}}, color={28,108,200}));
  connect(superheater.C_vent, superheater_vent_sink.C_in) annotation (Line(points={{88,104.2},{88,95.182}}, color={28,108,200}));
  connect(superheater.C_hot_out, superheater_drains_P_out_sensor.C_in) annotation (Line(points={{88,112},{100,112}}, color={28,108,200}));
  connect(superheater_drains_P_out_sensor.C_out, temp_superheater_drains_sink.C_in) annotation (Line(points={{112,112},{126,112},{126,27.182}},                color={28,108,200}));
  connect(superheated_steam_T_sensor.C_in,superheater. C_cold_out) annotation (Line(points={{88,130},{71.8,130.182},{71.8,120}}, color={28,108,200}));
  connect(superheater_control_valve.C_in, HP_control_valve.C_in) annotation (Line(points={{-136,112.182},{-174,112.182},{-174,72.1818},{-136,72.1818}}, color={28,108,200}));
  connect(superheater_control_valve.Opening, superheater_control_valve_opening_sensor.Opening) annotation (Line(points={{-131,120.909},{-131,131.9}}, color={0,0,127}));
  connect(superheater.C_hot_in, superheater_hot_P_in_sensor.C_out) annotation (Line(points={{56,112.2},{-35,112.2},{-35,112.182},{-94,112.182}}, color={28,108,200}));
  connect(superheater_hot_P_in_sensor.C_in, superheater_control_valve.C_out) annotation (Line(points={{-106,112.182},{-116,112.182},{-116,112.182},{-126,112.182}}, color={28,108,200}));
  connect(LP_turbine_1.C_out, LP_turbines_ext.C_in) annotation (Line(points={{170,130.182},{174,130.182},{174,130},{178,130},{178,130.182},{185.4,130.182}}, color={28,108,200}));
  connect(LP_turbines_ext.C_main_out, LP_turbine_2.C_in) annotation (Line(points={{206.6,130.182},{212,130.182},{212,130},{216,130},{216,130.182},{224,130.182}}, color={28,108,200}));
  connect(LP_turbines_ext.C_ext_out, LP_turbines_ext_P_sensor.C_in) annotation (Line(points={{196,123.382},{196,112}},
                                                                                                                     color={28,108,200}));
  connect(LP_turbines_ext_P_sensor.C_out, temp_LP_turbines_ext_sink.C_in) annotation (Line(points={{196,98},{196,85}}, color={28,108,200}));
  connect(LP_turbine_1.C_W_out,generator. C_in) annotation (Line(points={{170,136.902},{188,136.902},{188,168},{315.6,168}},
                                                                                                           color={244,125,35}));
  connect(LP_turbine_2.C_W_out, generator.C_in) annotation (Line(points={{242,136.902},{262,136.902},{262,168},{315.6,168}}, color={244,125,35}));
  connect(LP_turbine_2.C_out, condenser_P_in_sensor.C_in) annotation (Line(points={{242,130.182},{264,130.182},{264,130},{286,130}}, color={28,108,200}));
  connect(condenser_P_in_sensor.C_out, temp_main_steam_sink.C_in) annotation (Line(points={{298,130},{379.5,130},{379.5,130.182},{461,130.182}}, color={28,108,200}));
  connect(superheated_steam_T_sensor.C_out, LP_turbine_1_P_in_sensor.C_in) annotation (Line(points={{100,130},{103,130},{103,130.182},{106,130.182}}, color={28,108,200}));
  connect(LP_turbine_1_P_in_sensor.C_out, LP_turbine_1.C_in) annotation (Line(points={{118,130.182},{126,130},{126,130.182},{152,130.182}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{480,240}})), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{480,240}})));
end MetroscopiaNPP_reverse;
