within MetroscopeModelingLibrary.Examples.NuclearPowerPlant.MetroscopiaNPP;
model MetroscopiaNPP_reverse
  // Boundary Conditions
    // Steam generator
    input Real steam_generator_vapor_fraction(start = 1);
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
    input Real HP_control_valve_opening(start=0.8);

    // HP turbines
    input Real HP_turbine_1_P_in(start=48.5305) "barA";
    input Real HP_turbine_2_P_out(start=19.3986) "barA";
    input Real HP_steam_extraction_P_out(start=31.0032) "barA";

    // Superheater
    input Real superheater_drains_P_out(start=18.8965) "barA";
    input Real superheated_steam_temperature(start=228.021) "°C";

  // Calibrated parameters
    // HP turbines inlet control valve
    output Units.Cv HP_control_valve_Cvmax;
    output Units.Cv HP_control_valve_Cv;

    // HP Turbines
    output Units.Cst HP_turbine_1_Cst;
    output Units.Cst HP_turbine_2_Cst;

    output Units.Yield HP_turbines_eta_is;

  // Generator
  input Real ActivePower(start=568.78) "MW";

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
      WaterSteam.Pipes.SteamExtractionSplitter HP_turbine_1_ext annotation (Placement(transformation(extent={{-46,62.1818},{-26,80.1818}})));
      Sensors.WaterSteam.WaterPressureSensor HP_turbine_1_ext_P_sensor annotation (Placement(transformation(extent={{-7,-7},{7,7}},rotation=270,origin={-36,47})));


    // Generator
    Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{152,158},{172,178}})));
    Power.Machines.Generator generator annotation (Placement(transformation(extent={{98,156},{138,180}})));
    Sensors.Power.PowerSensor W_tot_sensor annotation (Placement(transformation(extent={{138,162},{150,174}})));

    // Temporary components
    WaterSteam.BoundaryConditions.Source temp_feedwater_source annotation (Placement(transformation(extent={{172,-80},{152,-60}})));
    WaterSteam.BoundaryConditions.Sink temp_main_steam_sink annotation (Placement(transformation(extent={{156,120.182},{176,140.182}})));


  // Unclassified components
  WaterSteam.BoundaryConditions.Sink HP_turbine_1_ext_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=270,origin={-36,22})));


  Sensors.WaterSteam.WaterPressureSensor HP_turbine_2_P_out_sensor annotation (Placement(transformation(extent={{20,66.1818},{32,78.1818}})));
  WaterSteam.Volumes.SteamDryer steam_dryer
    annotation (Placement(transformation(extent={{48,60.1818},{64,78.1818}})));
    WaterSteam.BoundaryConditions.Sink temp_steam_dryer_condensates_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={64,22.1818})));
  WaterSteam.HeatExchangers.SuperHeater superheater annotation (Placement(transformation(extent={{56,104},{88,120}})));
    WaterSteam.BoundaryConditions.Sink temp_superheater_drains_sink annotation (Placement(transformation(extent={{156,102.182},{176,122.182}})));
    WaterSteam.BoundaryConditions.Sink superheater_vent_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={88,90.182})));
  Sensors.WaterSteam.WaterPressureSensor superheater_drains_P_out_sensor annotation (Placement(transformation(extent={{100,106},{112,118}})));
  Sensors.WaterSteam.WaterTemperatureSensor superheated_steam_T_sensor annotation (Placement(transformation(extent={{88,124},{100,136}})));
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
      HP_turbine_1.eta_is = HP_turbines_eta_is;

      // Hypothesis : no nozzle
      HP_turbine_1.eta_nz = 1;
      HP_turbine_1.area_nz = 1;

      // Extraction 1
      HP_turbine_1_ext_P_sensor.P_barA = 53;
      HP_turbine_1_ext.alpha = 1;

    // HP Turbine 2
      // Calibrated parameters
      HP_turbine_2.Cst = HP_turbine_2_Cst;
      HP_turbine_2.eta_is = HP_turbines_eta_is;

      // Hypothesis : no nozzle
      HP_turbine_2.eta_nz = 1;
      HP_turbine_2.area_nz = 1;

      // Observable used for calibration
      HP_turbine_2_P_out_sensor.P_barA = HP_turbine_2_P_out;

  // Steam dryer
  steam_dryer.x_steam_out = 0.9;

  // Superheater
    // Fixed parameters
    superheater.S = 100;
    superheater.Kfr_cold = 0;
    superheater.Q_vent = 1;

    // Observables used for calibration
    superheater_drains_P_out_sensor.P_barA = superheater_drains_P_out;
    superheated_steam_T_sensor.T_degC = superheated_steam_temperature;

  // Generator
    // Observable used for calibration
    W_tot_sensor.W_MW = ActivePower; // Calibrates STs_eta_is

    // Fixed parameter
    generator.eta = 0.99;

  // Temporary components
  temp_feedwater_source.Q_out = -1000;
  HP_turbine_1_ext_sink.Q_in = 100;
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
  connect(HP_turbine_1.C_out,HP_turbine_1_ext. C_in) annotation (Line(points={{-62,72.1818},{-46.6,72.1818}},
                                                                                                   color={28,108,200}));
  connect(HP_turbine_1_ext.C_main_out,HP_turbine_2. C_in) annotation (Line(points={{-25.4,72.1818},{-8,72.1818}},
                                                                                     color={28,108,200}));
  connect(HP_turbine_1_ext.C_ext_out,HP_turbine_1_ext_P_sensor. C_in) annotation (Line(points={{-36,65.3818},{-36,54}},
                                                                                                                      color={28,108,200}));
  connect(HP_turbine_1_ext_P_sensor.C_out,HP_turbine_1_ext_sink. C_in) annotation (Line(points={{-36,40},{-36,27}},   color={28,108,200}));
  connect(powerSink.C_in,W_tot_sensor. C_out) annotation (Line(points={{157,168},{149.88,168}},
                                                                                              color={244,125,35}));
  connect(W_tot_sensor.C_in,generator. C_out) annotation (Line(points={{138,168},{132,168}},
                                                                                           color={244,125,35}));
  connect(HP_turbine_2.C_W_out,generator. C_in) annotation (Line(points={{10,78.9018},{18,78.9018},{18,168},{105.6,168}},
                                                                                                                   color={244,125,35}));
  connect(HP_turbine_1.C_W_out,generator. C_in) annotation (Line(points={{-62,78.9018},{-54,78.9018},{-54,168},{105.6,168}},
                                                                                                           color={244,125,35}));
  connect(HP_turbine_1.C_in, HP_turbine_1_P_in_sensor.C_out) annotation (Line(points={{-80,72.1818},{-94,72.1818}},                 color={28,108,200}));
  connect(HP_turbine_2_P_out_sensor.C_in, HP_turbine_2.C_out) annotation (Line(points={{20,72.1818},{10,72.1818}},
                                                                                                         color={28,108,200}));
  connect(steam_dryer.C_in, HP_turbine_2_P_out_sensor.C_out) annotation (Line(points={{48,71.6363},{40,71.6363},{40,72.1818},{32,72.1818}}, color={28,108,200}));
  connect(steam_dryer.C_hot_liquid, temp_steam_dryer_condensates_sink.C_in) annotation (Line(points={{64,65.0909},{64,27.1818}}, color={28,108,200}));
  connect(superheater.C_cold_in, steam_dryer.C_hot_steam) annotation (Line(points={{72,104},{72,71.6363},{64,71.6363}}, color={28,108,200}));
  connect(superheater.C_vent, superheater_vent_sink.C_in) annotation (Line(points={{88,104.2},{88,95.182}}, color={28,108,200}));
  connect(superheater.C_hot_out, superheater_drains_P_out_sensor.C_in) annotation (Line(points={{88,112},{100,112}}, color={28,108,200}));
  connect(superheater_drains_P_out_sensor.C_out, temp_superheater_drains_sink.C_in) annotation (Line(points={{112,112},{150,112},{150,112.182},{161,112.182}}, color={28,108,200}));
  connect(temp_main_steam_sink.C_in, superheated_steam_T_sensor.C_out) annotation (Line(points={{161,130.182},{130.5,130.182},{130.5,130},{100,130}}, color={28,108,200}));
  connect(superheated_steam_T_sensor.C_in,superheater. C_cold_out) annotation (Line(points={{88,130},{71.8,130.182},{71.8,120}}, color={28,108,200}));
  connect(superheater.C_hot_in, HP_control_valve.C_in) annotation (Line(points={{56,112.2},{-174,112.2},{-174,72.1818},{-136,72.1818}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{180,180}})), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{180,180}})));
end MetroscopiaNPP_reverse;
