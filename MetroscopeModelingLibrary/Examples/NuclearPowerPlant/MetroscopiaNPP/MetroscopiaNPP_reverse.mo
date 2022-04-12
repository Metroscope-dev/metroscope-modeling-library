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
    WaterSteam.HeatExchangers.SteamGenerator steam_generator annotation (Placement(transformation(extent={{-196,-52},{-152,40}})));
    Sensors.WaterSteam.WaterPressureSensor steam_generator_P_in_sensor annotation (Placement(transformation(extent={{-112,-12},{-124,0}})));
    WaterSteam.BoundaryConditions.Sink blow_down_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=180,origin={-190,-68})));

    // HP
      // HP Control Valve
      WaterSteam.Pipes.ControlValve HP_control_valve annotation (Placement(transformation(extent={{-136,70},{-126,82}})));
      Sensors.Other.OpeningSensor HP_control_valve_opening_sensor annotation (Placement(transformation(extent={{-136,92},{-126,102}})));

      // HP Turbines
      Sensors.WaterSteam.WaterPressureSensor HP_turbine_1_P_in_sensor annotation (Placement(transformation(extent={{-106,66.1818},{-94,78.1818}})));
      WaterSteam.Machines.StodolaTurbine HP_turbine_1 annotation (Placement(transformation(extent={{-80,64.1818},{-62,80.1818}})));
      WaterSteam.Machines.StodolaTurbine HP_turbine_2 annotation (Placement(transformation(extent={{-8,64.1818},{10,80.1818}})));
      WaterSteam.Pipes.SteamExtractionSplitter HP_turbine_1_ext annotation (Placement(transformation(extent={{-46,62.1818},{-26,80.1818}})));
      Sensors.WaterSteam.WaterPressureSensor HP_turbine_1_ext_P_sensor annotation (Placement(transformation(extent={{-7,-7},{7,7}},rotation=270,origin={-36,47})));


    // Generator
    Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{152,104},{172,124}})));
    Power.Machines.Generator generator annotation (Placement(transformation(extent={{98,102},{138,126}})));
    Sensors.Power.PowerSensor W_tot_sensor annotation (Placement(transformation(extent={{138,108},{150,120}})));

    // Temporary components
    WaterSteam.BoundaryConditions.Source temp_feedwater_source annotation (Placement(transformation(extent={{172,-16},{152,4}})));
    WaterSteam.BoundaryConditions.Sink temp_main_steam_sink annotation (Placement(transformation(extent={{152,62.1818},{172,82.1818}})));


  // Unclassified components
  WaterSteam.BoundaryConditions.Sink HP_turbine_1_ext_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}},rotation=270,origin={-36,22})));


  Sensors.WaterSteam.WaterPressureSensor HP_turbine_2_P_out_sensor annotation (Placement(transformation(extent={{20,66.1818},{32,78.1818}})));
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


  // Generator
    // Observable used for calibration
    W_tot_sensor.W_MW = ActivePower; // Calibrates STs_eta_is

    // Fixed parameter
    generator.eta = 0.99;

  // Temporary components
  temp_feedwater_source.Q_out = -1000;
  HP_turbine_1_ext_sink.Q_in = 100;
  connect(steam_generator_P_in_sensor.C_out, steam_generator.feedwater_inlet) annotation (Line(points={{-124,-6},{-163,-6}}, color={28,108,200}));
  connect(steam_generator.purge_outlet, blow_down_sink.C_in) annotation (Line(points={{-174,-51.2333},{-174,-68},{-185,-68}},
                                                                                                                   color={28,108,200}));
  connect(steam_generator_P_in_sensor.C_in, temp_feedwater_source.C_out) annotation (Line(points={{-112,-6},{157,-6}}, color={28,108,200}));
  connect(steam_generator.steam_outlet, HP_control_valve.C_in) annotation (Line(points={{-174,40},{-174,72.1818},{-136,72.1818}}, color={28,108,200}));
  connect(HP_control_valve.C_out, HP_turbine_1_P_in_sensor.C_in) annotation (Line(points={{-126,72.1818},{-116,72.1818},{-116,72.1818},{-106,72.1818}},
                                                                                                                                                      color={28,108,200}));
  connect(HP_control_valve.Opening, HP_control_valve_opening_sensor.Opening) annotation (Line(points={{-131,80.9091},{-131,91.9}}, color={0,0,127}));
  connect(HP_turbine_1.C_out,HP_turbine_1_ext. C_in) annotation (Line(points={{-62,72.1818},{-46.6,72.1818}},
                                                                                                   color={28,108,200}));
  connect(HP_turbine_1_ext.C_main_out,HP_turbine_2. C_in) annotation (Line(points={{-25.4,72.1818},{-8,72.1818}},
                                                                                     color={28,108,200}));
  connect(HP_turbine_1_ext.C_ext_out,HP_turbine_1_ext_P_sensor. C_in) annotation (Line(points={{-36,65.3818},{-36,54}},
                                                                                                                      color={28,108,200}));
  connect(HP_turbine_1_ext_P_sensor.C_out,HP_turbine_1_ext_sink. C_in) annotation (Line(points={{-36,40},{-36,27}},   color={28,108,200}));
  connect(powerSink.C_in,W_tot_sensor. C_out) annotation (Line(points={{157,114},{149.88,114}},
                                                                                              color={244,125,35}));
  connect(W_tot_sensor.C_in,generator. C_out) annotation (Line(points={{138,114},{132,114}},
                                                                                           color={244,125,35}));
  connect(HP_turbine_2.C_W_out,generator. C_in) annotation (Line(points={{10,78.9018},{18,78.9018},{18,114},{105.6,114}},
                                                                                                                   color={244,125,35}));
  connect(HP_turbine_1.C_W_out,generator. C_in) annotation (Line(points={{-62,78.9018},{-54,78.9018},{-54,114},{105.6,114}},
                                                                                                           color={244,125,35}));
  connect(HP_turbine_1.C_in, HP_turbine_1_P_in_sensor.C_out) annotation (Line(points={{-80,72.1818},{-94,72.1818}},                 color={28,108,200}));
  connect(temp_main_steam_sink.C_in, HP_turbine_2_P_out_sensor.C_out) annotation (Line(points={{157,72.1818},{32,72.1818}},                   color={28,108,200}));
  connect(HP_turbine_2_P_out_sensor.C_in, HP_turbine_2.C_out) annotation (Line(points={{20,72.1818},{10,72.1818}},
                                                                                                         color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{180,140}})), Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-140},{180,140}})));
end MetroscopiaNPP_reverse;
