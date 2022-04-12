within MetroscopeModelingLibrary.Examples.NuclearPowerPlant;
model MetroscopiaNPP_reverse_MML2
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  // Boundary Conditions
  //input Real liquidFractionSG(start = 0) "%";
  input Real PressureSG(start = 50) "barA";
  input Real PressureCS(start = 3) "barA";
  input Real TemperatureCS(start = 15) "°C";
  input Real VolumeFlowRateCS(start = 50) "m3/s";
  input Real ThermalPower(start = 1880) "MWth";

  // Observables used for calibration
  input Real ActivePower(start=568.78) "MW";

    // Steam generator
    input Real Qo_SteamDryer(start=45.7395) "kg/s";
    input Real Pi_SteamGenerator(start=57.782) "barA";

    // HP turbines
    input Real Po_HighPressureTurbine(start=19.3986) "barA";
    input Real Po_HPSteamExtraction(start=31.0032) "barA";
    input Real Pi_HighPressureTurbine(start=48.5305) "barA";

    // Superheater
    input Real Pc_Superheater(start=40.3773) "barA";

    // LP turbines
    input Real Po_LPSteamExtraction(start=5.0001) "barA";
    input Real Pi_LowPressureTurbine(start=18.8965) "barA";
    input Real Ti_LowPressureTurbine(start=228.021) "°C";

    // Condenser
    input Real Po_Condenser(start=69.7982) "mbar";

    // LP pump
    input Real Po_ValveWaterSuctionPump(start=20.5589) "barA";
    input Real Po_WaterSuctionPump(start=20.7963) "barA";

    // FeedWater tank
    input Real Po_FeedWaterTank(start=18.2933) "barA";

    // FWP (HP pump)
    input Real Pi_FeedWaterPump(start=18.3192) "barA";
    input Real Po_FeedWaterPump(start=59.464) "barA";

    // LP Reheater
    input Real To_LowPressureReheater(start=72.8234) "°C";

    // HP Reheater
    input Real To_HighPressureReheater(start=218.034) "°C";
    input Real Tc_HighPressureReheater(start=216.711) "°C";

  // Parameters to calibrate
  output Real condenser_Kth;
  output Real LowPressureTurbine_2_Cst;
  output Real LowPressureTurbine_1_Cst;
  //output Real Superheater_Kfr_cold; //Removed
  output Real Superheater_Kfr_hot;
  output Real HighPressureTurbine_2_Cst;
  output Real HighPressureTurbine_1_Cst;
  output Real LPpump_hn;
  output Real LPReheater_Kfr_cold;
  output Real PressureLoss_after_drum_Kfr;
  output Real PressureLoss_after_drum_Q;
  output Real PressureLoss_after_drum_rhom;
  output Real HPpump_hn;
  output Real HighPressureTurbines_eta_is;
  output Real LowPressureTurbines_eta_is;
  output Real SuperHeaterControlValve_Cvmax;
  output Real Superheater_Kth;
  output Real LPReheater_Kth;
  output Real HPReheater_Kfr_cold;
  output Real HPReheater_Kth_cond;
  output Real HPReheater_Kth_subc;

  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve
    HPControlValve
    annotation (Placement(transformation(extent={{-62,36},{-52,26}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine
    HighPressureTurbine_1
    annotation (Placement(transformation(extent={{-30,24},{-10,44}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe
    PressureLoss_SteamExtractionHP annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={6,10})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter
    SteamExtraction_HP
    annotation (Placement(transformation(extent={{-4,24},{16,42}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine
    HighPressureTurbine_2
    annotation (Placement(transformation(extent={{28,22},{48,42}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter SteamExtraction_BeforeDryer annotation (Placement(transformation(extent={{62,24},{82,42}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe PressureLoss_ExtractionBeforeDryer
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={72,-6})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe PressureLoss_ExhaustHP
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={98,34})));
  MetroscopeModelingLibrary.WaterSteam.Volumes.SteamDryer steamDryer
    annotation (Placement(transformation(extent={{132,22},{154,44}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe PressureLoss_DryerCondensats
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={154,-6})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.SuperHeater Superheater
    annotation (Placement(transformation(extent={{152,50},{184,66}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve SuperHeaterControlValve
    annotation (Placement(transformation(extent={{30,56},{40,66}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine LowPressureTurbine_1
    annotation (Placement(transformation(extent={{208,72},{228,92}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter SteamExtraction_LP
    annotation (Placement(transformation(extent={{240,70},{262,90}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe PressureLoss_LPExtraction annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={250,22})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine LowPressureTurbine_2
    annotation (Placement(transformation(extent={{280,72},{300,92}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source coldSource
    annotation (Placement(transformation(extent={{342,-8},{358,8}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe PressureLoss_Condenser
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={380,-60})));
  MetroscopeModelingLibrary.WaterSteam.Machines.Pump LPpump
    annotation (Placement(transformation(extent={{358,-86},{338,-66}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.DryReheater LPReheater
    annotation (Placement(transformation(extent={{262,-84},{230,-68}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve PumpControlValve
    annotation (Placement(transformation(extent={{308,-78},{298,-68}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve steamDryerValve
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=-90,
        origin={157,-31})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve LPReheaterControlValve
    annotation (Placement(transformation(extent={{318,-124},{328,-114}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.Pump HPpump
    annotation (Placement(transformation(extent={{80,-86},{60,-66}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe PressureLoss_before_drum
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={182,-76})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe PressureLoss_after_drum
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={126,-76})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe PressureLoss_SuperheaterDrains
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={202,16})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe PressureLoss_MainSteamExtraction
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={6,-46})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater HPReheater
    annotation (Placement(transformation(extent={{22,-84},{-10,-68}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve HPReheaterControlValve annotation (Placement(transformation(extent={{92,-124},{102,-114}})));
  Power.Machines.Generator generator
    annotation (Placement(transformation(extent={{334,112},{382,140}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve HPReheater_valve
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=0,
        origin={43,-17})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser
    annotation (Placement(transformation(extent={{367,-16},{393,6}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{436,116},{458,136}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{54,-60},{64,-50}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source source1
    annotation (Placement(transformation(extent={{336,-60},{346,-50}})));

  // Sensors
  MetroscopeModelingLibrary.Sensors.Power.PowerSensor ActivePower_sensor
    annotation (Placement(transformation(extent={{400,119},{414,133}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor
    Po_Condenser_sensor annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=270,
        origin={380,-27})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor
    Po_LPSteamExtraction_sensor annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={250,56})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor
    Pi_LowPressureTurbine_sensor
    annotation (Placement(transformation(extent={{174,76},{186,88}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor
    Po_HighPressureTurbine_sensor
    annotation (Placement(transformation(extent={{114,28},{126,40}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor
    Po_HPSteamExtraction_sensor annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={6,-10})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor
    Pi_HighPressureTurbine_sensor
    annotation (Placement(transformation(extent={{-48,28},{-36,40}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor
    Po_WaterSuctionPump_sensor
    annotation (Placement(transformation(extent={{328,-82},{316,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor Po_FeedWaterTank_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={72,-30})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor Po_ValveWaterSuctionPump_sensor
    annotation (Placement(transformation(extent={{292,-82},{280,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor Pi_FeedWaterPump_sensor
    annotation (Placement(transformation(extent={{96,-82},{84,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor Po_FeedWaterPump_sensor
    annotation (Placement(transformation(extent={{52,-82},{40,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor Pi_SteamGenerator_sensor
    annotation (Placement(transformation(extent={{-58,-82},{-70,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor Pc_Superheater_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={202,-8})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor Qo_SteamDryer_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={154,16})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor Ti_LowPressureTurbine_sensor
    "same value as To_Superheater"
    annotation (Placement(transformation(extent={{190,76},{202,88}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor To_LowPressureReheater_sensor
    annotation (Placement(transformation(extent={{204,-82},{216,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor To_HighPressureReheater_sensor
    "same value as Ti_SteamGenerator"
    annotation (Placement(transformation(extent={{-30,-82},{-18,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor Tc_HighPressureReheater_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={52,-122.182})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor Qi_SteamGenerator_sensor
    annotation (Placement(transformation(extent={{-38,-82},{-50,-70}})));
  WaterSteam.BoundaryConditions.Source waterSource annotation (Placement(transformation(extent={{-120,24},{-100,44}})));
equation
  // ----- Boundary Conditions ------
  // steamGenerator.thermal_power = ThermalPower*1e6;
  waterSource.h_out = 2.6e6; // ADDED
  waterSource.Q_out = 1000; // ADDED
  waterSource.P_out = PressureSG*1e5;
  //ThermalPower*1e6 = -waterSource.Q_out*waterSource.h_out - waterSink.Q_in*waterSink.h_in; // ADDED
  //steamGenerator.vapor_fraction = 1 - liquidFractionSG/100;

  coldSource.P_out = PressureCS*1e5;
  coldSource.T_out = TemperatureCS + 273.15;
  coldSource.Qv_out = -VolumeFlowRateCS;

  // ----- Components ------
  // SteamGenerator

  //Qi_SteamGenerator = Qi_SteamGenerator_sensor.Q;
  Pi_SteamGenerator = Pi_SteamGenerator_sensor.P_barA;
/*
  steamGenerator.P_purge = steamGenerator.feedwater_sink.P_in; // ADDED
  sinkBlowOff.Q_in = 1e-2;
*/
  // HighPressureTurbines
    // HPControlValve
    HPControlValve.Cvmax = 1e4;

    // Observables used for calibration (inlet)
    Pi_HighPressureTurbine = Pi_HighPressureTurbine_sensor.P_barA;

    // HighPressureTurbine_1
      // Calibrated parameters
      HighPressureTurbine_1_Cst = HighPressureTurbine_1.Cst;
      HighPressureTurbines_eta_is = HighPressureTurbine_1.eta_is;

      // Fixed parameter
      HighPressureTurbine_1.eta_nz = 1;
      HighPressureTurbine_1.area_nz = 1;

    // Steam extraction HP
      // Fixed parameter
      SteamExtraction_HP.alpha = 1;

      // Steam extraction HP pressure losses
      PressureLoss_SteamExtractionHP.Kfr = 1e-3;
      PressureLoss_SteamExtractionHP.delta_z = 0;

      // Observables used for calibration (extraction)
      Po_HPSteamExtraction = Po_HPSteamExtraction_sensor.P_barA;

    // HighPressureTurbine_2
      // Calibrated parameters
      HighPressureTurbine_2_Cst = HighPressureTurbine_2.Cst;
      HighPressureTurbines_eta_is = HighPressureTurbine_2.eta_is;

      // Fixed parameter
      HighPressureTurbine_2.eta_nz = 1;
      HighPressureTurbine_2.area_nz = 1;

    // Observables used for calibration (Outlet)
    Po_HighPressureTurbine = Po_HighPressureTurbine_sensor.P_barA;

  // Steam extraction before dryer
    // Fixed parameter
    SteamExtraction_BeforeDryer.alpha = 1;

    // Pressure Loss
    //PressureLoss_ExtractionBeforeDryer.Kfr;
    PressureLoss_ExtractionBeforeDryer.delta_z = 0;

  // Steam Dryer
    // Inlet pressure loss
    PressureLoss_ExhaustHP.delta_z = 0;
    PressureLoss_ExhaustHP.Kfr = 1e-6;

    // Fixed parameter
    steamDryer.x_steam_out = 0.99;

    // Observable used for calibration
    //Qo_SteamDryer = Qo_SteamDryer_sensor.Q; // TEMPORARY REMOVED

    // Condensates
      // Pressure loss
      PressureLoss_DryerCondensats.delta_z = 0;
      PressureLoss_DryerCondensats.Kfr = 1e-6;

      // Valve
      steamDryerValve.Cvmax = 1e4;

  // Superheater
    // Control valve (Hot inlet)
      // Fixed parameter
      SuperHeaterControlValve.Opening = 1;

      // Calibrated parameter
      SuperHeaterControlValve_Cvmax = SuperHeaterControlValve.Cvmax;

    // Calibrated parameters
    //Superheater_Kfr_cold = Superheater.Kfr_cold; REMOVED
    Superheater_Kth = Superheater.Kth;

    // Fixed parameter
    Superheater.S = 100;
    Superheater.Kfr_hot = Superheater_Kfr_hot; // REPLACED
    Superheater.C_vent.Q = -2.5; // ADDED


    // Observable used for calibration (Cooled outlet)
    Pc_Superheater = Pc_Superheater_sensor.P_barA;

    // Drains oressure loss
    PressureLoss_SuperheaterDrains.delta_z = 0;
    PressureLoss_SuperheaterDrains.Kfr = 1e-6;

  // Combined extractions pressure loss
  PressureLoss_MainSteamExtraction.delta_z = 0;
  PressureLoss_MainSteamExtraction.Kfr = 1e-6;

  // LP Turbines
    // Observable used for calibration
    Ti_LowPressureTurbine = Ti_LowPressureTurbine_sensor.T_degC;
    Pi_LowPressureTurbine = Pi_LowPressureTurbine_sensor.P_barA;

    // LP Turbine 1
      // Fixed parameters
      LowPressureTurbine_1.eta_nz = 1;
      LowPressureTurbine_1.area_nz = 1;

      // Calibrated parameters
      LowPressureTurbine_1_Cst = LowPressureTurbine_1.Cst;
      LowPressureTurbines_eta_is = LowPressureTurbine_1.eta_is;

    // Steam extraction
      // Fixed parameter
      SteamExtraction_LP.alpha = 1;

      // Observable used for calibration
      Po_LPSteamExtraction = Po_LPSteamExtraction_sensor.P_barA;

      // Steam extraction LP pressure losses
      PressureLoss_LPExtraction.Kfr = 1e-3;
      PressureLoss_LPExtraction.delta_z = 0;

    // LP Turbine 2
      // Fixed parameters
      LowPressureTurbine_2.eta_nz = 1;
      LowPressureTurbine_2.area_nz = 1;

      // Calibrated parameters
      LowPressureTurbine_2_Cst = LowPressureTurbine_2.Cst;
      LowPressureTurbines_eta_is = LowPressureTurbine_2.eta_is;

  // Generator
    // Fixed parameters
    generator.eta = 0.98;

    // Observable used for calibration
    ActivePower = ActivePower_sensor.W_MW;

  // Condenser
    // Fixed parameters
    condenser.Kfr_cold = 0.00600135;
    condenser.S = 100;
    condenser.water_height = 0;
    condenser.C_incond = 0;
    condenser.P_offset = 0;

    // Calibrated parameters
    condenser_Kth = condenser.Kth;

    // Observable used for calibration
    Po_Condenser = Po_Condenser_sensor.P_mbar;

    // Outlet Pressure loss
    PressureLoss_Condenser.delta_z = 0;
    PressureLoss_Condenser.Kfr = 1e-6;

  // Condensate
    // LP pump
      // Fixed parameters
      LPpump.VRot = 1400;
      LPpump.VRotn = 1400;
      LPpump.rm = 0.85;
      LPpump.a1 = -78.0237;
      LPpump.a2 = 0;
      // LPpump.a3 = 250;
      LPpump.b1 = 0;
      LPpump.b2 = 0;
      LPpump.b3 = 0.7;
      LPpump.rhmin = 0.20;

      // Calibrated parameters
      LPpump_hn = LPpump.hn;

      // Observable used for calibration
      Po_WaterSuctionPump = Po_WaterSuctionPump_sensor.P_barA;

    // Outlet control valve
      // Fixed parameters
      PumpControlValve.Cvmax = 1e4;

      // Observable used for calibration
      Po_ValveWaterSuctionPump = Po_ValveWaterSuctionPump_sensor.P_barA;

  // LP Reheater
    // Calibrated parameters
    LPReheater_Kth = LPReheater.Kth;
    LPReheater_Kfr_cold = LPReheater.Kfr_cold;

    // Fixed parameters
    LPReheater.S_condensing = 100;
    LPReheater.Kfr_hot = 1;

    // Drains
    LPReheaterControlValve.Cvmax = 1e4;

    // Observable used for calibration
    To_LowPressureReheater = To_LowPressureReheater_sensor.T_degC;

  // Drum
    Po_FeedWaterTank_sensor.P_barA = Po_FeedWaterTank;
    // Pressure losses fixed parameters
    PressureLoss_before_drum.Kfr = 1e-3;
    PressureLoss_before_drum.delta_z = 0;

    PressureLoss_after_drum.delta_z = 0;

    // Calibrated parameter
    PressureLoss_after_drum_Kfr = PressureLoss_after_drum.Kfr;

    PressureLoss_after_drum_Q = PressureLoss_after_drum.Q;
    PressureLoss_after_drum_rhom = PressureLoss_after_drum.rhom;

    // Drum equation
    PressureLoss_after_drum.h_in = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(PressureLoss_after_drum.P_in));

  // FWP
    // Fixed parameters
    HPpump.VRot = 1400;
    HPpump.VRotn = 1400;
    HPpump.rm = 0.85;
    HPpump.a1 =-76.0123;
    HPpump.a2 =0;
    // HPpump.a3 = 596;
    HPpump.b1= 0;
    HPpump.b2 = 0;
    HPpump.b3 = 0.7;
    HPpump.rhmin =0.20;

    // Calibrated parameters
    HPpump_hn = HPpump.hn;

    // Observable used for calibration
    Pi_FeedWaterPump = Pi_FeedWaterPump_sensor.P_barA;
    Po_FeedWaterPump = Po_FeedWaterPump_sensor.P_barA;

  // HP Reheater
    // Fixed parameters
    HPReheater.S_tot = 100;
    HPReheater.level = 0.3;
    HPReheater.Kfr_hot = 1;

    // Calibrated parameters
    HPReheater_Kth_cond = HPReheater.Kth_cond;
    HPReheater_Kth_subc = HPReheater.Kth_subc;
    HPReheater_Kfr_cold = HPReheater.Kfr_cold;

    // Drains
    HPReheaterControlValve.Cvmax = 1e4;
    HPReheater_valve.Cvmax = 1e4;

    // Observable used for calibration
    To_HighPressureReheater = To_HighPressureReheater_sensor.T_degC;
    Tc_HighPressureReheater = Tc_HighPressureReheater_sensor.T_degC;

  connect(HighPressureTurbine_1.C_out, SteamExtraction_HP.C_in)
    annotation (Line(points={{-10,34},{-4.6,34}},  color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_HP.C_ext_out, PressureLoss_SteamExtractionHP.C_in)
    annotation (Line(points={{6,27.2},{6,20}},
                                             color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_HP.C_main_out, HighPressureTurbine_2.C_in)
    annotation (Line(points={{16.6,34},{22,34},{22,32},{28,32}},
                                                 color={238,46,47},
      thickness=0.5));
  connect(HighPressureTurbine_2.C_out, SteamExtraction_BeforeDryer.C_in) annotation (Line(
      points={{48,32},{54,32},{54,34},{61.4,34}},
      color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_BeforeDryer.C_main_out, PressureLoss_ExhaustHP.C_in) annotation (Line(
      points={{82.6,34},{88,34}},
      color={238,46,47},
      thickness=0.5));
  connect(steamDryer.C_hot_steam, Superheater.C_cold_in)
    annotation (Line(points={{154,36},{168,36},{168,50}}, color={238,46,47},
      thickness=0.5));
  connect(Superheater.C_hot_in, SuperHeaterControlValve.C_out) annotation (Line(
        points={{152,58.2},{140,58.2},{140,58},{40,58},{40,57.8182}},     color={238,46,
          47},
      thickness=0.5));
  connect(LowPressureTurbine_1.C_out, SteamExtraction_LP.C_in)
    annotation (Line(points={{228,82},{232,82},{232,81.1111},{239.34,81.1111}},
                                                     color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_LP.C_main_out, LowPressureTurbine_2.C_in)
    annotation (Line(points={{262.66,81.1111},{262.66,82},{280,82}},
                                                   color={238,46,47},
      thickness=0.5));
  connect(LPReheater.C_hot_in, PressureLoss_LPExtraction.C_out) annotation (Line(
      points={{246,-68},{250,-68},{250,12}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_DryerCondensats.C_out, steamDryerValve.C_in) annotation (
     Line(points={{154,-16},{154,-21.1},{153.818,-21.1},{153.818,-26}},  color={28,108,
          200},
      thickness=0.5));
  connect(PressureLoss_after_drum.C_in, PressureLoss_before_drum.C_out)
    annotation (Line(points={{136,-76},{172,-76}},   color={28,108,200},
      thickness=0.5));
  connect(Superheater.C_hot_out, PressureLoss_SuperheaterDrains.C_in) annotation (Line(
      points={{184,58},{202,58},{202,26}},
      color={28,108,200},
      thickness=0.5));
  connect(HPReheaterControlValve.C_out, PressureLoss_before_drum.C_out) annotation (Line(
      points={{102,-122.182},{154,-122.182},{154,-76},{172,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_MainSteamExtraction.C_out, HPReheater.C_hot_in) annotation (Line(
      points={{6,-56},{6,-68}},
      color={238,46,47},
      thickness=0.5));
  connect(HPReheater_valve.C_out, PressureLoss_MainSteamExtraction.C_in) annotation (Line(
      points={{38,-20.1818},{6,-20.1818},{6,-36}},
      color={238,46,47},
      thickness=0.5));
  connect(HighPressureTurbine_1.C_W_out, generator.C_in) annotation (Line(
        points={{-10,42.4},{10,42.4},{10,126},{343.12,126}},
                                                         color={255,128,0},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(LowPressureTurbine_2.C_W_out, generator.C_in) annotation (Line(
        points={{300,90.4},{320,90.4},{320,126},{343.12,126}},
                                                   color={255,128,0},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(LowPressureTurbine_1.C_W_out, generator.C_in) annotation (Line(
        points={{228,90.4},{270,90.4},{270,126},{343.12,126}},
                                                     color={255,128,0},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(HighPressureTurbine_2.C_W_out, generator.C_in) annotation (Line(
        points={{48,40.4},{62,40.4},{62,126},{343.12,126}},
                                                   color={255,128,0},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(LowPressureTurbine_2.C_out, condenser.C_hot_in) annotation (Line(
      points={{300,82},{380,82},{380,6}},
      color={238,46,47},
      thickness=0.5));
  connect(LPReheaterControlValve.C_out, condenser.C_hot_in) annotation (
      Line(
      points={{328,-122.182},{434,-122.182},{434,14},{380,14},{380,6}},
      color={63,81,181},
      thickness=0.5));

  connect(Po_FeedWaterTank_sensor.C_out, PressureLoss_before_drum.C_out)
    annotation (Line(
      points={{72,-36},{72,-40},{154,-40},{154,-76},{172,-76}},
      color={238,46,47},
      thickness=0.5));
  connect(HPpump.C_power, source.C_out)
    annotation (Line(points={{70,-65.2},{70,-55},{61.4,-55}},
                                                            color={255,128,0},
      pattern=LinePattern.Dash));
  connect(LPpump.C_power, source1.C_out) annotation (Line(points={{348,-65.2},{348,-55},{343.4,-55}},
                           color={255,128,0},
      pattern=LinePattern.Dash));
  connect(generator.C_out, ActivePower_sensor.C_in)
    annotation (Line(points={{374.8,126},{400,126}},     color={255,128,0},
      thickness=0.5,
      pattern=LinePattern.Dash));
  connect(ActivePower_sensor.C_out, sink.C_in)
    annotation (Line(points={{413.86,126},{441.5,126}},
                                                      color={255,128,0},
      thickness=0.5,
      pattern=LinePattern.Dash));
  connect(Po_Condenser_sensor.C_in, condenser.C_hot_out)
    annotation (Line(points={{380,-22},{380,-16.4889}}, color={28,108,200},
      thickness=0.5));
  connect(SteamExtraction_LP.C_ext_out, Po_LPSteamExtraction_sensor.C_in)
    annotation (Line(
      points={{251,73.5556},{250,73.5556},{250,62}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_LPSteamExtraction_sensor.C_out, PressureLoss_LPExtraction.C_in) annotation (Line(
      points={{250,50},{250,32}},
      color={28,108,200},
      thickness=0.5));
  connect(Superheater.C_cold_out, Pi_LowPressureTurbine_sensor.C_in)
    annotation (Line(
      points={{167.8,66},{167.8,82},{174,82}},
      color={238,46,47},
      thickness=0.5));
  connect(PressureLoss_ExhaustHP.C_out, Po_HighPressureTurbine_sensor.C_in)
    annotation (Line(
      points={{108,34},{114,34}},
      color={238,46,47},
      thickness=0.5));
  connect(Po_HighPressureTurbine_sensor.C_out, steamDryer.C_in) annotation (
      Line(
      points={{126,34},{132,34},{132,36}},
      color={238,46,47},
      thickness=0.5));
  connect(PressureLoss_SteamExtractionHP.C_out, Po_HPSteamExtraction_sensor.C_in)
    annotation (Line(
      points={{6,-3.55271e-15},{6,-4}},
      color={238,46,47},
      thickness=0.5));
  connect(Po_HPSteamExtraction_sensor.C_out, PressureLoss_MainSteamExtraction.C_in) annotation (Line(
      points={{6,-16},{6,-36}},
      color={238,46,47},
      thickness=0.5));
  connect(HPControlValve.C_out, Pi_HighPressureTurbine_sensor.C_in) annotation (Line(
      points={{-52,34.1818},{-49.95,34.1818},{-49.95,34},{-48,34}},
      color={238,46,47},
      thickness=0.5));
  connect(HighPressureTurbine_1.C_in, Pi_HighPressureTurbine_sensor.C_out) annotation (
     Line(
      points={{-30,34},{-36,34}},
      color={238,46,47},
      thickness=0.5));
  connect(PumpControlValve.C_in, Po_WaterSuctionPump_sensor.C_out) annotation (
      Line(
      points={{308,-76.1818},{308,-76},{316,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_WaterSuctionPump_sensor.C_in, LPpump.C_out) annotation (Line(
      points={{328,-76},{338,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_ExtractionBeforeDryer.C_out, Po_FeedWaterTank_sensor.C_in) annotation (Line(
      points={{72,-16},{72,-24}},
      color={238,46,47},
      thickness=0.5));
  connect(LPReheater.C_cold_in, Po_ValveWaterSuctionPump_sensor.C_out)
    annotation (Line(
      points={{262.2,-76},{280,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_ValveWaterSuctionPump_sensor.C_in, PumpControlValve.C_out)
    annotation (Line(
      points={{292,-76},{296,-76},{296,-76.1818},{298,-76.1818}},
      color={28,108,200},
      thickness=0.5));
  connect(HPpump.C_in, Pi_FeedWaterPump_sensor.C_out) annotation (Line(
      points={{80,-76},{84,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_FeedWaterPump_sensor.C_in, HPpump.C_out) annotation (Line(
      points={{52,-76},{60,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_SuperheaterDrains.C_out, Pc_Superheater_sensor.C_in) annotation (Line(
      points={{202,6},{202,-2}},
      color={28,108,200},
      thickness=0.5));
  connect(Pc_Superheater_sensor.C_out, HPReheater_valve.C_in) annotation (
      Line(
      points={{202,-14},{202,-20.1818},{48,-20.1818}},
      color={28,108,200},
      thickness=0.5));
  connect(Qo_SteamDryer_sensor.C_in, steamDryer.C_hot_liquid) annotation (Line(
      points={{154,22},{154,24.1},{154,24.1},{154,28}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_DryerCondensats.C_in, Qo_SteamDryer_sensor.C_out)
    annotation (Line(
      points={{154,4},{154,10}},
      color={28,108,200},
      thickness=0.5));
  connect(Pi_LowPressureTurbine_sensor.C_out, Ti_LowPressureTurbine_sensor.C_in)
    annotation (Line(
      points={{186,82},{190,82}},
      color={238,46,47},
      thickness=0.5));
  connect(Ti_LowPressureTurbine_sensor.C_out, LowPressureTurbine_1.C_in)
    annotation (Line(
      points={{202,82},{208,82}},
      color={238,46,47},
      thickness=0.5));
  connect(PressureLoss_before_drum.C_in, To_LowPressureReheater_sensor.C_in)
    annotation (Line(
      points={{192,-76},{204,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(steamDryerValve.C_out, PressureLoss_before_drum.C_out) annotation (
      Line(points={{153.818,-36},{153.818,-76},{172,-76}},     color={28,108,
          200},
      thickness=0.5));
  connect(HPReheater.C_cold_out, To_HighPressureReheater_sensor.C_out)
    annotation (Line(
      points={{-10,-76},{-18,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Tc_HighPressureReheater_sensor.C_in, HPReheater.C_hot_out)
    annotation (Line(
      points={{46,-122.182},{6,-122.182},{6,-84}},
      color={28,108,200},
      thickness=0.5));
  connect(To_HighPressureReheater_sensor.C_in, Qi_SteamGenerator_sensor.C_in)
    annotation (Line(
      points={{-30,-76},{-38,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Qi_SteamGenerator_sensor.C_out, Pi_SteamGenerator_sensor.C_in)
    annotation (Line(
      points={{-50,-76},{-58,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_Condenser.C_out, LPpump.C_in) annotation (Line(points={{380,-70},{382,-70},{382,-76},{358,-76}}, color={28,108,200}));
  connect(SuperHeaterControlValve.C_in, HPControlValve.C_in) annotation (Line(points={{30,57.8182},{-6,57.8182},{-6,56},{-72,56},{-72,34},{-62,34},{-62,34.1818}}, color={28,108,200}));
  connect(Tc_HighPressureReheater_sensor.C_out, HPReheaterControlValve.C_in) annotation (Line(points={{58,-122.182},{76,-122.182},{76,-122.182},{92,-122.182}}, color={28,108,200}));
  connect(Po_Condenser_sensor.C_out, PressureLoss_Condenser.C_in) annotation (Line(points={{380,-32},{380,-50}}, color={28,108,200}));
  connect(Pi_FeedWaterPump_sensor.C_in, PressureLoss_after_drum.C_out) annotation (Line(points={{96,-76},{116,-76}},                     color={28,108,200}));
  connect(HPReheater.C_cold_in, Po_FeedWaterPump_sensor.C_out) annotation (Line(points={{22.2,-76},{40,-76}}, color={28,108,200}));
  connect(To_LowPressureReheater_sensor.C_out, LPReheater.C_cold_out) annotation (Line(points={{216,-76},{230,-76}}, color={28,108,200}));
  connect(PressureLoss_ExtractionBeforeDryer.C_in, SteamExtraction_BeforeDryer.C_ext_out) annotation (Line(points={{72,4},{72,27.2}}, color={28,108,200}));
  connect(waterSource.C_out, HPControlValve.C_in) annotation (Line(points={{-105,34},{-62,34},{-62,34.1818}}, color={28,108,200}));
  connect(coldSource.C_out, condenser.C_cold_in) annotation (Line(points={{354,0},{362,0},{362,-0.111111},{366.48,-0.111111}}, color={28,108,200}));
  connect(LPReheater.C_hot_out, LPReheaterControlValve.C_in) annotation (Line(points={{246,-84},{248,-84},{248,-122.182},{318,-122.182}}, color={28,108,200}));
  annotation (Diagram(coordinateSystem(extent={{-120,-140},{480,140}})), Icon(coordinateSystem(extent={{-120,-140},{480,140}})));
end MetroscopiaNPP_reverse_MML2;
