within MetroscopeModelingLibrary.Examples.NuclearPowerPlant;
model MetroscopiaNPP_reverse_MML2
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  // Boundary Conditions
  input Real liquidFractionSG(start = 0) "%";
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
  output Real Superheater_Kfr_cold;
  output Real HighPressureTurbine_2_Cst;
  output Real HighPressureTurbine_1_Cst;
  output Real LPpump_hn;
  output Real LPpump_Qv;
  output Real PressureLoss_DryerCondensats1_Kfr;
  output Real LPReheater_Kfr_cold;
  output Real PressureLoss_after_drum_Kfr;
  output Real PressureLoss_after_drum_Q;
  output Real PressureLoss_after_drum_rhom;
  output Real HPpump_hn;
  output Real HPpump_Qv;
  output Real HighPressureTurbine_2_eta_is;
  output Real HighPressureTurbine_1_eta_is;
  output Real LowPressureTurbine_2_eta_is;
  output Real LowPressureTurbine_1_eta_is;
  output Real SuperHeaterControlValve_Cvmax;
  output Real Superheater_Kth;
  output Real LPReheater_Kth;
  output Real HPReheater_Kfr_cold;
  output Real HPReheater_Kth_cond;
  output Real HPReheater_Kth_subc;

  // Observables i/o components
  output Real Qo_WaterSuctionPump;
  output Real Qo_FeedWaterPump;
  output Real Qi_SteamGenerator;
  output Real Qi_Superheater;
  output Real Qc_HighPressureReheater;
  output Real To_Superheater;
  output Real To_Condenser;
  output Real To_FeedWaterTank;
  output Real Ti_HighPressureReheater;
  output Real Ti_SteamGenerator;
  output Real Tc_LowPressureReheater;
  output Real Qc_HPSteamExtractiontoFeedWaterTank;
  output Real Po_LowPressureReheater;

  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.SteamGenerator
    steamGenerator
    annotation (Placement(transformation(extent={{-118,-102},{-88,-50}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterControlValve
    HPControlValve
    annotation (Placement(transformation(extent={{-62,36},{-52,26}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.WaterSink sinkBlowOff
    annotation (Placement(transformation(extent={{-108,-126},{-116,-118}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine
    HighPressureTurbine_1
    annotation (Placement(transformation(extent={{-30,24},{-10,44}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterPipe
    PressureLoss_SteamExtractionHP annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={6,10})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter
    SteamExtraction_HP
    annotation (Placement(transformation(extent={{-4,28},{16,36}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine
    HighPressureTurbine_2
    annotation (Placement(transformation(extent={{28,22},{48,42}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter SteamExtraction_BeforeDryer annotation (Placement(transformation(extent={{62,28},{82,36}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterPipe PressureLoss_ExtractionBeforeDryer
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={72,-6})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterPipe PressureLoss_ExhaustHP
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={98,34})));
  MetroscopeModelingLibrary.WaterSteam.Volumes.SteamDryer steamDryer
    annotation (Placement(transformation(extent={{132,22},{154,44}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterPipe PressureLoss_DryerCondensats
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={154,-6})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.SuperHeater Superheater
    annotation (Placement(transformation(extent={{152,50},{184,66}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.WaterSink sinkVent
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=90,
        origin={189,33})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterControlValve SuperHeaterControlValve
    annotation (Placement(transformation(extent={{30,56},{40,66}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine LowPressureTurbine_1
    annotation (Placement(transformation(extent={{208,72},{228,92}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter SteamExtraction_LP
    annotation (Placement(transformation(extent={{240,78},{260,86}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterPipe PressureLoss_LPExtraction annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={250,22})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine LowPressureTurbine_2
    annotation (Placement(transformation(extent={{280,72},{300,92}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.WaterSource coldSource
    annotation (Placement(transformation(extent={{300,-8},{316,8}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.WaterSink coldSink
    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=0,
        origin={423,-7})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterPipe PressureLoss_Condenser
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={380,-60})));
  MetroscopeModelingLibrary.WaterSteam.Machines.WaterPump LPpump
    annotation (Placement(transformation(extent={{348,-86},{328,-66}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.DryReheater LPReheater
    annotation (Placement(transformation(extent={{262,-84},{230,-68}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterControlValve PumpControlValve
    annotation (Placement(transformation(extent={{298,-78},{288,-68}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterControlValve steamDryerValve
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=-90,
        origin={157,-31})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterControlValve LPReheaterControlValve
    annotation (Placement(transformation(extent={{318,-124},{328,-114}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.WaterPump HPpump
    annotation (Placement(transformation(extent={{70,-86},{50,-66}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterPipe PressureLoss_before_drum
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={182,-76})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterPipe PressureLoss_after_drum
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={126,-76})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterPipe PressureLoss_SuperheaterDrains
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={202,16})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterPipe PressureLoss_MainSteamExtraction
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={6,-46})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater HPReheater
    annotation (Placement(transformation(extent={{22,-84},{-10,-68}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterControlValve HPReheaterControlValve annotation (Placement(transformation(extent={{92,-124},{102,-114}})));
  Power.Machines.Generator generator
    annotation (Placement(transformation(extent={{334,112},{382,140}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.WaterControlValve HPReheater_valve
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=0,
        origin={43,-17})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser
    annotation (Placement(transformation(extent={{367,-16},{393,6}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.PowerSink sink
    annotation (Placement(transformation(extent={{436,116},{458,136}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.PowerSource source
    annotation (Placement(transformation(extent={{44,-60},{54,-50}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.PowerSource source1
    annotation (Placement(transformation(extent={{326,-60},{336,-50}})));

  // Sensors
  MetroscopeModelingLibrary.Sensors.Power.PowerSensor ActivePower_sensor
    annotation (Placement(transformation(extent={{400,119},{414,133}})));
  MetroscopeModelingLibrary.Sensors.Other.OpeningSensor openingSensor
    annotation (Placement(transformation(
        extent={{4.5,-4.5},{-4.5,4.5}},
        rotation=180,
        origin={-57,14.5})));
  MetroscopeModelingLibrary.Sensors.Other.OpeningSensor openingSensor1
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=0,
        origin={35,82.5})));
  MetroscopeModelingLibrary.Sensors.Other.OpeningSensor openingSensor2
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=0,
        origin={43,2.5})));
  MetroscopeModelingLibrary.Sensors.Other.OpeningSensor openingSensor3
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=0,
        origin={293,-55.5})));
  MetroscopeModelingLibrary.Sensors.Other.OpeningSensor openingSensor4
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=-90,
        origin={177,-31.5})));
  MetroscopeModelingLibrary.Sensors.Other.OpeningSensor openingSensor5
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=0,
        origin={97,-103.5})));
  MetroscopeModelingLibrary.Sensors.Other.OpeningSensor openingSensor6
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=0,
        origin={323,-105.5})));
  MetroscopeModelingLibrary.Sensors.Other.VRotSensor rotSpeedSensor
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={70,-98})));
  MetroscopeModelingLibrary.Sensors.Other.VRotSensor rotSpeedSensor1
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},
        rotation=270,
        origin={352,-100})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor
    PressureSG_sensor
    annotation (Placement(transformation(extent={{-88,28},{-76,40}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor
    PressureCS_sensor
    annotation (Placement(transformation(extent={{318,-6},{330,6}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor
    TemperatureCS_sensor
    annotation (Placement(transformation(extent={{334,-6},{346,6}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor VolumeFlowRateCS_sensor
    "volumic sensor"
    annotation (Placement(transformation(extent={{348,-6},{360,6}})));
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
    annotation (Placement(transformation(extent={{318,-82},{306,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor Po_FeedWaterTank_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={72,-30})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor Po_ValveWaterSuctionPump_sensor
    annotation (Placement(transformation(extent={{282,-82},{270,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor Pi_FeedWaterPump_sensor
    annotation (Placement(transformation(extent={{86,-82},{74,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor Po_FeedWaterPump_sensor
    annotation (Placement(transformation(extent={{50,-82},{38,-70}})));
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
    annotation (Placement(transformation(extent={{194,-82},{206,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor To_HighPressureReheater_sensor
    "same value as Ti_SteamGenerator"
    annotation (Placement(transformation(extent={{-30,-82},{-18,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor Tc_HighPressureReheater_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={16,-122.182})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor Qo_WaterSuctionPump_sensor
    annotation (Placement(transformation(extent={{368,-82},{356,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor Qo_FeedWaterPump_sensor
    annotation (Placement(transformation(extent={{100,-82},{88,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor Qi_SteamGenerator_sensor
    annotation (Placement(transformation(extent={{-38,-82},{-50,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor Qi_Superheater_sensor
    annotation (Placement(transformation(extent={{-38,51.8182},{-26,63.8182}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor Qc_HighPressureReheater_sensor
    annotation (Placement(transformation(extent={{38,-128},{50,-116}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor To_Condenser_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={380,-40})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor To_FeedWaterTank_sensor
    annotation (Placement(transformation(extent={{112,-82},{100,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor Ti_HighPressureReheater_sensor
    annotation (Placement(transformation(extent={{38,-82},{26,-70}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor Tc_LowPressureReheater_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={262,-122.182})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor Qc_HPSteamExtractiontoFeedWaterTank_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={72,16})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor Po_LowPressureReheater_sensor
    annotation (Placement(transformation(extent={{224,-82},{212,-70}})));
equation
  // ----- Boundary Conditions ------
  steamGenerator.thermal_power = ThermalPower*1e6;
  steamGenerator.vapor_fraction = 1 - liquidFractionSG/100;
  PressureSG_sensor.P_barA = PressureSG;
  PressureCS_sensor.P_barA = PressureCS;
  TemperatureCS_sensor.T_degC = TemperatureCS;
  coldSource.Qv_out = -VolumeFlowRateCS;
  //VolumeFlowRateCS_sensor.Qv = VolumeFlowRateCS;

  // ----- Components ------
  // SteamGenerator
  Qi_SteamGenerator = Qi_SteamGenerator_sensor.Q;
  Pi_SteamGenerator = Pi_SteamGenerator_sensor.P_barA;

  // HighPressureTurbines
    // HPControlValve
    HPControlValve.Cvmax = 1e4;

    // Observables used for calibration (inlet)
    Pi_HighPressureTurbine = Pi_HighPressureTurbine_sensor.P_barA;

    // HighPressureTurbine_1
      // Calibrated parameters
      HighPressureTurbine_1_Cst = HighPressureTurbine_1.Cst;
      HighPressureTurbine_1_eta_is = HighPressureTurbine_1.eta_is;

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
      HighPressureTurbine_2_eta_is = HighPressureTurbine_2.eta_is;

      // Fixed parameter
      HighPressureTurbine_2.eta_nz = 1;
      HighPressureTurbine_2.area_nz = 1;

    // Observables used for calibration (Outlet)
    Po_HighPressureTurbine = Po_HighPressureTurbine_sensor.P_barA;

    // Hypothesis
    HighPressureTurbine_1_eta_is = HighPressureTurbine_2_eta_is;
    LowPressureTurbine_1_eta_is = LowPressureTurbine_2_eta_is;

  // Steam extraction before dryer
    // Fixed parameter
    SteamExtraction_BeforeDryer.alpha = 1;

    // Pressure Loss
    PressureLoss_DryerCondensats1_Kfr = PressureLoss_ExtractionBeforeDryer.Kfr;
    PressureLoss_ExtractionBeforeDryer.delta_z = 0;

  // Steam Dryer
    // Inlet pressure loss
    PressureLoss_ExhaustHP.delta_z = 0;
    PressureLoss_ExhaustHP.Kfr = 1e-6;

    // Fixed parameter
    steamDryer.x_steam_out = 0.99;

    // Observable used for calibration
    Qo_SteamDryer = Qo_SteamDryer_sensor.Q;

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
    Superheater_Kfr_cold = Superheater.Kfr_cold;
    Superheater_Kth = Superheater.Kth;

    // Fixed parameter
    Superheater.S = 100;
    Superheater.Kfr_hot = 1;

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
      LowPressureTurbine_1_eta_is = LowPressureTurbine_1.eta_is;

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
      LowPressureTurbine_2_eta_is = LowPressureTurbine_2.eta_is;

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
      LPpump_Qv = LPpump.Qv_in;
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
    Tc_LowPressureReheater = Tc_LowPressureReheater_sensor.T_degC;

  // Drum
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
    HPpump_Qv = HPpump.Qv_in;
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

  // I/O observables
  Qc_HPSteamExtractiontoFeedWaterTank = Qc_HPSteamExtractiontoFeedWaterTank_sensor.Q_in;
  Qi_Superheater = Qi_Superheater_sensor.Q;
  Qo_WaterSuctionPump = Qo_WaterSuctionPump_sensor.Q;
  Qo_FeedWaterPump = Qo_FeedWaterPump_sensor.Q;
  Qc_HighPressureReheater = Qc_HighPressureReheater_sensor.Q;
  To_Condenser = To_Condenser_sensor.T_degC;
  To_FeedWaterTank = To_FeedWaterTank_sensor.T_degC;
  Ti_HighPressureReheater = Ti_HighPressureReheater_sensor.T_degC;
  Po_LowPressureReheater = Po_LowPressureReheater_sensor.P_barA;

  To_Superheater = LowPressureTurbine_1.T_in - 273.15;
  Ti_SteamGenerator = steamGenerator.feedwater_sink.T_in - 273.15;

  connect(sinkBlowOff.C_in, steamGenerator.purge_outlet) annotation (Line(points={{-110,-122},{-104,-122},{-104,-108},{-103,-108},{-103,-101.567}},
                                                   color={28,108,200},
      thickness=0.5));
  connect(HighPressureTurbine_1.C_out, SteamExtraction_HP.C_in)
    annotation (Line(points={{-10,34},{-8,34},{-8,32.4444},{-4.6,32.4444}},
                                                   color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_HP.C_ext_out, PressureLoss_SteamExtractionHP.C_in)
    annotation (Line(points={{6,29.4222},{6,20}},
                                             color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_HP.C_main_out, HighPressureTurbine_2.C_in)
    annotation (Line(points={{16.6,32.4444},{22,32.4444},{22,32},{28,32}},
                                                 color={238,46,47},
      thickness=0.5));
  connect(HighPressureTurbine_2.C_out, SteamExtraction_BeforeDryer.C_in) annotation (Line(
      points={{48,32},{54,32},{54,32.4444},{61.4,32.4444}},
      color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_BeforeDryer.C_main_out, PressureLoss_ExhaustHP.C_in) annotation (Line(
      points={{82.6,32.4444},{86,32.4444},{86,34},{88,34}},
      color={238,46,47},
      thickness=0.5));
  connect(steamDryer.C_hot_steam, Superheater.C_cold_in)
    annotation (Line(points={{154,36},{168,36},{168,50}}, color={238,46,47},
      thickness=0.5));
  connect(Superheater.C_hot_in, SuperHeaterControlValve.C_out) annotation (Line(
        points={{152,58.2},{140,58.2},{140,58},{40,58},{40,57.8182}},     color={238,46,
          47},
      thickness=0.5));
  connect(Superheater.C_vent, sinkVent.C_in) annotation (Line(points={{184,50.2},{188,50.2},{188,35.5},{189,35.5}},
                                       color={28,108,200},
      thickness=0.5));
  connect(LowPressureTurbine_1.C_out, SteamExtraction_LP.C_in)
    annotation (Line(points={{228,82},{234,82},{234,82.4444},{239.4,82.4444}},
                                                     color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_LP.C_main_out, LowPressureTurbine_2.C_in)
    annotation (Line(points={{260.6,82.4444},{270,82.4444},{270,82},{280,82}},
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
  connect(condenser.C_cold_out, coldSink.C_in) annotation (Line(points={{393,-6.95556},{402,-6.95556},{402,-7},{419.5,-7}},
                                             color={0,140,72},
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
    annotation (Line(points={{60,-65.2},{60,-55},{51.4,-55}},
                                                            color={255,128,0},
      pattern=LinePattern.Dash));
  connect(LPpump.C_power, source1.C_out) annotation (Line(points={{338,-65.2},{338,-55},{333.4,-55}},
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
  connect(HPControlValve.Opening, openingSensor.Opening)
    annotation (Line(points={{-57,26.9091},{-57,20},{-57,19.09},{-57,19.09}},
                                                         color={0,0,127}));
  connect(SuperHeaterControlValve.Opening, openingSensor1.Opening)
    annotation (Line(points={{35,65.0909},{35,77.91}}, color={0,0,127}));
  connect(HPReheater_valve.Opening, openingSensor2.Opening)
    annotation (Line(points={{43,-12.9091},{43,-6},{43,-2.09},{43,-2.09}},
                                                        color={0,0,127}));
  connect(PumpControlValve.Opening, openingSensor3.Opening)
    annotation (Line(points={{293,-68.9091},{293,-62},{293,-60.09},{293,-60.09}},
                                                           color={0,0,127}));
  connect(steamDryerValve.Opening, openingSensor4.Opening) annotation (Line(
        points={{161.091,-31},{163.115,-31},{163.115,-31.5},{172.41,-31.5}},
        color={0,0,127}));
  connect(HPReheaterControlValve.Opening, openingSensor5.Opening) annotation (Line(points={{97,-114.909},{97,-114.909},{97,-108.09},{97,-108.09}}, color={0,0,127}));
  connect(LPReheaterControlValve.Opening, openingSensor6.Opening)
    annotation (Line(points={{323,-114.909},{323,-114.909},{323,-110.09},{323,-110.09}},
                                                            color={0,0,127}));
  connect(HPpump.VRot, rotSpeedSensor.VRot)
    annotation (Line(points={{60,-88},{60,-98},{63.88,-98}}, color={0,0,127}));
  connect(LPpump.VRot, rotSpeedSensor1.VRot) annotation (Line(points={{338,-88},{338,-100},{345.88,-100}},
                                   color={0,0,127}));
  connect(steamGenerator.steam_outlet, PressureSG_sensor.C_in) annotation (Line(
      points={{-103,-50},{-103,34},{-88,34}},
      color={238,46,47},
      thickness=0.5));
  connect(PressureSG_sensor.C_out, HPControlValve.C_in) annotation (Line(
      points={{-76,34},{-62,34},{-62,34.1818}},
      color={238,46,47},
      thickness=0.5));
  connect(coldSource.C_out, PressureCS_sensor.C_in)
    annotation (Line(points={{312,0},{318,0}}, color={0,140,72},
      thickness=0.5));
  connect(PressureCS_sensor.C_out, TemperatureCS_sensor.C_in)
    annotation (Line(points={{330,0},{334,0}},    color={0,140,72},
      thickness=0.5));
  connect(condenser.C_cold_in, VolumeFlowRateCS_sensor.C_out) annotation (Line(
        points={{366.48,-0.111111},{363.43,-0.111111},{363.43,0},{360,0}},
        color={0,140,72},
      thickness=0.5));
  connect(TemperatureCS_sensor.C_out, VolumeFlowRateCS_sensor.C_in)
    annotation (Line(points={{346,0},{348,0}},    color={0,140,72},
      thickness=0.5));
  connect(Po_Condenser_sensor.C_in, condenser.C_hot_out)
    annotation (Line(points={{380,-22},{380,-16.4889}}, color={28,108,200},
      thickness=0.5));
  connect(SteamExtraction_LP.C_ext_out, Po_LPSteamExtraction_sensor.C_in)
    annotation (Line(
      points={{250,79.4222},{250,62}},
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
      points={{298,-76.1818},{298,-76},{306,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_WaterSuctionPump_sensor.C_in, LPpump.C_out) annotation (Line(
      points={{318,-76},{328,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_ExtractionBeforeDryer.C_out, Po_FeedWaterTank_sensor.C_in) annotation (Line(
      points={{72,-16},{72,-24}},
      color={238,46,47},
      thickness=0.5));
  connect(LPReheater.C_cold_in, Po_ValveWaterSuctionPump_sensor.C_out)
    annotation (Line(
      points={{262.2,-76},{270,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_ValveWaterSuctionPump_sensor.C_in, PumpControlValve.C_out)
    annotation (Line(
      points={{282,-76},{286,-76},{286,-76.1818},{288,-76.1818}},
      color={28,108,200},
      thickness=0.5));
  connect(HPpump.C_in, Pi_FeedWaterPump_sensor.C_out) annotation (Line(
      points={{70,-76},{74,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_FeedWaterPump_sensor.C_in, HPpump.C_out) annotation (Line(
      points={{50,-76},{50,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Pi_SteamGenerator_sensor.C_out, steamGenerator.feedwater_inlet)
    annotation (Line(
      points={{-70,-76},{-95.5,-76}},
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
      points={{192,-76},{194,-76}},
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
      points={{10,-122.182},{6,-122.182},{6,-84}},
      color={28,108,200},
      thickness=0.5));
  connect(LPpump.C_in, Qo_WaterSuctionPump_sensor.C_out) annotation (Line(
      points={{348,-76},{356,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Qo_WaterSuctionPump_sensor.C_in, PressureLoss_Condenser.C_out)
    annotation (Line(
      points={{368,-76},{380,-76},{380,-70}},
      color={28,108,200},
      thickness=0.5));
  connect(Pi_FeedWaterPump_sensor.C_in, Qo_FeedWaterPump_sensor.C_out)
    annotation (Line(
      points={{86,-76},{88,-76}},
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
  connect(SuperHeaterControlValve.C_in, Qi_Superheater_sensor.C_out)
    annotation (Line(
      points={{30,57.8182},{2,57.8182},{2,57.8182},{-26,57.8182}},
      color={238,46,47},
      thickness=0.5));
  connect(Qi_Superheater_sensor.C_in, HPControlValve.C_in) annotation (Line(
      points={{-38,57.8182},{-68,57.8182},{-68,34},{-62,34},{-62,34.1818}},
      color={238,46,47},
      thickness=0.5));
  connect(HPReheaterControlValve.C_in, Qc_HighPressureReheater_sensor.C_out) annotation (Line(
      points={{92,-122.182},{50,-122.182},{50,-122}},
      color={28,108,200},
      thickness=0.5));
  connect(Qc_HighPressureReheater_sensor.C_in, Tc_HighPressureReheater_sensor.C_out)
    annotation (Line(
      points={{38,-122},{30.06,-122},{30.06,-122.182},{22,-122.182}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_Condenser.C_in, To_Condenser_sensor.C_out) annotation (
      Line(
      points={{380,-50},{380,-46}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_Condenser_sensor.C_out, To_Condenser_sensor.C_in) annotation (Line(
      points={{380,-32},{380,-34}},
      color={28,108,200},
      thickness=0.5));
  connect(Qo_FeedWaterPump_sensor.C_in, To_FeedWaterTank_sensor.C_out)
    annotation (Line(
      points={{100,-76},{100,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_after_drum.C_out, To_FeedWaterTank_sensor.C_in)
    annotation (Line(
      points={{116,-76},{112,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(HPReheater.C_cold_in, Ti_HighPressureReheater_sensor.C_out)
    annotation (Line(
      points={{22.2,-76},{26,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_FeedWaterPump_sensor.C_out, Ti_HighPressureReheater_sensor.C_in)
    annotation (Line(
      points={{38,-76},{38,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(LPReheaterControlValve.C_in, Tc_LowPressureReheater_sensor.C_out)
    annotation (Line(
      points={{318,-122.182},{294,-122.182},{294,-122.182},{268,-122.182}},
      color={28,108,200},
      thickness=0.5));
  connect(Tc_LowPressureReheater_sensor.C_in, LPReheater.C_hot_out) annotation (
     Line(
      points={{256,-122.182},{246,-122.182},{246,-84}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_ExtractionBeforeDryer.C_in, Qc_HPSteamExtractiontoFeedWaterTank_sensor.C_out) annotation (Line(
      points={{72,4},{72,10}},
      color={28,108,200},
      thickness=0.5));
  connect(Qc_HPSteamExtractiontoFeedWaterTank_sensor.C_in, SteamExtraction_BeforeDryer.C_ext_out) annotation (Line(
      points={{72,22},{72,29.4222}},
      color={28,108,200},
      thickness=0.5));
  connect(LPReheater.C_cold_out, Po_LowPressureReheater_sensor.C_in)
    annotation (Line(
      points={{230,-76},{224,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_LowPressureReheater_sensor.C_out, To_LowPressureReheater_sensor.C_out)
    annotation (Line(
      points={{212,-76},{206,-76}},
      color={28,108,200},
      thickness=0.5));
  annotation (Diagram(coordinateSystem(extent={{-120,-140},{480,140}})), Icon(coordinateSystem(extent={{-120,-140},{480,140}})));
end MetroscopiaNPP_reverse_MML2;
