within MetroscopeModelingLibrary.Tests.SimpleExamples.PowerPlant.MetroscopiaNPP;
model MetroscopiaNPP_reverse
  extends
    MetroscopeModelingLibrary.Tests.SimpleExamples.PowerPlant.MetroscopiaNPP.MetroscopiaNPP_topological;
  /* ----- Boundary Conditions ------ */
  input Real liquidFractionSG(start = 0) "%";
  input Real PressureSG(start = 50) "bar";
  input Real PressureCS(start = 3) "bar";
  input Real TemperatureCS(start = 15) "°C";
  input Real VolumeFlowRateCS(start = 50) "m3/s";
  input Real ThermalPower(start = 1880) "MWth";

  //Inputs for calibration
  input Real Po_Condenser(start=69.7982);
  input Real Po_LPSteamExtraction(start=5.0001);
  input Real Pi_LowPressureTurbine(start=18.8965);
  input Real Po_HighPressureTurbine(start=19.3986);
  input Real Po_HPSteamExtraction(start=31.0032);
  input Real Pi_HighPressureTurbine(start=48.5305);
  input Real Po_WaterSuctionPump(start=20.7963);
  input Real Po_FeedWaterTank(start=18.2933);
  input Real Po_ValveWaterSuctionPump(start=20.5589);
  input Real Pi_FeedWaterPump(start=18.3192);
  input Real Po_FeedWaterPump(start=59.464);
  input Real Pi_SteamGenerator(start=57.782);
  input Real Pc_Superheater(start=40.3773);
  input Real Qo_SteamDryer(start=45.7395);
  input Real Ti_LowPressureTurbine(start=228.021);
  input Real ActivePower(start=568.78);
  input Real To_LowPressureReheater(start=72.8234);
  input Real To_HighPressureReheater(start=218.034);
  input Real Tc_HighPressureReheater(start=216.711);

  // Observables
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
  output Real HPReheater_Kfr_cold;
  output Real SuperHeaterControlValve_Cvmax;
  output Real HighPressureTurbine_2_eta_is;
  output Real HighPressureTurbine_1_eta_is;
  output Real Superheater_Kth;
  output Real LowPressureTurbine_2_eta_is(start=0.859751);
  output Real LowPressureTurbine_1_eta_is(start=0.859751);
  output Real LPReheater_Kth;
  output Real HPReheater_Kth_cond;
  output Real HPReheater_Kth_purge;



equation
  /* ----- Boundary Conditions ------ */
  steamGenerator.ThermalPower = ThermalPower*1e6;
  steamGenerator.VaporFraction = 1 -liquidFractionSG/100;
  steamGenerator.steamSource.P_out = PressureSG*1e5;
  coldSource.P_out = PressureCS*1e5;
  coldSource.Qv_out = -VolumeFlowRateCS;
  coldSource.T_vol = TemperatureCS+273.15;

  /* ---- Observables ----- */
  ActivePower = generator.Welec/1e6;
  Qo_SteamDryer = steamDryer.liquidSide.Q_in;
  Qo_WaterSuctionPump = LPpump.Q_in;
  Qo_FeedWaterPump = HPpump.Q_in;
  Qi_SteamGenerator =steamGenerator.waterSupplySink.Q_in;
  Qi_Superheater = SuperHeaterControlValve.Q_in;
  Qc_HighPressureReheater = HPCondReheaterControlValve.Q_in;
  Pi_HighPressureTurbine = HighPressureTurbine_1.P_in/1e5;
  Po_HPSteamExtraction = PressureLoss_SteamExtractionHP.P_in/1e5;
  Po_HighPressureTurbine = PressureLoss_ExhaustHP.P_in/1e5;
  Po_FeedWaterTank = PressureLoss_DryerCondensats1.P_out/1e5;
  Po_Condenser = condenser.Psat/1e5*1e3;
  Po_FeedWaterPump = HPpump.P_out/1e5;
  Pi_SteamGenerator =steamGenerator.waterSupplySink.P_in/1e5;
  To_Superheater = LowPressureTurbine_1.T_in - 273.15;
  To_Condenser = PressureLoss_Condenser.T_in - 273.15;
  To_LowPressureReheater = PressureLoss_before_drum.T_in - 273.15;
  To_FeedWaterTank = PressureLoss_after_drum.T_out-273.15;
  Ti_HighPressureReheater = HPpump.T_out - 273.15;
  To_HighPressureReheater =steamGenerator.waterSupplySink.T_in - 273.15;
  Tc_HighPressureReheater = HPCondReheaterControlValve.T_in -273.15;
  Ti_SteamGenerator =steamGenerator.waterSupplySink.T_in - 273.15;
  Tc_LowPressureReheater = LPCondReheaterControlValve.T_in - 273.15;
  Ti_LowPressureTurbine = LowPressureTurbine_1.T_in - 273.15;
  Pi_LowPressureTurbine = LowPressureTurbine_1.P_in/1e5;
  Po_LPSteamExtraction = PressureLoss_DryerCondensats2.P_in/1e5;
  Po_WaterSuctionPump = LPpump.P_out/1e5;
  Po_ValveWaterSuctionPump = PumpControlValve.P_out/1e5;
  Qc_HPSteamExtractiontoFeedWaterTank = PressureLoss_DryerCondensats1.Q_in;
  Po_LowPressureReheater = PressureLoss_before_drum.P_in/1e5;
  Pi_FeedWaterPump = HPpump.P_in/1e5;
  Pc_Superheater = PressureLoss_SteamExtractionHP1.P_in/1e5;

  /* ---- Parameters to calibrate ----- */
  condenser_Kth = condenser.Kth;
  LowPressureTurbine_2_Cst = LowPressureTurbine_2.Cst;
  LowPressureTurbine_1_Cst = LowPressureTurbine_1.Cst;
  Superheater_Kfr_cold=Superheater.Kfr_cold;
  HighPressureTurbine_2_Cst=HighPressureTurbine_2.Cst;
  HighPressureTurbine_1_Cst=HighPressureTurbine_1.Cst;
  LPpump_Qv = LPpump.Qv;
  LPpump_hn = LPpump.hn;
  PressureLoss_DryerCondensats1_Kfr = PressureLoss_DryerCondensats1.Kfr;
  LPReheater_Kfr_cold = LPReheater.Kfr_cold;
  PressureLoss_after_drum_Kfr = PressureLoss_after_drum.Kfr;
  PressureLoss_after_drum_Q = PressureLoss_after_drum.Q_in;
  PressureLoss_after_drum_rhom = PressureLoss_after_drum.rhom;
  HPpump_Qv = HPpump.Qv;
  HPpump_hn = HPpump.hn;
  HPReheater_Kfr_cold = HPReheater.Kfr_cold;
  SuperHeaterControlValve_Cvmax = SuperHeaterControlValve.Cvmax;
  HighPressureTurbine_2_eta_is = HighPressureTurbine_2.eta_is;
  HighPressureTurbine_1_eta_is = HighPressureTurbine_1.eta_is;
  Superheater_Kth=Superheater.Kth;
  LPReheater_Kth = LPReheater.Kth;
  HPReheater_Kth_cond = HPReheater.Kth_cond;
  HPReheater_Kth_purge = HPReheater.Kth_purge;
  LowPressureTurbine_2_eta_is = LowPressureTurbine_2.eta_is;
  LowPressureTurbine_1_eta_is = LowPressureTurbine_1.eta_is;


  /* ---- Hypothesis ----- */
  HighPressureTurbine_1_eta_is = HighPressureTurbine_2_eta_is;
  LowPressureTurbine_1_eta_is = LowPressureTurbine_2_eta_is;


  generator.eta = 0.98;
  SteamExtraction_Tank.alpha = 1;
  SteamExtraction_HP.alpha = 1;
  SteamExtraction_LP.alpha = 1;
  steamDryer.x = 0.99;
  PressureLoss_DryerCondensats2.Kfr = 1e-3;
  PressureLoss_SteamExtractionHP.Kfr = 1e-3;
  LPpump.b3 = 0.7;
  HPpump.b3 = 0.7;
  PressureLoss_before_drum.Kfr = 1e-3;
  PressureLoss_before_drum.z1 = 0;
  PressureLoss_before_drum.z2 = 0;
  PressureLoss_after_drum.z1 = 0;
  PressureLoss_after_drum.z2 = 0;



  /* ---- Parameters ----- */
  // Valves
  SuperHeaterControlValve.Opening= 1;

  // Regulating Valves
  PumpControlValve.Cvmax = 1e4;
  steamDryerValve.Cvmax = 1e4;
  LPCondReheaterControlValve.Cvmax = 1e4;
  HPCondReheaterControlValve.Cvmax = 1e4;
  HPCondReheater_valve.Cvmax = 1e4;
  HPcontrolValve.Cvmax = 1e4;
  // Turbines
  HighPressureTurbine_1.eta_nz = 1;
  HighPressureTurbine_1.area_nz = 1;


  HighPressureTurbine_2.eta_nz = 1;
  HighPressureTurbine_2.area_nz = 1;


  LowPressureTurbine_1.eta_nz = 1;
  LowPressureTurbine_1.area_nz = 1;


  LowPressureTurbine_2.eta_nz = 1;
  LowPressureTurbine_2.area_nz = 1;


  // Heat Exchangers
  HPReheater.S_tot = 100;
  HPReheater.Level = 0.3;


  HPReheater.Kfr_hot = 1;
  LPReheater.S_tot = 100;


  LPReheater.Kfr_hot = 1;
  Superheater.S_tot = 100;


  Superheater.Kfr_hot=1;
  // Pumps
  LPpump.VRot = 1400;
  LPpump.VRotn = 1400;
  LPpump.rm = 0.85;
  LPpump.a1 = -78.0237;
  LPpump.a2 = 0;
  //LPpump.a3 =250;
  LPpump.b1 = 0;
  LPpump.b2 = 0;

  LPpump.rhmin = 0.20;
  HPpump.VRot = 1400;
  HPpump.VRotn = 1400;
  HPpump.rm = 0.85;
  HPpump.a1 =-76.0123;
  HPpump.a2 =0;
  //HPpump.a3 =596;
  HPpump.b1= 0;
  HPpump.b2 = 0;

  HPpump.rhmin =0.20;
  // Condenser

  condenser.Kfr_cold=0.00600135;
  condenser.S = 100;
  condenser.WaterHeight = 0;
  condenser.C_incond = 0;
  condenser.P_offset = 0;
  // Steam Extractions and dryers


  // Generator

  // Pipes
  PressureLoss_SteamExtractionHP1.z1 = 0;
  PressureLoss_SteamExtractionHP1.z2 = 0;
  PressureLoss_SteamExtractionHP1.Kfr = 1e-6;
  PressureLoss_SteamExtractionHP.z1 = 0;
  PressureLoss_SteamExtractionHP.z2 = 0;

  PressureLoss_DryerCondensats1.z1 = 0;
  PressureLoss_DryerCondensats1.z2 = 0;

  PressureLoss_ExhaustHP.z1 = 0;
  PressureLoss_ExhaustHP.z2 = 0;
  PressureLoss_ExhaustHP.Kfr = 1e-6;
  PressureLoss_DryerCondensats.z1 = 0;
  PressureLoss_DryerCondensats.z2 = 0;
  PressureLoss_DryerCondensats.Kfr = 1e-6;

  PressureLoss_DryerCondensats2.z1 = 0;
  PressureLoss_DryerCondensats2.z2 = 0;

  PressureLoss_Condenser.z1 = 0;
  PressureLoss_Condenser.z2 = 0;
  PressureLoss_Condenser.Kfr = 1e-6;
  PressureLoss_SteamExtractionHP2.z1 = 0;
  PressureLoss_SteamExtractionHP2.z2 = 0;
  PressureLoss_SteamExtractionHP2.Kfr = 1e-6;
  // Sink
  sinkBlowOff.h_vol = 1e4;
  sinkBlowOff.P_in = 50e5;
  sinkBlowOff.Q_in = 1e-2;
  sinkVent.Q_in = 0.01;
  sinkVent.h_vol = 1.2e6;
  coldSink.h_vol =1e4;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end MetroscopiaNPP_reverse;
