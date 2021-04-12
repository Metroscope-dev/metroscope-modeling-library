within MetroscopeModelingLibrary.Tests.SimpleExamples.PowerPlant.MetroscopiaNPP;
model MetroscopiaNPP_direct
  extends
    MetroscopeModelingLibrary.Tests.SimpleExamples.PowerPlant.MetroscopiaNPP.MetroscopiaNPP_topological;
  /* ----- Boundary Conditions ------ */
  input Real liquidFractionSG(start = 0) "%";
  input Real PressureSG(start = 50) "bar";
  input Real PressureCS(start = 3) "bar";
  input Real TemperatureCS(start = 15) "°C";
  input Real VolumeFlowRateCS(start = 50) "m3/s";
  input Real ThermalPower(start = 1880) "MWth";
  // Observables
  output Real ActivePower;
  output Real Qo_SteamDryer;
  output Real Qo_WaterSuctionPump;
  output Real Qo_FeedWaterPump;
  output Real Qi_SteamGenerator;
  output Real Qi_Superheater;
  output Real Qc_HighPressureReheater;
  output Real Pi_HighPressureTurbine;
  output Real Po_HPSteamExtraction;
  output Real Po_HighPressureTurbine;
  output Real Po_FeedWaterTank;
  output Real Po_Condenser;
  output Real Po_FeedWaterPump;
  output Real Pi_SteamGenerator;
  output Real To_Superheater;
  output Real To_Condenser;
  output Real To_LowPressureReheater;
  output Real To_FeedWaterTank;
  output Real Ti_HighPressureReheater;
  output Real To_HighPressureReheater;
  output Real Tc_HighPressureReheater;
  output Real Ti_SteamGenerator;
  output Real Tc_LowPressureReheater;
  output Real Ti_LowPressureTurbine;
  output Real Pi_LowPressureTurbine;
  output Real Po_LPSteamExtraction;
  output Real Po_WaterSuctionPump;
  output Real Po_ValveWaterSuctionPump;
  output Real Qc_HPSteamExtractiontoFeedWaterTank;
  output Real Po_LowPressureReheater;
  output Real Pi_FeedWaterPump;
  output Real Pc_Superheater;

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


  /* ---- Parameters ----- */
  // Valves
  SuperHeaterControlValve.Opening= 1;
  SuperHeaterControlValve.Cvmax = 1248.76;
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
  HighPressureTurbine_1.Cst=26580.1;
  HighPressureTurbine_1.eta_is=0.780417;
  HighPressureTurbine_2.eta_nz = 1;
  HighPressureTurbine_2.area_nz = 1;
  HighPressureTurbine_2.Cst=12802.4;
  HighPressureTurbine_2.eta_is=0.958154;
  LowPressureTurbine_1.eta_nz = 1;
  LowPressureTurbine_1.area_nz = 1;
  LowPressureTurbine_1.Cst = 14109.6;
  LowPressureTurbine_1.eta_is = 0.881233;
  LowPressureTurbine_2.eta_nz = 1;
  LowPressureTurbine_2.area_nz = 1;
  LowPressureTurbine_2.Cst = 1538.88;
  LowPressureTurbine_2.eta_is = 0.859751;
  // Heat Exchangers
  HPReheater.S_tot = 100;
  HPReheater.Level = 0.3;
  HPReheater.Kth_cond = 25438.4;
  HPReheater.Kth_purge = 25438.4/3;
  HPReheater.Kfr_cold = 140;
  HPReheater.Kfr_hot = 1;
  LPReheater.S_tot = 100;
  LPReheater.Kth = 10178.4;
  LPReheater.Kfr_cold = 75;
  LPReheater.Kfr_hot = 1;
  Superheater.S_tot = 100;
  Superheater.Kth=11319.5;
  Superheater.Kfr_cold=1;//113.3;
  Superheater.Kfr_hot=1;
  // Pumps
  LPpump.VRot = 1400;
  LPpump.VRotn = 1400;
  LPpump.rm = 0.85;
  LPpump.a1 = -78.0237;
  LPpump.a2 = 0;
  LPpump.a3 =250;
  LPpump.b1 = 0;
  LPpump.b2 = 0;
  LPpump.b3 = 0.85;
  LPpump.rhmin = 0.20;
  HPpump.VRot = 1400;
  HPpump.VRotn = 1400;
  HPpump.rm = 0.85;
  HPpump.a1 =-76.0123;
  HPpump.a2 =0;
  HPpump.a3 =596;
  HPpump.b1= 0;
  HPpump.b2 = 0;
  HPpump.b3 = 0.85;
  HPpump.rhmin =0.20;
  // Condenser
  condenser.Kth = 630645;
  condenser.Kfr_cold=0.00600135;
  condenser.S = 100;
  condenser.WaterHeight = 0;
  // Steam Extractions and dryers
  SteamExtraction_Tank.alpha = 0.95;
  SteamExtraction_HP.alpha = 0.95;
  SteamExtraction_LP.alpha = 0.95;
  steamDryer.x = 1;
  // Generator
  generator.eta = 0.974;
  // Pipes
  PressureLoss_SteamExtractionHP1.z1 = 0;
  PressureLoss_SteamExtractionHP1.z2 = 0;
  PressureLoss_SteamExtractionHP1.Kfr = 1e-6;
  PressureLoss_SteamExtractionHP.z1 = 0;
  PressureLoss_SteamExtractionHP.z2 = 0;
  PressureLoss_SteamExtractionHP.Kfr = 1e-6;
  PressureLoss_DryerCondensats1.z1 = 0;
  PressureLoss_DryerCondensats1.z2 = 0;
  PressureLoss_DryerCondensats1.Kfr = 21.3;
  PressureLoss_ExhaustHP.z1 = 0;
  PressureLoss_ExhaustHP.z2 = 0;
  PressureLoss_ExhaustHP.Kfr = 1e-6;
  PressureLoss_DryerCondensats.z1 = 0;
  PressureLoss_DryerCondensats.z2 = 0;
  PressureLoss_DryerCondensats.Kfr = 1e-6;
  PressureLoss_DryerCondensats2.Kfr = 1e-6;
  PressureLoss_DryerCondensats2.z1 = 0;
  PressureLoss_DryerCondensats2.z2 = 0;
  PressureLoss_before_drum.Kfr = 98;
  PressureLoss_before_drum.z1 = 0;
  PressureLoss_before_drum.z2 = 15;
  PressureLoss_after_drum.Kfr = 103;
  PressureLoss_after_drum.z1 = 15;
  PressureLoss_after_drum.z2 = 0;
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
end MetroscopiaNPP_direct;
