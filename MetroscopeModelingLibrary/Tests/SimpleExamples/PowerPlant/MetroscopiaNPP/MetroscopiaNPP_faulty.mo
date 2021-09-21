within MetroscopeModelingLibrary.Tests.SimpleExamples.PowerPlant.MetroscopiaNPP;
model MetroscopiaNPP_faulty
  extends MetroscopiaNPP_direct(mode=2);

  input Real Failure_FTSH(start=0);
  input Real Failure_PSSH(start=0);
  input Real Failure_PSHPR(start=0);
  input Real Failure_FTHPR(start=0);
  input Real Failure_PSLPR(start=0);
  input Real Failure_FTLPR(start=0);
  input Real Failure_BPSG(start=0);
  input Real Failure_BPHPCV(start=0);
  input Real Failure_BPSHCV(start=0);
  input Real Failure_BPSEHP(start=0);
  input Real Failure_BPSETank(start=0);
  input Real Failure_BPSELP(start=0);
  input Real Failure_BPSD(start=0);
  input Real Failure_BPHPR(start=0);
  input Real Failure_BPSH(start=0);
  input Real Failure_HPR_Fouling(start=0);
  input Real Failure_LPR_Fouling(start=0);
  input Real Failure_SH_Fouling(start=0);
  input Real Failure_Cond_Fouling(start=0);
  input Real Failure_HPR_SSC(start=0);


  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    FuiteTub_SuperHeater
    annotation (Placement(transformation(extent={{134,78},{144,94}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    PlaqSep_SuperHeater
    annotation (Placement(transformation(extent={{134,92},{144,106}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    PlaqSep_HPReheater annotation (Placement(transformation(
        extent={{-6,-5},{6,5}},
        rotation=180,
        origin={34,-59})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    FuiteTub_HPReheater annotation (Placement(transformation(
        extent={{-6,-5},{6,5}},
        rotation=180,
        origin={34,-67})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    PlaqSep_LPReheater annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=180,
        origin={265,-47})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    FuiteTub_LPReheater annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=180,
        origin={265,-57})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    Bypass_SteamGen2Cond annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=-90,
        origin={-68,16})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    BypassHPCV2Cond annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=-90,
        origin={-40,16})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    Bypass_SHCV2Cond annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=-90,
        origin={42,56})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    Bypass_SEHP2Cond annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=-90,
        origin={24,16})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    Bypass_SETank2Cond annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=-90,
        origin={100,18})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    Bypass_SELP2Cond annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=-90,
        origin={272,60})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    Bypass_SD2Cond
    annotation (Placement(transformation(extent={{168,-2},{174,6}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    Bypass_HPR2Cond
    annotation (Placement(transformation(extent={{16,-118},{24,-108}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    Bypass_SH2Cond
    annotation (Placement(transformation(extent={{186,62},{194,72}})));
equation


  FuiteTub_SuperHeater.Q_in = 1e-3 + Failure_FTSH;
  PlaqSep_SuperHeater.Q_in = 1e-3 + Failure_PSSH;
  PlaqSep_HPReheater.Q_in = 1e-3 + Failure_PSHPR;
  FuiteTub_HPReheater.Q_in = 1e-3 + Failure_FTHPR;
  PlaqSep_LPReheater.Q_in = 1e-3 + Failure_PSLPR;
  FuiteTub_LPReheater.Q_in = 1e-3 + Failure_FTLPR;
  Bypass_SteamGen2Cond.Q_in = 1e-3 + Failure_BPSG;
  BypassHPCV2Cond.Q_in = 1e-3 + Failure_BPHPCV;
  Bypass_SHCV2Cond.Q_in = 1e-3 + Failure_BPSHCV;
  Bypass_SEHP2Cond.Q_in = 1e-3 + Failure_BPSEHP;
  Bypass_SETank2Cond.Q_in = 1e-3 + Failure_BPSETank;
  Bypass_SELP2Cond.Q_in = 1e-3 + Failure_BPSELP;
  Bypass_SD2Cond.Q_in = 1e-3 + Failure_BPSD;
  Bypass_HPR2Cond.Q_in = 1e-3 + Failure_BPHPR;
  Bypass_SH2Cond.Q_in = 1e-3 + Failure_BPSH;

  HPR_Fouling_Coef = Failure_HPR_Fouling/100;
  LPR_Fouling_Coef = Failure_LPR_Fouling/100;
  SH_Fouling_Coef = Failure_SH_Fouling/100;
  Cond_Fouling_Coef = Failure_Cond_Fouling/100;
  HPR_Subcooling_Surface_Change = Failure_HPR_SSC;


  connect(FuiteTub_SuperHeater.C_in, SuperHeaterControlValve.C_out) annotation (
     Line(points={{134,86},{94,86},{94,71.8182},{40.1,71.8182}}, color={217,67,180}));
  connect(PlaqSep_SuperHeater.C_in, SuperHeaterControlValve.C_out) annotation (
      Line(points={{134,99},{94,99},{94,71.8182},{40.1,71.8182}}, color={217,67,
          180}));
  connect(FuiteTub_SuperHeater.C_out, LowPressureTurbine_1.C_in) annotation (
      Line(points={{144.1,86},{198,86},{198,82},{208,82}}, color={217,67,180}));
  connect(PlaqSep_SuperHeater.C_out, PressureLoss_SteamExtractionHP1.C_in)
    annotation (Line(points={{144.1,99},{202,99},{202,26}}, color={217,67,180}));
  connect(HPpump.C_out, FuiteTub_HPReheater.C_in) annotation (Line(points={{55.8,
          -76},{46,-76},{46,-67},{40,-67}}, color={217,67,180}));
  connect(HPpump.C_out, PlaqSep_HPReheater.C_in) annotation (Line(points={{55.8,
          -76},{46,-76},{46,-59},{40,-59}}, color={217,67,180}));
  connect(FuiteTub_HPReheater.C_out, HPCondReheaterControlValve.C_in)
    annotation (Line(points={{27.88,-67},{26,-67},{26,-74},{44,-74},{44,
          -124.182},{92,-124.182}},
                          color={217,67,180}));
  connect(PlaqSep_HPReheater.C_out, steamGenerator.C_water_in) annotation (Line(
        points={{27.88,-59},{-27.06,-59},{-27.06,-15.6},{-83.28,-15.6}}, color={217,67,
          180}));
  connect(PumpControlValve.C_out, FuiteTub_LPReheater.C_in) annotation (Line(
        points={{283.9,-76.1818},{276,-76.1818},{276,-57},{270,-57}}, color={217,67,
          180}));
  connect(PumpControlValve.C_out, PlaqSep_LPReheater.C_in) annotation (Line(
        points={{283.9,-76.1818},{276,-76.1818},{276,-47},{270,-47}}, color={217,67,
          180}));
  connect(PlaqSep_LPReheater.C_out, PressureLoss_before_drum.C_in) annotation (
      Line(points={{259.9,-47},{208,-47},{208,-76},{198,-76}}, color={217,67,180}));
  connect(FuiteTub_LPReheater.C_out, LPCondReheaterControlValve.C_in)
    annotation (Line(points={{259.9,-57},{258,-57},{258,-66},{270,-66},{270,
          -126.182},{300,-126.182}},
                           color={217,67,180}));
  connect(steamGenerator.C_steam_out, Bypass_SteamGen2Cond.C_in) annotation (
      Line(points={{-98,18.42},{-98,34},{-68,34},{-68,20}}, color={217,67,180}));
  connect(HPcontrolValve.C_out, BypassHPCV2Cond.C_in) annotation (Line(points={{-41.9,
          33.8182},{-41.9,26.9091},{-40,26.9091},{-40,20}},       color={217,67,
          180}));
  connect(SteamExtraction_HP.C_ext_out, Bypass_SEHP2Cond.C_in)
    annotation (Line(points={{6,28},{6,20},{24,20}}, color={217,67,180}));
  connect(SuperHeaterControlValve.C_out, Bypass_SHCV2Cond.C_in) annotation (
      Line(points={{40.1,71.8182},{46,71.8182},{46,70},{54,70},{54,60},{42,60}},
        color={217,67,180}));
  connect(SteamExtraction_Tank.C_ext_out, Bypass_SETank2Cond.C_in)
    annotation (Line(points={{86,28},{86,22},{100,22}}, color={217,67,180}));
  connect(SteamExtraction_LP.C_ext_out, Bypass_SELP2Cond.C_in) annotation (Line(
        points={{250,76},{250,68},{266,68},{266,70},{272,70},{272,64}}, color={217,67,
          180}));
  connect(Bypass_SELP2Cond.C_out, condenser.C_hot_in) annotation (Line(points={{272,
          55.92},{272,18},{340,18},{340,14},{343,14},{343,6.24444}},   color={217,67,
          180}));
  connect(Bypass_SETank2Cond.C_out, condenser.C_hot_in) annotation (Line(points={{100,
          13.92},{142,13.92},{142,-2},{284,-2},{284,18},{340,18},{340,14},{343,14},
          {343,6.24444}},       color={217,67,180}));
  connect(Bypass_SHCV2Cond.C_out, condenser.C_hot_in) annotation (Line(points={{42,
          51.92},{42,46},{124,46},{124,12},{142,12},{142,-2},{284,-2},{284,18},{
          340,18},{340,14},{343,14},{343,6.24444}},color={217,67,180}));
  connect(Bypass_SEHP2Cond.C_out, condenser.C_hot_in) annotation (Line(points={{24,
          11.92},{24,0},{74,0},{74,-2},{138,-2},{138,12},{142,12},{142,-2},{284,
          -2},{284,18},{340,18},{340,14},{343,14},{343,6.24444}},
                                                                color={217,67,180}));
  connect(BypassHPCV2Cond.C_out, condenser.C_hot_in) annotation (Line(points={{-40,
          11.92},{-40,-8},{46,-8},{46,0},{74,0},{74,-2},{138,-2},{138,12},{142,12},
          {142,-2},{284,-2},{284,18},{340,18},{340,14},{343,14},{343,6.24444}},
        color={217,67,180}));
  connect(Bypass_SteamGen2Cond.C_out, condenser.C_hot_in) annotation (Line(
        points={{-68,11.92},{-50,11.92},{-50,2},{-40,2},{-40,-8},{46,-8},{46,0},
          {74,0},{74,-2},{138,-2},{138,12},{142,12},{142,-2},{284,-2},{284,18},{
          340,18},{340,14},{343,14},{343,6.24444}},
                                                  color={217,67,180}));
  connect(PressureLoss_DryerCondensats.C_out, Bypass_SD2Cond.C_in) annotation (
      Line(points={{154,1.8},{154,-10},{168,-10},{168,2}}, color={217,67,180}));
  connect(Bypass_SD2Cond.C_out, condenser.C_hot_in) annotation (Line(points={{174.06,
          2},{282,2},{282,18},{340,18},{340,14},{343,14},{343,6.24444}},
                                                                       color={63,
          81,181}));
  connect(Superheater.C_cold_out, Bypass_SH2Cond.C_in)
    annotation (Line(points={{168,58},{168,67},{186,67}}, color={217,67,180}));
  connect(Bypass_SH2Cond.C_out, condenser.C_hot_in) annotation (Line(points={{194.08,
          67},{230,67},{230,2},{282,2},{282,18},{340,18},{340,14},{343,14},{343,
          6.24444}},
                   color={217,67,180}));
  connect(HPReheater.C_hot_out, Bypass_HPR2Cond.C_in)
    annotation (Line(points={{0,-106},{0,-113},{16,-113}}, color={217,67,180}));
  connect(Bypass_HPR2Cond.C_out, condenser.C_hot_in) annotation (Line(points={{24.08,
          -113},{86,-113},{86,-108},{300,-108},{300,-18},{288,-18},{288,18},{340,
          18},{340,14},{343,14},{343,6.24444}},
                                              color={217,67,180}));
  annotation ();
end MetroscopiaNPP_faulty;
