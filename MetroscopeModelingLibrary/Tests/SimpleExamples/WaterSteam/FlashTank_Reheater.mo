within MetroscopeModelingLibrary.Tests.SimpleExamples.WaterSteam;
model FlashTank_Reheater
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;
  replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.CondReheater CondReheater
    annotation (Placement(transformation(extent={{-20,26},{12,42}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_cold_hotwater
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-186,34})));

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_cold_coldwater
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={48,34})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_hot_SatSteam_1
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-4,78})));
  MetroscopeModelingLibrary.WaterSteam.Junctions.FlashTank flashTank
    annotation (Placement(transformation(extent={{-132,-50},{-90,-22}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_hot_SatSteam_2
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-132,80})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss DPz
    annotation (Placement(transformation(
        extent={{-12.5,-12.5},{12.5,12.5}},
        rotation=180,
        origin={-34.5,8.5})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StaticCentrifugalPump PRP
    annotation (Placement(transformation(
        extent={{7,-8},{-7,8}},
        rotation=0,
        origin={-117,-68})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureLoss DP_vaporBalance
    annotation (Placement(transformation(extent={{-68,50},{-56,62}})));
equation
  //inlets
  source_hot_SatSteam_1.h_out = 2.5e6;
  source_hot_SatSteam_1.P_out = 11e5;

  source_cold_coldwater.P_out = 50e5;
  source_cold_coldwater.T_out = 130 + 273.15;
  source_cold_coldwater.Q_out = -1000;

  source_hot_SatSteam_2.h_out = 8e5;
  source_hot_SatSteam_2.Q_out = -500;
  //source_hot_SatSteam_2.P_out = 11e5;

  //outlets
  sink_cold_hotwater.h_vol = 0.9e6;
  //sink_cold_hotwater.T_in = 180 + 273.15;

  //sink_hot_CondSteam.h_vol = 1e6;

  //heatexchanger
  CondReheater.S_tot=100;
  CondReheater.Kfr_hot=1;
  CondReheater.Kfr_cold=1;
  CondReheater.Kth=1e5;

  DPz.z1 = 10;
  DPz.z2 = 0;
  DPz.Kfr = 1;

  //FlashTank and pump
  PRP.VRot = 1000;
  PRP.VRotn = 1000;
  PRP.rm = 1;
  PRP.a1 = 0;
  PRP.a2 = 0;
  PRP.b1 = 0;
  PRP.b2 = 0;
  PRP.rhmin = 0.20;
  PRP.rh = 1;

  connect(source_hot_SatSteam_1.C_out, CondReheater.C_hot_in)
    annotation (Line(points={{-4,68},{-4,42},{-3,42}}, color={63,81,181}));
  connect(source_cold_coldwater.C_out, CondReheater.C_cold_in)
    annotation (Line(points={{38,34},{12,34}}, color={63,81,181}));
  connect(sink_cold_hotwater.C_in, CondReheater.C_cold_out) annotation (Line(
        points={{-176,34},{-20,34}},                  color={63,81,181}));
  connect(source_hot_SatSteam_2.C_out, flashTank.C_in)
    annotation (Line(points={{-132,70},{-132,-22.7}}, color={63,81,181}));
  connect(CondReheater.C_hot_out, DPz.C_in) annotation (Line(points={{-4,26},{-6,
          26},{-6,8},{-22,8.5}}, color={63,81,181}));
  connect(DPz.C_out, flashTank.C_in) annotation (Line(points={{-47.25,8.5},{-132,
          8},{-132,-22.7}}, color={63,81,181}));
  connect(flashTank.C_liquid_out, PRP.C_in) annotation (Line(points={{-87.9,-49.3},
          {-87.9,-68},{-110,-68}}, color={63,81,181}));
  connect(PRP.C_out, sink_cold_hotwater.C_in) annotation (Line(points={{-124.14,
          -68},{-152,-68},{-152,34},{-176,34}}, color={63,81,181}));
  connect(flashTank.C_vapor_out, DP_vaporBalance.C_in) annotation (Line(points=
          {{-87.9,-22},{-88,-22},{-88,56},{-68,56}}, color={63,81,181}));
  connect(DP_vaporBalance.C_out, CondReheater.C_hot_in)
    annotation (Line(points={{-55.88,56},{-3,56},{-3,42}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{100,100}})));
end FlashTank_Reheater;
