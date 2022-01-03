within MetroscopeModelingLibrary.Tests.SimpleExamples.WaterSteam;
model MetroscopeTrainingTest
  extends Modelica.Icons.Example;
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  /* --- Boundary Conditions --- */
  input Real W_heater(start = 35);
  input Real P_heater(start = 4);
  input Real T_heater(start = 66);
  input Real P_cooling(start = 3);
  input Real T_cooling(start = 12);
  input Real Q_cooling(start = 990);
  /* --- Parameters --- */
  parameter Real controlValve_Cvmax = 1e4;
  parameter Real heatExchanger_heatTransfer = 1e4;
  parameter Real heatExchanger_friction_coldSide = 1e-6;
  parameter Real heatExchanger_friction_hotSide = 1424.61;
  /* --- Failures --- */
  input Real heatEchanger_foiling(start=0.05);
  input Real controlValve_closing(start=0.1);
  input Real controlValve_bypass_flow(start=10);
  /* --- Observables --- */
  output Real controlValve_P_out;
  output Real controlValve_T_out;
  output Real heatExchanger_hotSide_P_out;
  output Real heatExchanger_hotSide_T_out;
  output Real heatExchanger_hotSide_Q_out;
  output Real heatExchanger_coldSide_P_out;
  output Real heatExchanger_coldSide_T_out;
  output Real pump_P_in;
  output Real pump_T_in;
  output Real pump_P_out;
  output Real pump_T_out;
  MetroscopeModelingLibrary.WaterSteam.Machines.StaticCentrifugalPump pump
    annotation (Placement(transformation(extent={{-16,10},{-36,-10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-76,36},{-56,56}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-58,-10},{-78,10}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve controlValve
    annotation (Placement(transformation(extent={{-34,42},{-14,64}})));
  Multifluid.HeatExchangers.NTUCounterCurrentHeatExchanger heatExchanger(
      redeclare package HotMedium = WaterSteamMedium, redeclare package
      ColdMedium = WaterSteamMedium) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={68,22})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_cooling
    annotation (Placement(transformation(extent={{26,12},{46,32}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_cooling
    annotation (Placement(transformation(extent={{88,12},{108,32}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve bypassValve
    annotation (Placement(transformation(
        extent={{4.5,-5.5},{-4.5,5.5}},
        rotation=90,
        origin={11.5,22.5})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureLoss
    singularPressureLoss
    annotation (Placement(transformation(extent={{38,40},{50,52}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureLoss
    singularPressureLoss1
    annotation (Placement(transformation(extent={{50,-6},{38,6}})));
equation
  /* --- Boundary Conditions --- */
  W_heater*1e6 = sink.Q_in*(source.h_out - sink.h_in);
  source.P_out = P_heater * 1e5;
  source.T_out = T_heater +273.15;
  source_cooling.P_out = P_cooling*1e5;
  source_cooling.Q_out = -Q_cooling;
  source_cooling.T_vol = T_cooling + 273.15;
  /* --- Parameters --- */
  controlValve.Cvmax = controlValve_Cvmax;
  heatExchanger.K_friction_cold = heatExchanger_friction_coldSide;
  heatExchanger.K_friction_hot = heatExchanger_friction_hotSide;
  heatExchanger.K = heatExchanger_heatTransfer*(1-heatEchanger_foiling);
  /* --- Observables --- */
  controlValve_P_out = controlValve.P_out / 1e5;
  controlValve_T_out  = controlValve.T_out -273.15;
  heatExchanger_hotSide_P_out = heatExchanger.hotSide.P_out /1e5;
  heatExchanger_hotSide_T_out = heatExchanger.hotSide.T_out - 273.15;
  heatExchanger_hotSide_Q_out = -heatExchanger.hotSide.Q_out;
  heatExchanger_coldSide_P_out = heatExchanger.coldSide.P_out /1e5;
  heatExchanger_coldSide_T_out = heatExchanger.coldSide.T_out - 273.15;
  pump_P_in = pump.P_in / 1e5;
  pump_T_in = pump.T_in -273.15;
  pump_P_out = pump.P_out / 1e5;
  pump_T_out = pump.T_out -273.15;
  /* --- Others --- */
  sink.T_vol = 20+273.15;
  pump.VRot = 1400;
  pump.VRotn = 1400;
  pump.rm = 0.85;
  pump.a1 = -88.67;
  pump.a2 = 0;
  pump.a3 = 43.15;
  pump.b1 = -3.7751;
  pump.b2 = 3.61;
  pump.b3 = -0.0075464;
  pump.rhmin = 0.20;
  heatExchanger.S=100;
  controlValve.Opening=1-controlValve_closing;
  bypassValve.Cvmax = 1e4;
  bypassValve.Q_in = controlValve_bypass_flow+1e-6;
  sink_cooling.T_vol = 8 + 273.15;
  singularPressureLoss.Kfr = 1e-6;
  singularPressureLoss1.Kfr = 1e-6;
  connect(sink.C_in, pump.C_out)
    annotation (Line(points={{-58,0},{-36.2,0}}, color={0,0,255}));
  connect(source.C_out, controlValve.C_in)
    annotation (Line(points={{-56,46},{-34,46}}, color={238,46,47}));
  connect(source_cooling.C_out, heatExchanger.C_cold_in)
    annotation (Line(points={{46,22},{58,22}}, color={238,46,47}));
  connect(heatExchanger.C_cold_out, sink_cooling.C_in)
    annotation (Line(points={{77.8,22},{88,22}}, color={238,46,47}));
  connect(bypassValve.C_out, pump.C_in)
    annotation (Line(points={{15,17.91},{15,0},{-16,0}}, color={238,46,47}));
  connect(heatExchanger.C_hot_in, singularPressureLoss.C_out) annotation (Line(
        points={{67.8,31.8},{67.8,46},{50.12,46}}, color={0,0,255}));
  connect(controlValve.C_out, singularPressureLoss.C_in)
    annotation (Line(points={{-13.8,46},{38,46}}, color={238,46,47}));
  connect(bypassValve.C_in, singularPressureLoss.C_in)
    annotation (Line(points={{15,27},{15,46},{38,46}}, color={0,0,255}));
  connect(heatExchanger.C_hot_out, singularPressureLoss1.C_in)
    annotation (Line(points={{68,12.2},{68,0},{50,0}}, color={238,46,47}));
  connect(singularPressureLoss1.C_out, pump.C_in)
    annotation (Line(points={{37.88,0},{-16,0}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                         Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},{120,80}})));
end MetroscopeTrainingTest;
