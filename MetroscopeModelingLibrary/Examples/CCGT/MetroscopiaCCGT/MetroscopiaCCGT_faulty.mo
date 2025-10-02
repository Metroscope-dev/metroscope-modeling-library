within MetroscopeModelingLibrary.Examples.CCGT.MetroscopiaCCGT;
model MetroscopiaCCGT_faulty "Metroscopia CCGT faulty"
  extends MetroscopiaCCGT_direct_mml4_wSV(
    condenser(faulty=true),
    AirFilter(faulty=true),
    HPsuperheater1(faulty=true),
    HPsuperheater2(faulty=true),
    Reheater(faulty=true),
    evaporator(faulty=true),
    economiser(faulty=true),
    HPST_control_valve(faulty=true),
    airCompressor(faulty=true),
    gasTurbine(faulty=true));

  // Heat exchangers failures
  input Real Fault_Reheater_fouling(start=0);
  input Real Fault_evaporator_fouling(start=0);
  input Real Fault_HPsuperheater1_fouling(start=0);
  input Real Fault_HPsuperheater2_fouling(start=0);
  input Real Fault_economiser_fouling(start=0);
  input Real Fault_condenser_fouling(start=0);
  input Real Fault_condenser_air_intake(start=0);
  input Real Fault_condenser_Qv_cold_in_decrease(start=0);

  // Leaks
  input Real Fault_bypass_HP_turbine_to_condenser_leak_Q(start=0);
  input Real Fault_bypass_HP_CV_to_condenser_leak_Q(start=0);
  input Real Fault_bypass_IP_turbine_to_condenser_leak_Q(start=0);
  input Real Fault_bypass_IP_CV_to_condenser_leak_Q(start=0);
  input Real Fault_deSH_controlValve_leak_Q(start=0);

  // Gas turbine failures
  input Real Fault_AirFilter_fouling(start=0);
  input Real Fault_airCompressor_tau_decrease(start=0);
  input Real Fault_airCompressor_eta_is_decrease(start=0);
  input Real Fault_gasTurbine_eta_is_decrease(start=0);

  // Steam turbine
  input Real Fault_HPST_CV_closed_valve(start=0);

  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_HP_turbine_to_condenser_leak
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-104,200})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_HP_CV_to_condenser_leak
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-166,220})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_IP_CV_to_condenser_leak
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-6,320})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_IP_turbine_to_condenser_leak
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-4,300})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak deSH_controlValve_leak
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-148,80})));
equation

  //Condenser
  condenser.fouling = Fault_condenser_fouling;
  condenser.air_intake = Fault_condenser_air_intake;
  condenser.Qv_cold_in_decrease = Fault_condenser_Qv_cold_in_decrease;

  //Reheater
  Reheater.fouling = Fault_Reheater_fouling;
  evaporator.fouling = Fault_evaporator_fouling;
  //economiser
  economiser.fouling = Fault_economiser_fouling;

  //Superheater
  HPsuperheater1.fouling = Fault_HPsuperheater1_fouling;
  HPsuperheater2.fouling = Fault_HPsuperheater2_fouling;
  deSH_controlValve_leak.Q = Fault_deSH_controlValve_leak_Q + 1E-3;

  //Steam Turbines
  bypass_HP_turbine_to_condenser_leak.Q = Fault_bypass_HP_turbine_to_condenser_leak_Q+1E-3;
  bypass_HP_CV_to_condenser_leak.Q = Fault_bypass_HP_CV_to_condenser_leak_Q+1E-3;
  bypass_IP_turbine_to_condenser_leak.Q = Fault_bypass_IP_turbine_to_condenser_leak_Q+1E-3;
  bypass_IP_CV_to_condenser_leak.Q = Fault_bypass_IP_CV_to_condenser_leak_Q+1E-3;
  HPST_control_valve.closed_valve = Fault_HPST_CV_closed_valve;

  //Gas turbine
  AirFilter.fouling = Fault_AirFilter_fouling;
  airCompressor.tau_decrease = Fault_airCompressor_tau_decrease;
  airCompressor.eta_is_decrease = Fault_airCompressor_eta_is_decrease;
  gasTurbine.eta_is_decrease = Fault_gasTurbine_eta_is_decrease;

  connect(P_HPST_in_sensor.C_in, HPST_control_valve.C_out) annotation (Line(
        points={{-200,180},{-183.375,180},{-183.375,180},{-206.75,180}}, color={
          28,108,200}));
  connect(LPST_control_valve.C_out, bypass_IP_turbine_to_condenser_leak.C_in)
    annotation (Line(points={{-44.75,280},{-36,280},{-36,300},{-14,300}}, color={217,67,
          180}));
  connect(deSH_controlValve_leak.C_in, loopBreaker.C_in) annotation (Line(points={{-138,80},{-118,80},{-118,120},{180,120},{180,64}}, color={28,108,200}));
  connect(deSH_controlValve_leak.C_out, HPsuperheater2.C_cold_in) annotation (Line(points={{-158,80},{-172,80},{-172,86},{-192,86},{-192,120},{-260,120},{-260,34},{-260,34},{-260,24}},       color={28,108,200}));
  connect(bypass_HP_turbine_to_condenser_leak.C_in, HPsteamTurbine.C_in) annotation (Line(points={{-114,200},{-166,200},{-166,180},{-180,180}}, color={28,108,200}));
  connect(bypass_HP_CV_to_condenser_leak.C_in, HPST_control_valve.C_in) annotation (Line(points={{-176,220},{-220,220},{-220,180},{-223.25,180}}, color={28,108,200}));
  connect(bypass_HP_CV_to_condenser_leak.C_out, condenser.C_hot_in) annotation (Line(points={{-156,220},{60,220},{60,218.134}}, color={28,108,200}));
  connect(bypass_HP_turbine_to_condenser_leak.C_out, condenser.C_hot_in) annotation (Line(points={{-94,200},{-40,200},{-40,220},{60,220},{60,218.134}}, color={28,108,200}));
  connect(bypass_IP_turbine_to_condenser_leak.C_out, condenser.C_hot_in) annotation (Line(points={{6,300},{60,300},{60,218.134}}, color={28,108,200}));
  connect(bypass_IP_CV_to_condenser_leak.C_out, condenser.C_hot_in) annotation (Line(points={{4,320},{60,320},{60,218.134}}, color={28,108,200}));
  connect(bypass_IP_CV_to_condenser_leak.C_in, LPST_control_valve.C_in) annotation (Line(points={{-16,320},{-70,320},{-70,280},{-61.25,280}}, color={28,108,200}));
  annotation (Diagram(coordinateSystem(extent={{-720,-120},{260,340}})), Icon(
        coordinateSystem(extent={{-720,-120},{260,340}})));
end MetroscopiaCCGT_faulty;
