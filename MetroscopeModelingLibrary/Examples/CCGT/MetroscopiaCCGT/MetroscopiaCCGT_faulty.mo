within MetroscopeModelingLibrary.Examples.CCGT.MetroscopiaCCGT;
model MetroscopiaCCGT_faulty "Metroscopia CCGT faulty"

  extends MetroscopiaCCGT_direct_withStartValues(condenser(faulty=true),
    filter(faulty=true),
    HPSH1(faulty=true),
    HPSH2(faulty=true),
    RHT(faulty=true),
    EVAP(faulty=true),
    ECO(faulty=true),
    HPST_admission_valve(faulty=true),
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
  input Real Fault_bypass_HP_CV_to_condenser_leak_Q(start=0);

  // Gas turbine failures
  input Real Fault_AirFilter_fouling(start=0);
  input Real Fault_airCompressor_tau_decrease(start=0);
  input Real Fault_airCompressor_eta_is_decrease(start=0);
  input Real Fault_gasTurbine_eta_is_decrease(start=0);

  // Steam turbine
  input Real Fault_HPST_CV_closed_valve(start=0);

  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_HP_CV_to_condenser_leak
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-166,220})));
equation

  //Condenser
  condenser.fouling = Fault_condenser_fouling;
  condenser.air_intake = Fault_condenser_air_intake;
  condenser.Qv_cold_in_decrease = Fault_condenser_Qv_cold_in_decrease;

  //Reheater
  RHT.fouling = Fault_Reheater_fouling;
  EVAP.fouling = Fault_evaporator_fouling;
  //economiser
  ECO.fouling = Fault_economiser_fouling;

  //Superheater
  HPSH1.fouling = Fault_HPsuperheater1_fouling;
  HPSH2.fouling = Fault_HPsuperheater2_fouling;

  //Steam Turbines
  bypass_HP_CV_to_condenser_leak.Q = Fault_bypass_HP_CV_to_condenser_leak_Q+1E-3;
  HPST_admission_valve.closed_valve = Fault_HPST_CV_closed_valve;

  //Gas turbine
  filter.fouling = Fault_AirFilter_fouling;
  airCompressor.tau_decrease = Fault_airCompressor_tau_decrease;
  airCompressor.eta_is_decrease = Fault_airCompressor_eta_is_decrease;
  gasTurbine.eta_is_decrease = Fault_gasTurbine_eta_is_decrease;

  connect(bypass_HP_CV_to_condenser_leak.C_out, condenser.C_hot_in) annotation (Line(points={{-156,
          220},{-20,220},{-20,-240}},                                                                                           color={28,108,200}));
  connect(bypass_HP_CV_to_condenser_leak.C_in, HPST_admission_valve.C_in)
    annotation (Line(points={{-176,220},{-296,220},{-296,206},{-540,206},{-540,-200},
          {-530,-200}}, color={28,108,200}));
  annotation (Diagram(coordinateSystem(extent={{-720,-120},{260,340}})), Icon(
        coordinateSystem(extent={{-720,-120},{260,340}})));
end MetroscopiaCCGT_faulty;
