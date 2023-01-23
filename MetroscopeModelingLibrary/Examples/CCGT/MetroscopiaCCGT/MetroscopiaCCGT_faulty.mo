within MetroscopeModelingLibrary.Examples.CCGT.MetroscopiaCCGT;
model MetroscopiaCCGT_faulty "Metroscopia CCGT faulty"
  extends MetroscopeModelingLibrary.Examples.CCGT.MetroscopiaCCGT.MetroscopiaCCGT_causality_direct_withStartValues(
    condenser(faulty=true),
    AirFilter(faulty=true),
    HPsuperheater1(faulty=true),
    HPsuperheater2(faulty=true),
    Reheater(faulty=true),
    evaporator(faulty=true),
    economiser(faulty=true),
    HPST_control_valve(faulty=true));

  // Heat exchangers failures
  input Real Failure_Reheater_fouling(start=0);
  input Real Failure_evaporator_fouling(start=0);
  input Real Failure_HPsuperheater1_fouling(start=0);
  input Real Failure_HPsuperheater2_fouling(start=0);
  input Real Failure_economiser_fouling(start=0);
  input Real Failure_condenser_fouling(start=0);
  input Real Failure_condenser_air_intake(start=0);

  // Leaks
  input Real Failure_bypass_HP_turbine_to_condenser_leak_Q(start=0);
  input Real Failure_bypass_HP_CV_to_condenser_leak_Q(start=0);
  input Real Failure_bypass_IP_turbine_to_condenser_leak_Q(start=0);
  input Real Failure_bypass_IP_CV_to_condenser_leak_Q(start=0);
  input Real Failure_deSH_controlValve_leak_Q(start=0);

  // Gas turbine failures
  input Real Failure_AirFilter_fouling;

  // Steam turbine
  input Real Failure_HPST_CV_opening_fault(start=0);

  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_HP_turbine_to_condenser_leak
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-110,182})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_HP_CV_to_condenser_leak
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-162,192})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_IP_CV_to_condenser_leak
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-4,284})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak bypass_IP_turbine_to_condenser_leak
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-4,268})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak deSH_controlValve_leak
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-158,64})));
equation

  //Condenser
  condenser.fouling = Failure_condenser_fouling;
  condenser.air_intake = Failure_condenser_air_intake;

  //Reheater
  Reheater.fouling = Failure_Reheater_fouling;
  evaporator.fouling = Failure_evaporator_fouling;
  //economiser
  economiser.fouling = Failure_economiser_fouling;

  //Superheater
  HPsuperheater1.fouling = Failure_HPsuperheater1_fouling;
  HPsuperheater2.fouling = Failure_HPsuperheater2_fouling;
  deSH_controlValve_leak.Q = Failure_deSH_controlValve_leak_Q + 1E-3;

  //Steam Turbines
  bypass_HP_turbine_to_condenser_leak.Q = Failure_bypass_HP_turbine_to_condenser_leak_Q+1E-3;
  bypass_HP_CV_to_condenser_leak.Q = Failure_bypass_HP_CV_to_condenser_leak_Q+1E-3;
  bypass_IP_turbine_to_condenser_leak.Q = Failure_bypass_IP_turbine_to_condenser_leak_Q+1E-3;
  bypass_IP_CV_to_condenser_leak.Q = Failure_bypass_IP_CV_to_condenser_leak_Q+1E-3;
  HPST_control_valve.opening_fault = Failure_HPST_CV_opening_fault + 0.2*time;

  //Gas turbine
  AirFilter.fouling = Failure_AirFilter_fouling;

  connect(P_HPST_in_sensor.C_in, HPST_control_valve.C_out) annotation (Line(
        points={{-180,148},{-183.375,148},{-183.375,148},{-186.75,148}}, color={
          28,108,200}));
  connect(bypass_HP_CV_to_condenser_leak.C_out, condenser.C_hot_in) annotation (
     Line(points={{-152,192.2},{52,192.2},{52,176.778}},
                                                     color={217,67,180}));
  connect(HPsteamTurbine.C_in, bypass_HP_turbine_to_condenser_leak.C_in)
    annotation (Line(points={{-160,148},{-158,148},{-158,182},{-120,182}},
        color={217,67,180}));
  connect(bypass_HP_turbine_to_condenser_leak.C_out, condenser.C_hot_in)
    annotation (Line(points={{-100,182.2},{52,182.2},{52,176.778}},
                                                                color={217,67,
          180}));
  connect(bypass_IP_CV_to_condenser_leak.C_in, LPST_control_valve.C_in)
    annotation (Line(points={{-14,284},{-80,284},{-80,214},{-61.25,214}}, color={217,67,
          180}));
  connect(bypass_IP_CV_to_condenser_leak.C_out, condenser.C_hot_in) annotation (
     Line(points={{6,284.2},{52,284.2},{52,176.778}},
                                                  color={217,67,180}));
  connect(LPST_control_valve.C_out, bypass_IP_turbine_to_condenser_leak.C_in)
    annotation (Line(points={{-44.75,214},{-38,214},{-38,268},{-14,268}}, color={217,67,
          180}));
  connect(bypass_IP_turbine_to_condenser_leak.C_out, condenser.C_hot_in)
    annotation (Line(points={{6,267.8},{52,267.8},{52,176.778}},
                                                             color={217,67,180}));
  connect(deSH_controlValve_leak.C_out, deSH_controlValve.C_out) annotation (
      Line(points={{-168,64.2},{-184,64.2},{-184,92},{-171.25,92}},
                                                                color={217,67,
          180}));
  connect(deSH_controlValve_leak.C_in, Q_deSH_sensor.C_in) annotation (Line(
        points={{-148,64},{-132,64},{-132,92}}, color={217,67,180}));
  connect(bypass_HP_CV_to_condenser_leak.C_in, HPST_control_valve.C_in)
    annotation (Line(points={{-172,192},{-222,192},{-222,148},{-203.25,148}},
        color={217,67,180}));
  annotation (Diagram(coordinateSystem(extent={{-720,-120},{260,340}})), Icon(
        coordinateSystem(extent={{-720,-120},{260,340}})));
end MetroscopiaCCGT_faulty;
