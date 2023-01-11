within MetroscopeModelingLibrary.Examples.CCGT.MetroscopiaCCGT;
model MetroscopiaCCGT_faulty "Metroscopia CCGT faulty"
  extends MetroscopeModelingLibrary.Examples.CCGT.MetroscopiaCCGT.MetroscopiaCCGT_causality_direct_withStartValues(
    condenser(faulty=true),
    AirFilter(faulty=true),
    HPsuperheater1(faulty=true),
    HPsuperheater2(faulty=true),
    Reheater(faulty=true),
    evaporator(faulty=true),
    economiser(faulty=true));

//Heat exchangers failures
  input Real Reheater_fouling(start=0);
  input Real evaporator_fouling(start=0);
  input Real HPsuperheater1_fouling(start=0);
  input Real HPsuperheater2_fouling(start=0);
  input Real economiser_fouling(start=0);
  input Real condenser_fouling(start=0);
  input Real condenser_air_intake(start=0);

//Leaks
  input Real bypass_HP_turbine_to_condenser_leak_Q(start=0);
  input Real bypass_HP_CV_to_condenser_leak_Q(start=0);
  input Real bypass_IP_turbine_to_condenser_leak_Q(start=0);
  input Real bypass_IP_CV_to_condenser_leak_Q(start=0);
  input Real deSH_controlValve_leak_Q(start=0);

//Gas turbine failures
  input Real AirFilter_fouling;

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
  condenser.fouling=condenser_fouling;
  condenser.air_intake=condenser_air_intake;

  //Reheater
  Reheater.fouling=Reheater_fouling;
  evaporator.fouling=evaporator_fouling;
  //economiser
  economiser.fouling=economiser_fouling;

  //Superheater
  HPsuperheater1.fouling=HPsuperheater1_fouling;
  HPsuperheater2.fouling=HPsuperheater2_fouling;
  deSH_controlValve_leak.Q = deSH_controlValve_leak_Q + 1E-3;

  //Steam Turbines
  bypass_HP_turbine_to_condenser_leak.Q=bypass_HP_turbine_to_condenser_leak_Q+1E-3;
  bypass_HP_CV_to_condenser_leak.Q=bypass_HP_CV_to_condenser_leak_Q+1E-3;
  bypass_IP_turbine_to_condenser_leak.Q=bypass_IP_turbine_to_condenser_leak_Q+1E-3;
  bypass_IP_CV_to_condenser_leak.Q=bypass_IP_CV_to_condenser_leak_Q+1E-3;

  //Gas turbine
  AirFilter.fouling=AirFilter_fouling;

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
