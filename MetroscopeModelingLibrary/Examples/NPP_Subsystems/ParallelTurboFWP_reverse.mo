within MetroscopeModelingLibrary.Examples.NPP_Subsystems;
model ParallelTurboFWP_reverse
  // Initialization parameters
  parameter Units.Pressure STs_CV_P_in_0 = 30e5;
  parameter Units.Pressure STs_P_out_0 = 0.07e5;
  parameter Units.Pressure FWPs_P_in_0 = 44.6e5;
  // BC
  //STs_CV
  input Units.Pressure STs_CV_P_in(start=STs_CV_P_in_0) "Pa";
  input Units.Fraction ST1_CV_opening(start=0.70) "%";
  input Units.Fraction ST2_CV_opening(start=0.70) "%";
  //STs
  input Units.Pressure STs_P_out(start=STs_P_out_0) "condenser pressure, Pa";
  // FWPs
  input Units.Pressure FWPs_P_in(start=FWPs_P_in_0) "Pa";
  input Units.OutletMassFlowRate FWPs_Q_in(start=-1.5e3) "kg/s";
  input Units.Temperature FWPs_T_in(start=186 + 273.15) "degC";

  // Observables used for calibration
  // STs_CV
  input Real STs_CV_Q_in(start=13.4) "kg/s";
  // STs
  input Real ST1_P_in(start=10) "barA";
  input Real ST2_P_in(start=10) "barA";
  // FWPs
  input Real FWP1_Q_in(start=783.5) "kg/s";
  input Real FWP1_VRot(start=4253) "rpm";
  input Real FWP2_VRot(start=4206) "rpm";
  input Real FWPs_T_out(start=186.7) "degC";
  input Real FWPs_P_out(start=82) "barA";

  // Component characteristics
  // STs_CV
  output Units.Cv STs_CVmax;
  // STs
  output Units.Yield ST1_eta_is;
  output Units.Yield ST2_eta_is;
  output Units.Cst ST1_Cst;
  output Units.Cst ST2_Cst;
  // FWPs
  output Real FWP1_a3;
  output Real FWP2_a3;
  output Real FWPs_b3;

  // Components
  // ST BCs
  WaterSteam.BoundaryConditions.WaterSource STs_source annotation (Placement(transformation(extent={{-128,70},{-108,90}})));
  WaterSteam.BoundaryConditions.WaterSink STs_sink annotation (Placement(transformation(extent={{108,70},{128,90}})));
  // STs
  Sensors.WaterSteam.WaterPressureSensor ST1_P_in_sensor annotation (Placement(transformation(extent={{10,93.0001},{24,107}})));
  Sensors.WaterSteam.WaterPressureSensor ST2_P_in_sensor annotation (Placement(transformation(extent={{10,53},{24,67}})));
  WaterSteam.Machines.StodolaTurbine ST1(P_out_0=STs_P_out_0) annotation (Placement(transformation(extent={{32,90.0002},{52,110}})));
  WaterSteam.Machines.StodolaTurbine ST2(P_out_0=STs_P_out_0) annotation (Placement(transformation(extent={{32,70},{52,50}})));
  // STs CV
  WaterSteam.Pipes.WaterControlValve ST2_CV(P_in_0=STs_CV_P_in_0) annotation (Placement(transformation(extent={{-16,62.5455},{-4,48.5455}})));
  WaterSteam.Pipes.WaterControlValve ST1_CV(P_in_0=STs_CV_P_in_0) annotation (Placement(transformation(extent={{-16,97.4545},{-4,111.455}})));
  Sensors.Other.OpeningSensor ST1_CV_opening_sensor annotation (Placement(transformation(extent={{-14,116},{-6,124}})));
  Sensors.Other.OpeningSensor ST2_CV_opening_sensor annotation (Placement(transformation(extent={{-14,44},{-6,36}})));

  // FWP BCs
  WaterSteam.BoundaryConditions.WaterSource FWPs_source annotation (Placement(transformation(extent={{128,-70},{108,-50}})));
  WaterSteam.BoundaryConditions.WaterSink FWPs_sink annotation (Placement(transformation(extent={{-108,-70},{-128,-50}})));
  // Pumps
  WaterSteam.Machines.WaterPump FWP2 annotation (Placement(transformation(extent={{-10,-40},{-30,-20}})));
  WaterSteam.Machines.WaterPump FWP1 annotation (Placement(transformation(extent={{-10,-80},{-30,-100}})));
  Sensors.WaterSteam.WaterFlowSensor STs_CV_Q_in_sensor annotation (Placement(transformation(extent={{-76,73},{-62,87}})));
  Sensors.Other.VRotSensor FWP1_VRot_sensor annotation (Placement(transformation(
        extent={{-7.5,-7.5},{7.5,7.5}},
        rotation=270,
        origin={0,-70})));
  Sensors.Other.VRotSensor FWP2_VRot_sensor annotation (Placement(transformation(
        extent={{-7.5,-7.5},{7.5,7.5}},
        rotation=270,
        origin={0,-50})));
  // Pumps outlet
  Sensors.WaterSteam.WaterTemperatureSensor FWPs_T_out_sensor annotation (Placement(transformation(extent={{-84,-67},{-98,-53}})));
  Sensors.WaterSteam.WaterPressureSensor FWPs_P_out_sensor annotation (Placement(transformation(extent={{-64,-67},{-78,-53}})));
  Sensors.WaterSteam.WaterFlowSensor FWP1_Q_in_sensor annotation (Placement(transformation(extent={{38,-97},{24,-83}})));
equation
  // Boundary conditions
  // STs source
  STs_source.h_out = 2.7718e6; // set Temp ?
  STs_source.P_out = STs_CV_P_in;

  // ST1 CV
  ST1_CV_opening_sensor.Opening = ST1_CV_opening;

  // ST2_CV
  ST2_CV_opening_sensor.Opening = ST2_CV_opening;

  // STs sink
  STs_sink.P_in = STs_P_out;

  // FWPs
  FWPs_source.T_out = FWPs_T_in;
  FWPs_source.P_out = FWPs_P_in;
  FWPs_source.Q_out = FWPs_Q_in;


  // STs_CV
  // Observables used for calibration
  STs_CV_Q_in_sensor.Q = STs_CV_Q_in;

  // ST1_CV // Hyp on CVs : same Cv (or same mass flow)
  // Calibrated  parameters
  ST1_CV.Cvmax = STs_CVmax;

  // ST1_CV
  // Calibrated parameters
  ST2_CV.Cvmax = STs_CVmax;

  // ST1
  // Observables used for calibration
  ST1_P_in_sensor.P_barA = ST1_P_in;
  // Calibrated parameters
  ST1.eta_nz = 1.0;
  ST1.area_nz = 1.0;
  ST1.eta_is = ST1_eta_is;
  ST1.Cst = ST1_Cst;

  // ST2
  // Observables used for calibration
  ST2_P_in_sensor.P_barA = ST2_P_in;
  // Calibrated parameters
  ST2.eta_nz = 1.0;
  ST2.area_nz = 1.0;
  ST2.eta_is = ST2_eta_is;
  ST2.Cst = ST2_Cst;

  // FWPs
  // Observables used for calibration
  FWPs_T_out_sensor.T_degC = FWPs_T_out;
  FWPs_P_out_sensor.P_barA = FWPs_P_out;

  // FWP1
  // Observables used for calibration
  FWP1_Q_in_sensor.Q = FWP1_Q_in;
  FWP1_VRot_sensor.VRot = FWP1_VRot;
  // Calibrated parameters
  FWP1.b3 = FWPs_b3;
  FWP1.a3 = FWP1_a3;
  // fixed parameters
  FWP1.VRotn = 4300;
  FWP1.rm = 0.85;
  FWP1.rhmin = 0.20;
  FWP1.b2 = 0;
  FWP1.b1 = 0;
  FWP1.a2 = 0;
  FWP1.a1 = -172;

  // FWP2
  // Observables used for calibration
  FWP2_VRot_sensor.VRot = FWP2_VRot;
  // Calibrated parameters
  FWP2.b3 = FWPs_b3;
  FWP2.a3 = FWP2_a3;
  // fixed parameters
  FWP2.VRotn = 4500;
  FWP2.rm = 0.85;
  FWP2.rhmin = 0.20;
  FWP2.b2 = 0;
  FWP2.b1 = 0;
  FWP2.a2 = 0;
  FWP2.a1 = -172;
  connect(ST1.C_out, ST2.C_out) annotation (Line(points={{52,100},{70,100},{70,60},{52,60}}, color={28,108,200}));
  connect(STs_sink.C_in, ST2.C_out) annotation (Line(points={{113,80},{70,80},{70,60},{52,60}}, color={28,108,200}));
  connect(ST1_CV.C_in, ST2_CV.C_in) annotation (Line(points={{-16,100},{-36,100},{-36,60},{-16,60}}, color={28,108,200}));
  connect(ST1_CV.Opening, ST1_CV_opening_sensor.Opening) annotation (Line(points={{-10,110.182},{-10,115.92}}, color={0,0,127}));
  connect(ST2_CV.Opening, ST2_CV_opening_sensor.Opening) annotation (Line(points={{-10,49.8182},{-10,44.08}}, color={0,0,127}));
  connect(FWP1.C_out, FWP2.C_out) annotation (Line(points={{-30,-90},{-40,-90},{-40,-30},{-30,-30}},
                                                                                                   color={28,108,200}));
  connect(ST2.C_W_out, FWP2.C_power) annotation (Line(points={{52,51.6},{62,51.6},{62,-8},{-20,-8},{-20,-19.2}},
                                                                                                               color={244,125,35}));
  connect(ST1.C_W_out, FWP1.C_power) annotation (Line(points={{52,108.4},{160,108.4},{160,-120},{-20,-120},{-20,-100.8}},
                                                                                                                        color={244,125,35}));
  connect(STs_source.C_out, STs_CV_Q_in_sensor.C_in) annotation (Line(points={{-113,80},{-76,80}}, color={28,108,200}));
  connect(STs_CV_Q_in_sensor.C_out, ST2_CV.C_in) annotation (Line(points={{-62,80},{-36,80},{-36,60},{-16,60}}, color={28,108,200}));
  connect(FWP1.VRot, FWP1_VRot_sensor.VRot) annotation (Line(points={{-20,-78},{-20,-70},{-7.65,-70}}, color={0,0,127}));
  connect(FWP2.VRot, FWP2_VRot_sensor.VRot) annotation (Line(points={{-20,-42},{-20,-50},{-7.65,-50}}, color={0,0,127}));
  connect(FWPs_P_out_sensor.C_in, FWP2.C_out) annotation (Line(points={{-64,-60},{-40,-60},{-40,-30},{-30,-30}}, color={28,108,200}));
  connect(FWPs_sink.C_in, FWPs_T_out_sensor.C_out) annotation (Line(points={{-113,-60},{-98,-60}}, color={28,108,200}));
  connect(FWPs_T_out_sensor.C_in, FWPs_P_out_sensor.C_out) annotation (Line(points={{-84,-60},{-78,-60}}, color={28,108,200}));
  connect(ST1_CV.C_out, ST1_P_in_sensor.C_in) annotation (Line(points={{-4,100},{0,100},{0,100},{4,100},{4,100},{10,100}},
                                                                                           color={28,108,200}));
  connect(ST1_P_in_sensor.C_out, ST1.C_in) annotation (Line(points={{24,100},{26,100},{26,100},{28,100},{28,100},{32,100}},
                                                                                        color={28,108,200}));
  connect(ST2.C_in, ST2_P_in_sensor.C_out) annotation (Line(points={{32,60},{24,60}}, color={28,108,200}));
  connect(ST2_P_in_sensor.C_in, ST2_CV.C_out) annotation (Line(points={{10,60},{4,60},{4,60},{-4,60}}, color={28,108,200}));
  connect(FWPs_source.C_out, FWP1_Q_in_sensor.C_in) annotation (Line(points={{113,-60},{60,-60},{60,-90},{38,-90}}, color={28,108,200}));
  connect(FWP1_Q_in_sensor.C_out, FWP1.C_in) annotation (Line(points={{24,-90},{-10,-90}}, color={28,108,200}));
  connect(FWP2.C_in, FWP1_Q_in_sensor.C_in) annotation (Line(points={{-10,-30},{60,-30},{60,-90},{38,-90}}, color={28,108,200}));
  annotation (Diagram(coordinateSystem(extent={{-140,-140},{140,140}})), Icon(coordinateSystem(extent={{-140,-140},{140,140}}), graphics={
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,0},{80,0}}),
        Line(points={{80,0},{2,60}}),
        Line(points={{80,0},{0,-60}})}));
end ParallelTurboFWP_reverse;
