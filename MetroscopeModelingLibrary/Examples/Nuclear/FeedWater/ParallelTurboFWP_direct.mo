within MetroscopeModelingLibrary.Examples.Nuclear.FeedWater;
model ParallelTurboFWP_direct
  import MetroscopeModelingLibrary.Utilities.Units;

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
  input Units.NegativeMassFlowRate FWPs_Q_in(start=-1.5e3) "kg/s";
  input Units.Temperature FWPs_T_in(start=186 + 273.15) "degC";

  // Component characteristics
  // STs_CV
  parameter Units.Cv STs_CVmax = 8e3;
  // STs
  parameter Units.Yield ST1_eta_is = 0.8;
  parameter Units.Yield ST2_eta_is = 0.9;
  parameter Units.Cst ST1_Cst = 5e3;
  parameter Units.Cst ST2_Cst = 6e3;
  // FWPs
  parameter Real FWP1_a3 = 634.53723;
  parameter Real FWP2_a3 = 623.4584;
  parameter Real FWPs_b3 = 0.8617299;

  // Components
  // ST sources
  WaterSteam.BoundaryConditions.Source STs_source annotation (Placement(transformation(extent={{-128,70},{-108,90}})));
  WaterSteam.BoundaryConditions.Sink STs_sink annotation (Placement(transformation(extent={{108,70},{128,90}})));
  // STs
  WaterSteam.Machines.SteamTurbine ST1(P_out_0=STs_P_out_0) annotation (Placement(transformation(extent={{20,90.0002},{40,110}})));
  WaterSteam.Machines.SteamTurbine ST2(P_out_0=STs_P_out_0) annotation (Placement(transformation(extent={{20,70},{40,50}})));
  // STs CV
  WaterSteam.Pipes.ControlValve ST2_CV(P_in_0=STs_CV_P_in_0) annotation (Placement(transformation(extent={{-16,62.5455},{-4,48.5455}})));
  WaterSteam.Pipes.ControlValve ST1_CV(P_in_0=STs_CV_P_in_0) annotation (Placement(transformation(extent={{-16,97.4545},{-4,111.455}})));
  Sensors.Outline.OpeningSensor ST1_CV_opening_sensor annotation (Placement(transformation(extent={{-14,116},{-6,124}})));
  Sensors.Outline.OpeningSensor ST2_CV_opening_sensor annotation (Placement(transformation(extent={{-14,44},{-6,36}})));

  // FWPs
  WaterSteam.BoundaryConditions.Source FWPs_source annotation (Placement(transformation(extent={{128,-70},{108,-50}})));
  WaterSteam.BoundaryConditions.Sink FWPs_sink annotation (Placement(transformation(extent={{-108,-70},{-128,-50}})));

  WaterSteam.Machines.Pump FWP2(P_in_0=FWPs_P_in_0) annotation (Placement(transformation(extent={{-10,-40},{-30,-20}})));
  WaterSteam.Machines.Pump FWP1(P_in_0=FWPs_P_in_0) annotation (Placement(transformation(extent={{-10,-80},{-30,-100}})));
equation
  // Boundary conditions
  // STs source
  STs_source.h_out = 2.7718e6;
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

  // Component parameters
  // ST1_CV // Hyp on CVs : same Cv (or same mass flow)
  ST1_CV.Cv_max = STs_CVmax;

  // ST1_CV
  ST2_CV.Cv_max = STs_CVmax;

  // ST1
  ST1.eta_is = ST1_eta_is;
  ST1.Cst = ST1_Cst;

  // ST2
  ST2.eta_is = ST2_eta_is;
  ST2.Cst = ST2_Cst;

  // FWP1
  FWP1.VRotn = 4300;
  FWP1.rm = 1; // No conversion on turbopumps, so no yield
  FWP1.rh_min = 0.20;
  FWP1.b3 = FWPs_b3;
  FWP1.b2 = 0;
  FWP1.b1 = 0;
  FWP1.a3 = FWP1_a3;
  FWP1.a2 = 0;
  FWP1.a1 = -172;

  // FWP2
  FWP2.VRotn = 4500;
  FWP2.rm = 1; // No conversion on turbopumps, so no yield
  FWP2.rh_min = 0.20;
  FWP2.b3 = FWPs_b3;
  FWP2.b2 = 0;
  FWP2.b1 = 0;
  FWP2.a3 = FWP2_a3;
  FWP2.a2 = 0;
  FWP2.a1 = -172;
  connect(ST1.C_out, ST2.C_out) annotation (Line(points={{40,100},{70,100},{70,60},{40,60}}, color={28,108,200}));
  connect(STs_sink.C_in, ST2.C_out) annotation (Line(points={{113,80},{70,80},{70,60},{40,60}}, color={28,108,200}));
  connect(ST1_CV.C_in, ST2_CV.C_in) annotation (Line(points={{-16,100},{-36,100},{-36,60},{-16,60}}, color={28,108,200}));
  connect(ST1_CV.Opening, ST1_CV_opening_sensor.Opening) annotation (Line(points={{-10,110.182},{-10,115.92}}, color={0,0,127}));
  connect(ST2_CV.Opening, ST2_CV_opening_sensor.Opening) annotation (Line(points={{-10,49.8182},{-10,44.08}}, color={0,0,127}));
  connect(STs_source.C_out, ST2_CV.C_in) annotation (Line(points={{-113,80},{-36,80},{-36,60},{-16,60}}, color={28,108,200}));
  connect(FWP2.C_in, FWP1.C_in) annotation (Line(points={{-10,-30},{60,-30},{60,-90},{-10,-90}},
                                                                                               color={28,108,200}));
  connect(FWP1.C_out, FWP2.C_out) annotation (Line(points={{-30,-90},{-40,-90},{-40,-30},{-30,-30}},
                                                                                                   color={28,108,200}));
  connect(FWPs_source.C_out, FWP1.C_in) annotation (Line(points={{113,-60},{60,-60},{60,-90},{-10,-90}},color={28,108,200}));
  connect(FWPs_sink.C_in, FWP2.C_out) annotation (Line(points={{-113,-60},{-40,-60},{-40,-30},{-30,-30}},color={28,108,200}));
  connect(ST2.C_W_out, FWP2.C_power) annotation (Line(points={{40,51.6},{62,51.6},{62,-8},{-20,-8},{-20,-19.2}},
                                                                                                               color={244,125,35}));
  connect(ST1.C_W_out, FWP1.C_power) annotation (Line(points={{40,108.4},{160,108.4},{160,-120},{-20,-120},{-20,-100.8}},
                                                                                                                        color={244,125,35}));
  connect(ST1_CV.C_out, ST1.C_in) annotation (Line(points={{-4,100},{8,100},{8,100},{20,100}}, color={28,108,200}));
  connect(ST2.C_in, ST2_CV.C_out) annotation (Line(points={{20,60},{8,60},{8,60},{-4,60}}, color={28,108,200}));
  annotation (Diagram(coordinateSystem(extent={{-140,-140},{140,140}})), Icon(coordinateSystem(extent={{-140,-140},{140,140}}), graphics={
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,0},{80,0}}),
        Line(points={{80,0},{2,60}}),
        Line(points={{80,0},{0,-60}})}));
end ParallelTurboFWP_direct;
