within MetroscopeModelingLibrary.Examples.Nuclear.FeedWater;
model ParallelTurboFWP_direct
  import MetroscopeModelingLibrary.Utilities.Units;

  // Initialization parameters
  parameter Units.Pressure STs_CV_P_in_0 = 30e5;
  parameter Units.Pressure STs_P_out_0 = 0.07e5;
  parameter Units.Pressure FWPs_P_in_0 = 44.6e5;
  // BC
  // Observables used for calibration
  // Component characteristics
  // FWPs
  parameter Real FWP1_a3 = 579.387573242;
  parameter Real FWP2_a3 = 623.458618164;
  parameter Real FWPs_b3 = 0.86172991991;

  // Components
  // ST BCs
  WaterSteam.BoundaryConditions.Source STs_source annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-175,80}, rotation=0)));
  WaterSteam.BoundaryConditions.Sink STs_sink annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={155,80}, rotation=0)));
  // STs
  SensorsControl.WaterSteam.PressureSensor ST1_P_in_sensor(sensor_function="Calibration",
    causality="ST1_Cst",
    P_start=10,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(10),
    Q_0=6.7,
    h_0=2.7718e6) annotation (Placement(transformation(extent={{10,93.0001},{24,107}})));
  SensorsControl.WaterSteam.PressureSensor ST2_P_in_sensor(sensor_function="Calibration",
    causality="ST2_Cst",
    P_start=10,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(10),
    Q_0=6.7,
    h_0=2.7718e6) annotation (Placement(transformation(extent={{10,53},{24,67}})));
  WaterSteam.Machines.SteamTurbine ST1(P_out_0=STs_P_out_0,
    P_in_0=1000000,
    Q_0=6.7,
    h_in_0=2771800,
    h_out_0=2197456.25) annotation (Placement(transformation(extent={{32,90.0002},{52,110}})));
  WaterSteam.Machines.SteamTurbine ST2(P_out_0=STs_P_out_0,
    P_in_0=1000000,
    Q_0=6.7,
    h_in_0=2771800,
    h_out_0=2246570.5) annotation (Placement(transformation(extent={{32,70},{52,50}})));
  // STs CV
  WaterSteam.Pipes.ControlValve ST2_CV(P_in_0=STs_CV_P_in_0) annotation (Placement(transformation(extent={{-16,62.5455},{-4,48.5455}})));
  WaterSteam.Pipes.ControlValve ST1_CV(P_in_0=STs_CV_P_in_0) annotation (Placement(transformation(extent={{-16,97.4545},{-4,111.455}})));
  SensorsControl.Outline.OpeningSensor ST1_CV_opening_sensor(sensor_function="BC",
    causality="",
    Opening_start=70,
    signal_unit="%",
    Opening_pc_0=70) annotation (Placement(transformation(extent={{-14,116},{-6,124}})));
  SensorsControl.Outline.OpeningSensor ST2_CV_opening_sensor(sensor_function="BC",
    causality="",
    Opening_start=70,
    signal_unit="%",
    Opening_pc_0=70) annotation (Placement(transformation(extent={{-14,44},{-6,36}})));

  // FWP BCs
  WaterSteam.BoundaryConditions.Source FWPs_source annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={185,-60}, rotation=180)));
  WaterSteam.BoundaryConditions.Sink FWPs_sink annotation (Placement(transformation(extent={{-108,-70},{-128,-50}})));
  // Pumps
  WaterSteam.Machines.Pump FWP2(P_in_0=FWPs_P_in_0,
    P_out_0=8200000,
    Q_0=716.5,
    T_in_0=459.15,
    T_out_0=459.85,
    h_in_0=791283.0,
    h_out_0=796194.4375,
    VRot(start=4206),
    hn(start=431.57699585)) annotation (Placement(transformation(extent={{-10,-40},{-30,-20}})));
  WaterSteam.Machines.Pump FWP1(P_in_0=FWPs_P_in_0,
    P_out_0=8200000,
    Q_0=783.5,
    T_in_0=459.15,
    T_out_0=459.85,
    h_in_0=791283.0,
    h_out_0=796194.4375,
    VRot(start=4253),
    hn(start=431.57699585)) annotation (Placement(transformation(extent={{-10,-80},{-30,-100}})));
  SensorsControl.WaterSteam.FlowSensor STs_CV_Q_in_sensor(sensor_function="Calibration",
    causality="STs_CVmax",
    Q_start=13.4,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=30e5,
    Q_0=13.4,
    h_0=2.7718e6) annotation (Placement(transformation(extent={{-76,73},{-62,87}})));
  RotationalSpeedSensor FWP1_VRot_sensor(sensor_function="Calibration",
    causality="ST1_eta_is",
    VRot_start=4253) annotation (Placement(transformation(
        extent={{-7.5,-7.5},{7.5,7.5}},
        rotation=270,
        origin={0,-70})));
  RotationalSpeedSensor FWP2_VRot_sensor(sensor_function="Calibration",
    causality="ST2_eta_is",
    VRot_start=4206) annotation (Placement(transformation(
        extent={{-7.5,-7.5},{7.5,7.5}},
        rotation=270,
        origin={0,-50})));
  // Pumps outlet
  SensorsControl.WaterSteam.TemperatureSensor FWPs_T_out_sensor(sensor_function="Calibration",
    causality="FWPs_b3",
    T_start=186.7,
    signal_unit="degC",
    display_unit="degC",
    P_0=82e5,
    Q_0=1500,
    h_0=790000,
    T_0=273.15 + (186.7)) annotation (Placement(transformation(extent={{-84,-67},{-98,-53}})));
  SensorsControl.WaterSteam.PressureSensor FWPs_P_out_sensor(sensor_function="Calibration",
    causality="FWP2_a3",
    P_start=82,
    signal_unit="barA",
    display_unit="barA",
    P_0=1e5*(82),
    Q_0=1500,
    h_0=790000) annotation (Placement(transformation(extent={{-64,-67},{-78,-53}})));
  SensorsControl.WaterSteam.FlowSensor FWP1_Q_in_sensor(sensor_function="Calibration",
    causality="FWP1_a3",
    Q_start=783.5,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=44.6e5,
    Q_0=783.5,
    h_0=790000) annotation (Placement(transformation(extent={{38,-97},{24,-83}})));
  // MML4 interfaces: quantities are declared at their connected instruments.
  Utilities.Interfaces.BoundaryCondition STs_CV_P_in annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-135,108})));
  Utilities.Interfaces.BoundaryCondition STs_P_out annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={110,108})));
  Utilities.Interfaces.BoundaryCondition FWPs_P_in annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={150,-88})));
  Utilities.Interfaces.BoundaryCondition FWPs_T_in annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={120,-88})));
  Utilities.Interfaces.BoundaryCondition FWPs_Q_in annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={90,-88})));
  Utilities.Interfaces.BoundaryCondition ST1_CV_opening annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-10,142.08})));
  Utilities.Interfaces.BoundaryCondition ST2_CV_opening annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-10,17.92})));
  Utilities.Interfaces.Observable STs_CV_Q_in annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-69,105})));
  Utilities.Interfaces.Observable ST1_P_in annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={17,125})));
  Utilities.Interfaces.Observable ST2_P_in annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={17,85})));
  Utilities.Interfaces.Observable FWPs_T_out annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-91,-35})));
  Utilities.Interfaces.Observable FWPs_P_out annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-71,-35})));
  Utilities.Interfaces.Observable FWP1_Q_in annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={31,-65})));
  Utilities.Interfaces.Observable FWP1_VRot annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={25.65,-70})));
  Utilities.Interfaces.Observable FWP2_VRot annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={25.65,-50})));
  Utilities.Interfaces.RealOutput STs_CVmax annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-30.4,96.6549})));
  Utilities.Interfaces.RealOutput ST1_eta_is annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={65,115})));
  Utilities.Interfaces.RealOutput ST1_Cst annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={28,128})));
  Utilities.Interfaces.RealOutput ST2_eta_is annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={65,35})));
  Utilities.Interfaces.RealOutput ST2_Cst annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={28,25})));
  SensorsControl.WaterSteam.PressureSensor STs_CV_P_in_sensor(sensor_function="BC",
    causality="",
    P_start=STs_CV_P_in_0/1e5,
    signal_unit="barA",
    display_unit="barA",
    P_0=STs_CV_P_in_0,
    Q_0=13.4,
    h_0=2.7718e6) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-135,80}, rotation=0)));
  SensorsControl.WaterSteam.PressureSensor STs_P_out_sensor(sensor_function="BC",
    causality="",
    P_start=STs_P_out_0/1e5,
    signal_unit="barA",
    display_unit="barA",
    P_0=STs_P_out_0,
    Q_0=13.4,
    h_0=1.9e6) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={110,80}, rotation=0)));
  SensorsControl.WaterSteam.PressureSensor FWPs_P_in_sensor(sensor_function="BC",
    causality="",
    P_start=FWPs_P_in_0/1e5,
    signal_unit="barA",
    display_unit="barA",
    P_0=FWPs_P_in_0,
    Q_0=1500,
    h_0=790000) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={150,-60}, rotation=180)));
  SensorsControl.WaterSteam.TemperatureSensor FWPs_T_in_sensor(sensor_function="BC",
    causality="",
    T_start=186,
    signal_unit="degC",
    display_unit="degC",
    P_0=FWPs_P_in_0,
    T_0=459.15,
    Q_0=1500,
    h_0=790000) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={120,-60}, rotation=180)));
  SensorsControl.WaterSteam.FlowSensor FWPs_Q_in_sensor(sensor_function="BC",
    causality="",
    Q_start=1500,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=FWPs_P_in_0,
    Q_0=1500,
    h_0=790000) annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={90,-60}, rotation=180)));

equation
  // Boundary conditions
  // STs source
  STs_source.h_out = 2.7718e6; // set Temp ?

  // ST1 CV

  // ST2_CV

  // STs sink

  // FWPs

  // STs_CV
  // Observables used for calibration

  // ST1_CV // Hyp on CVs : same Cv (or same mass flow)
  // Calibrated  parameters

  // ST1_CV
  // Calibrated parameters

  // ST1
  // Observables used for calibration

  // Calibrated parameters

  // ST2
  // Observables used for calibration

  // Calibrated parameters

  // FWPs
  // Observables used for calibration

  // FWP1
  // Observables used for calibration

  // Calibrated parameters
  FWP1.b3 = FWPs_b3;
  FWP1.a3 = FWP1_a3;
  // fixed parameters
  FWP1.VRotn = 4300;
  FWP1.rm = 1; // No conversion on turbopumps, so no yield
  FWP1.rh_min = 0.20;
  FWP1.b2 = 0;
  FWP1.b1 = 0;
  FWP1.a2 = 0;
  FWP1.a1 = -172;

  // FWP2
  // Observables used for calibration

  // Calibrated parameters
  FWP2.b3 = FWPs_b3;
  FWP2.a3 = FWP2_a3;
  // fixed parameters
  FWP2.VRotn = 4500;
  FWP2.rm = 1; // No conversion on turbopumps, so no yield
  FWP2.rh_min = 0.20;
  FWP2.b2 = 0;
  FWP2.b1 = 0;
  FWP2.a2 = 0;
  FWP2.a1 = -172;
  connect(ST1.C_out, ST2.C_out) annotation (Line(points={{52,100.0001},{70,100.0001},{70,100},{70,60},{52,60}}, color={28,108,200}));
  connect(ST1_CV.C_in, ST2_CV.C_in) annotation (Line(points={{-16,100.2546},{-36,100.2546},{-36,100},{-36,60},{-16,60},{-16,59.7455}}, color={28,108,200}));
  connect(ST1_CV.Opening, ST1_CV_opening_sensor.Opening) annotation (Line(points={{-10,111.455},{-10,115.92}}, color={0,0,127}));
  connect(ST2_CV.Opening, ST2_CV_opening_sensor.Opening) annotation (Line(points={{-10,48.5455},{-10,44.08}}, color={0,0,127}));
  connect(FWP1.C_out, FWP2.C_out) annotation (Line(points={{-30,-90},{-40,-90},{-40,-30},{-30,-30}},
                                                                                                   color={28,108,200}));
  connect(ST2.C_W_out, FWP2.C_power) annotation (Line(points={{52,51.6},{62,51.6},{62,-8},{-20,-8},{-20,-19.2}},
                                                                                                               color={244,125,35}));
  connect(ST1.C_W_out, FWP1.C_power) annotation (Line(points={{52,108.400016},{160,108.400016},{160,108.4},{160,-120},{-20,-120},{-20,-100.8}},
                                                                                                                        color={244,125,35}));
  connect(STs_CV_Q_in_sensor.C_out, ST2_CV.C_in) annotation (Line(points={{-62,80},{-36,80},{-36,60},{-16,60},{-16,59.7455}}, color={28,108,200}));
  connect(FWP1.VRot, FWP1_VRot_sensor.VRot) annotation (Line(points={{-20,-78},{-20,-70},{-7.65,-70}}, color={0,0,127}));
  connect(FWP2.VRot, FWP2_VRot_sensor.VRot) annotation (Line(points={{-20,-42},{-20,-50},{-7.65,-50}}, color={0,0,127}));
  connect(FWPs_P_out_sensor.C_in, FWP2.C_out) annotation (Line(points={{-64,-60},{-40,-60},{-40,-30},{-30,-30}}, color={28,108,200}));
  connect(FWPs_sink.C_in, FWPs_T_out_sensor.C_out) annotation (Line(points={{-113,-60},{-98,-60}}, color={28,108,200}));
  connect(FWPs_T_out_sensor.C_in, FWPs_P_out_sensor.C_out) annotation (Line(points={{-84,-60},{-78,-60}}, color={28,108,200}));
  connect(ST1_CV.C_out, ST1_P_in_sensor.C_in) annotation (Line(points={{-4,100.2546},{0,100.2546},{0,100},{4,100},{10,100},{10,100.00005}},
                                                                                           color={28,108,200}));
  connect(ST1_P_in_sensor.C_out, ST1.C_in) annotation (Line(points={{24,100.00005},{26,100.00005},{26,100},{28,100},{32,100},{32,100.0001}},
                                                                                        color={28,108,200}));
  connect(ST2.C_in, ST2_P_in_sensor.C_out) annotation (Line(points={{32,60},{24,60}}, color={28,108,200}));
  connect(ST2_P_in_sensor.C_in, ST2_CV.C_out) annotation (Line(points={{10,60},{4,60},{-4,60},{-4,59.7455}}, color={28,108,200}));
  connect(FWP1_Q_in_sensor.C_out, FWP1.C_in) annotation (Line(points={{24,-90},{-10,-90}}, color={28,108,200}));
  connect(FWP2.C_in, FWP1_Q_in_sensor.C_in) annotation (Line(points={{-10,-30},{60,-30},{60,-90},{38,-90}}, color={28,108,200}));

  // Instrument signals and connected characteristics.
  connect(STs_source.C_out, STs_CV_P_in_sensor.C_in) annotation (Line(points={{-170,80},{-145,80}}, color={28,108,200}));
  connect(STs_CV_P_in_sensor.C_out, STs_CV_Q_in_sensor.C_in) annotation (Line(points={{-125,80},{-76,80}}, color={28,108,200}));
  connect(ST2.C_out, STs_P_out_sensor.C_in) annotation (Line(points={{52,60},{100,60},{100,80}}, color={28,108,200}));
  connect(STs_P_out_sensor.C_out, STs_sink.C_in) annotation (Line(points={{120,80},{150,80}}, color={28,108,200}));
  connect(FWPs_source.C_out, FWPs_P_in_sensor.C_in) annotation (Line(points={{180,-60},{160,-60}}, color={28,108,200}));
  connect(FWPs_P_in_sensor.C_out, FWPs_T_in_sensor.C_in) annotation (Line(points={{140,-60},{130,-60}}, color={28,108,200}));
  connect(FWPs_T_in_sensor.C_out, FWPs_Q_in_sensor.C_in) annotation (Line(points={{110,-60},{100,-60}}, color={28,108,200}));
  connect(FWPs_Q_in_sensor.C_out, FWP1_Q_in_sensor.C_in) annotation (Line(points={{80,-60},{38,-60},{38,-90}}, color={28,108,200}));
  connect(STs_CV_P_in_sensor.P_sensor, STs_CV_P_in) annotation (Line(points={{-135,90},{-135,108}}, color={0,0,127}));
  connect(STs_P_out_sensor.P_sensor, STs_P_out) annotation (Line(points={{110,90},{110,108}}, color={0,0,127}));
  connect(FWPs_P_in_sensor.P_sensor, FWPs_P_in) annotation (Line(points={{150,-70},{150,-88}}, color={0,0,127}));
  connect(FWPs_T_in_sensor.T_sensor, FWPs_T_in) annotation (Line(points={{120,-70},{120,-88}}, color={0,0,127}));
  connect(FWPs_Q_in_sensor.Q_sensor, FWPs_Q_in) annotation (Line(points={{90,-70},{90,-88}}, color={0,0,127}));
  connect(ST1_CV_opening_sensor.opening_sensor, ST1_CV_opening) annotation (Line(points={{-10,124.08},{-10,142.08}}, color={0,0,127}));
  connect(ST2_CV_opening_sensor.opening_sensor, ST2_CV_opening) annotation (Line(points={{-10,35.92},{-10,17.92}}, color={0,0,127}));
  connect(STs_CV_Q_in_sensor.Q_sensor, STs_CV_Q_in) annotation (Line(points={{-69,87},{-69,105}}, color={0,0,127}));
  connect(ST1_P_in_sensor.P_sensor, ST1_P_in) annotation (Line(points={{17,107},{17,125}}, color={0,0,127}));
  connect(ST2_P_in_sensor.P_sensor, ST2_P_in) annotation (Line(points={{17,67},{17,85}}, color={0,0,127}));
  connect(FWPs_T_out_sensor.T_sensor, FWPs_T_out) annotation (Line(points={{-91,-53},{-91,-35}}, color={0,0,127}));
  connect(FWPs_P_out_sensor.P_sensor, FWPs_P_out) annotation (Line(points={{-71,-53},{-71,-35}}, color={0,0,127}));
  connect(FWP1_Q_in_sensor.Q_sensor, FWP1_Q_in) annotation (Line(points={{31,-83},{31,-65}}, color={0,0,127}));
  connect(FWP1_VRot_sensor.VRot_sensor, FWP1_VRot) annotation (Line(points={{7.65,-70},{25.65,-70}}, color={0,0,127}));
  connect(FWP2_VRot_sensor.VRot_sensor, FWP2_VRot) annotation (Line(points={{7.65,-50},{25.65,-50}}, color={0,0,127}));
  connect(STs_CVmax, ST1_CV.Cv_max) annotation (Line(points={{-30.4,96.6549},{-12.4,96.6549},{-12.4,108.655}}, color={0,0,127}));
  connect(STs_CVmax, ST2_CV.Cv_max) annotation (Line(points={{-30.4,96.6549},{-12.4,96.6549},{-12.4,51.3455}}, color={0,0,127}));
  STs_CVmax = 583.716125488;
  connect(ST1_eta_is, ST1.eta_is) annotation (Line(points={{65,115},{40,115},{40,92.0002}}, color={0,0,127}));
  ST1_eta_is = 0.852958083153;
  connect(ST1_Cst, ST1.Cst) annotation (Line(points={{28,128},{36,128},{36,93.0002}}, color={0,0,127}));
  ST1_Cst = 49300252;
  connect(ST2_eta_is, ST2.eta_is) annotation (Line(points={{65,35},{40,35},{40,68}}, color={0,0,127}));
  ST2_eta_is = 0.771477103233;
  connect(ST2_Cst, ST2.Cst) annotation (Line(points={{28,25},{36,25},{36,67}}, color={0,0,127}));
  ST2_Cst = 49300252;
  annotation (Diagram(coordinateSystem(extent={{-210,-170},{210,180}})), Icon(coordinateSystem(extent={{-140,-140},{140,140}}), graphics={
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,0},{80,0}}),
        Line(points={{80,0},{2,60}}),
        Line(points={{80,0},{0,-60}})}));
end ParallelTurboFWP_direct;
