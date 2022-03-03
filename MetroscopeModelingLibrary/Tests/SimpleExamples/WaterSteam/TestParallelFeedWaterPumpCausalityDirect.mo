within MetroscopeModelingLibrary.Tests.SimpleExamples.WaterSteam;
model TestParallelFeedWaterPumpCausalityDirect
  extends Modelica.Icons.Example;

  // BC
  //STs_CV
  input Real STs_CV_P_in(start=15) "barA"; // 11 ??
  //Real STs_CV_P_in;
  //STs
  input Real STs_P_out(start=0.07) "condenser pressure, barA";
  // FWPs
  input Real FWPs_P_in(start=44.6) "barA";//44.6
  //Real FWPs_P_in;
  input Real FWPs_Q_in(start=1.5e3) "kg/s";
  input Real FWPs_T_in(start=186) "degC";
  input Real ST1_CV_opening(start=15) "barA";
  input Real ST2_CV_opening(start=15) "barA";

  // Observables
  // STs_CV
  output Real STs_CV_Q_in; //13.4
  //Real STs_CV_Q_in;
  // STs
  output Real ST1_P_in;
  output Real ST2_P_in;
  // FWPs
  output Real FWP1_Q_in; //783.5
  //Real FWP1_Q_in;
  output Real FWP1_VRot; // 4253
  //Real FWP1_VRot;
  output Real FWP2_VRot; // 4206
  //Real FWP2_VRot;
  output Real FWPs_T_out; //186.7
  //Real FWPs_T_out;
  output Real FWPs_P_out; //82
  //Real FWPs_P_out;

  // Component characteristics
  // STs_CV
  parameter Real STs_CVmax = 129.65929;
  // STs
  parameter Real ST1_eta_is = 1.0269433;
  parameter Real ST2_eta_is = 0.92677194;
  parameter Real ST1_Cst = 49300252.0;
  parameter Real ST2_Cst = 49300252.0;
  // FWPs
  parameter Real FWP1_a3 = 634.53723;
  parameter Real FWP2_a3 = 623.4584;
  parameter Real FWPs_b3 = 0.8617299;
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source STs_source
    annotation (Placement(transformation(extent={{-136,28},{-116,48}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink STs_sink
    annotation (Placement(transformation(extent={{166,28},{186,48}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor STs_CV_P_in_sensor
    annotation (Placement(transformation(extent={{-108,28},{-88,48}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine ST1
    annotation (Placement(transformation(extent={{40,10},{60,-10}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    STs_P_out_sensor
    annotation (Placement(transformation(extent={{124,28},{144,48}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine ST2
    annotation (Placement(transformation(extent={{40,64},{60,84}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source FWPs_source
    annotation (Placement(transformation(extent={{188,-114},{168,-94}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink FWPs_sink
    annotation (Placement(transformation(extent={{-116,-112},{-136,-92}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    FWPs_P_in_sensor
    annotation (Placement(transformation(extent={{136,-114},{116,-94}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor
    FWPs_T_in_sensor
    annotation (Placement(transformation(extent={{108,-114},{88,-94}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StaticCentrifugalPump FWP1
    annotation (Placement(transformation(extent={{10,-78},{-10,-58}})));
  MetroscopeModelingLibrary.Common.Sensors.RotSpeedSensor FWP1_VRot_sensor
    annotation (Placement(transformation(extent={{10,-102},{30,-82}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor FWPs_P_out_sensor
    annotation (Placement(transformation(extent={{-36,-112},{-56,-92}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor
    FWPs_T_out_sensor
    annotation (Placement(transformation(extent={{-74,-112},{-94,-92}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor FWP1_Q_in_sensor
    annotation (Placement(transformation(extent={{48,-78},{28,-58}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StaticCentrifugalPump FWP2
    annotation (Placement(transformation(extent={{10,-128},{-10,-148}})));
  MetroscopeModelingLibrary.Common.Sensors.RotSpeedSensor FWP2_VRot_sensor
    annotation (Placement(transformation(extent={{10,-106},{30,-126}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor ST1_P_in_sensor
    annotation (Placement(transformation(extent={{10,10},{30,-10}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve ST1_CV
    annotation (Placement(transformation(extent={{-20,4},{0,-18}})));
  MetroscopeModelingLibrary.Common.Sensors.OpeningSensor ST1_CV_opening_sensor
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={-38,-30})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor ST2_P_in_sensor
    annotation (Placement(transformation(extent={{10,64},{30,84}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve ST2_CV
    annotation (Placement(transformation(extent={{-20,70},{0,92}})));
  MetroscopeModelingLibrary.Common.Sensors.OpeningSensor ST2_CV_opening_sensor
    annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={-38,104})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor STs_CV_Q_in_sensor
    annotation (Placement(transformation(extent={{-78,28},{-58,48}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor FWPs_Q_in_sensor
    annotation (Placement(transformation(extent={{162,-94},{142,-114}})));
equation
  //STs_CV_P_in = time * 15 + (1-time)*25;
  //FWPs_P_in = time * 44.6 + (1-time)*44;
  //STs_CV_Q_in = time * 13.4 + (1-time)*1500;
  //FWP1_Q_in = time * 7.5e2 + (1-time)*783.5;
  //FWP1_VRot = time * 4253 + (1-time)*4000;
  //FWP2_VRot = time * 4206 + (1-time)*4000;
  //FWPs_T_out = time * 186.7 + (1-time)*186.4;
  //FWPs_P_out = time * 82+ (1-time)*69;

  // ------------- REVERSE ------------- //
  // STs source
  STs_source.h_out = 2.7718e6; // set Temp ?
  STs_CV_Q_in_sensor.Q = STs_CV_Q_in;
  STs_CV_P_in_sensor.P_barA = STs_CV_P_in;

  // ST1_CV
  ST1_CV_opening_sensor.Opening = ST1_CV_opening;
  ST1_CV.Cvmax = STs_CVmax;

  // ST2_CV
  ST2_CV_opening_sensor.Opening = ST2_CV_opening;

  // Hyp on CVs : same mass flow
  ST1_CV.Cvmax = ST2_CV.Cvmax;
  //ST1_CV.Q_in = ST2_CV.Q_in;

  // ST1
  ST1_P_in_sensor.P_barA = ST1_P_in;
  ST1.eta_nz = 1.0;
  ST1.area_nz = 1.0;
  ST1.eta_is = ST1_eta_is;
  ST1.Cst = ST1_Cst;

  // ST2
  ST2_P_in_sensor.P_barA = ST2_P_in;
  ST2.eta_nz = 1.0;
  ST2.area_nz = 1.0;
  ST2.eta_is = ST2_eta_is;
  ST2.Cst = ST2_Cst;

  // STs sink
  STs_P_out_sensor.P_barA = STs_P_out;
  STs_sink.h_vol = 1e6;

  // FWPs source
  FWPs_T_in_sensor.T_degC = FWPs_T_in;
  FWPs_P_in_sensor.P_barA = FWPs_P_in;
  FWPs_Q_in_sensor.Q = FWPs_Q_in;

  // FWP1
  FWP1_Q_in_sensor.Q = FWP1_Q_in;
  FWP1_VRot_sensor.VRot = FWP1_VRot;
  FWP1.VRotn = 4500;
  FWP1.rm = 0.85;
  FWP1.rhmin = 0.20;
  FWP1.b3 = FWPs_b3;
  FWP1.b2 = 0;
  FWP1.b1 = 0;
  FWP1.a3 = FWP1_a3;
  FWP1.a2 = 0;
  FWP1.a1 = -172;

  // FWP2
  FWP2_VRot_sensor.VRot = FWP2_VRot;
  FWP2.VRotn = 4500;
  FWP2.rm = 0.85;
  FWP2.rhmin = 0.20;
  FWP2.b2 = 0;
  FWP2.b1 = 0;
  FWP2.a3 = FWP2_a3;
  FWP2.a2 = 0;
  FWP2.a1 = -172;

  // Hyp on both pumps
  //FWP1.a3 = FWP2.a3; // same contribution to pressure
  FWP1.b3 = FWP2.b3; // same contribution to enthalpy

  // FWPs sink
  FWPs_T_out_sensor.T_degC = FWPs_T_out;
  FWPs_P_out_sensor.P_barA = FWPs_P_out;

  FWPs_sink.h_vol = 1e6;
  // --------------------------------------- //
  connect(STs_source.C_out, STs_CV_P_in_sensor.C_in)
    annotation (Line(points={{-116,38},{-108,38}}, color={63,81,181}));
  connect(STs_P_out_sensor.C_out,STs_sink. C_in)
    annotation (Line(points={{144.2,38},{166,38}},
                                                 color={63,81,181}));
  connect(ST2.C_out,ST1. C_out) annotation (Line(points={{60.2,74},{72,74},{72,0},
          {60.2,0}},       color={63,81,181}));
  connect(STs_P_out_sensor.C_in,ST1. C_out) annotation (Line(points={{124,38},{72,
          38},{72,0},{60.2,0}},    color={63,81,181}));
  connect(FWPs_P_in_sensor.C_out,FWPs_T_in_sensor. C_in)
    annotation (Line(points={{115.8,-104},{108,-104}},
                                                   color={63,81,181}));
  connect(FWP1_VRot_sensor.VRot,FWP1. VRot)
    annotation (Line(points={{9.2,-92},{0,-92},{0,-80}},    color={0,0,0}));
  connect(FWPs_P_out_sensor.C_in,FWP1. C_out)
    annotation (Line(points={{-36,-102},{-20,-102},{-20,-68},{-10.2,-68}},
                                                   color={63,81,181}));
  connect(FWPs_sink.C_in,FWPs_T_out_sensor. C_out)
    annotation (Line(points={{-116,-102},{-94.2,-102}},
                                                     color={63,81,181}));
  connect(FWPs_T_out_sensor.C_in,FWPs_P_out_sensor. C_out)
    annotation (Line(points={{-74,-102},{-56.2,-102}},
                                                     color={63,81,181}));
  connect(FWP1.C_in,FWP1_Q_in_sensor. C_out)
    annotation (Line(points={{10,-68},{27.8,-68}}, color={63,81,181}));
  connect(FWP1_Q_in_sensor.C_in, FWPs_T_in_sensor.C_out) annotation (Line(
        points={{48,-68},{64,-68},{64,-104},{87.8,-104}},
                                                        color={63,81,181}));
  connect(FWP2_VRot_sensor.VRot, FWP2.VRot)
    annotation (Line(points={{9.2,-116},{0,-116},{0,-126}}, color={0,0,0}));
  connect(FWP2.C_out, FWP1.C_out) annotation (Line(points={{-10.2,-138},{-20,-138},
          {-20,-68},{-10.2,-68}}, color={63,81,181}));
  connect(ST1.C_power, FWP1.C_power) annotation (Line(
      points={{61.4,-8.6},{92,-8.6},{92,-48},{0,-48},{0,-56.8}},
      color={0,0,0},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(ST2.C_power, FWP2.C_power) annotation (Line(
      points={{61.4,82.6},{61.4,82},{198,82},{198,-160},{0,-160},{0,-149.2}},
      color={0,0,0},
      thickness=0.5,
      pattern=LinePattern.Dash));
  connect(ST1_CV.Opening,ST1_CV_opening_sensor. Opening)
    annotation (Line(points={{-10,-18.2},{-10,-30},{-27.2,-30}},
                                                      color={0,0,127}));
  connect(ST2_P_in_sensor.C_in,ST2_CV. C_out)
    annotation (Line(points={{10,74},{0.2,74}},
                                              color={63,81,181}));
  connect(ST2_CV.Opening,ST2_CV_opening_sensor. Opening) annotation (Line(
        points={{-10,92.2},{-10,104},{-27.2,104}},  color={0,0,127}));
  connect(ST2.C_in, ST2_P_in_sensor.C_out)
    annotation (Line(points={{40,74},{30.2,74}}, color={63,81,181}));
  connect(ST1_CV.C_out, ST1_P_in_sensor.C_in)
    annotation (Line(points={{0.2,0},{10,0}}, color={63,81,181}));
  connect(ST1.C_in, ST1_P_in_sensor.C_out)
    annotation (Line(points={{40,0},{30.2,0}}, color={63,81,181}));
  connect(ST1_CV.C_in, ST2_CV.C_in) annotation (Line(points={{-20,0},{-50,0},{-50,
          74},{-20,74}}, color={63,81,181}));
  connect(STs_CV_P_in_sensor.C_out, STs_CV_Q_in_sensor.C_in)
    annotation (Line(points={{-87.8,38},{-78,38}}, color={63,81,181}));
  connect(STs_CV_Q_in_sensor.C_out, ST2_CV.C_in) annotation (Line(points={{-57.8,
          38},{-50,38},{-50,74},{-20,74}}, color={63,81,181}));
  connect(FWPs_P_in_sensor.C_in, FWPs_Q_in_sensor.C_out)
    annotation (Line(points={{136,-104},{141.8,-104}}, color={63,81,181}));
  connect(FWPs_Q_in_sensor.C_in, FWPs_source.C_out)
    annotation (Line(points={{162,-104},{168,-104}}, color={63,81,181}));
  connect(FWP2.C_in, FWPs_T_in_sensor.C_out) annotation (Line(points={{10,-138},
          {64,-138},{64,-104},{87.8,-104}}, color={63,81,181}));
  annotation (Diagram(coordinateSystem(extent={{-200,-180},{240,140}})), Icon(
        coordinateSystem(extent={{-200,-180},{240,140}})));
end TestParallelFeedWaterPumpCausalityDirect;
