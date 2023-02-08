within MetroscopeModelingLibrary.Examples.Nuclear.MainSteam;
model TurbineLPCondenser_direct
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Real source_P(start=15.5, unit="bar", nominal=20, min=0, max=200) "barA";
  input Units.SpecificEnthalpy source_h(start=2.9e6);
  input Units.PositiveMassFlowRate source_Q(start=1150);

  input Units.PositiveMassFlowRate LP_turbine1_ext_Q(start=78);
  input Units.PositiveMassFlowRate LP_turbine2_ext_Q(start=75);

  input Real P_cold_source(start=5, min=0, nominal=10) "barA";
  input Real T_cold_source(start=15, min=0, nominal = 50) "degC";

  // Hypothesis on component parameters
  // Turbines
  parameter Units.Area LP_turbines_area_nz = 12; // pi*turbine_R**2 - pi*rotor_R**2, with turbine_R = 2m, rotor_R = 0.35m
  parameter Units.Yield LP_turbines_eta_is = 0.9;

  // Extraction splitters
  parameter Units.Fraction LP_turbine1_ext_alpha = 1;
  parameter Units.Fraction LP_turbine2_ext_alpha = 1;
  parameter Units.Fraction LP_turbine3_ext_alpha = 1;

  // Condenser
  parameter Units.Area S = 100;
  parameter Units.Height water_height = 1;
  parameter Real C_incond(unit="mol/m3") = 0;
  parameter Units.Pressure P_offset = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 1;
  //parameter Units.VolumeFlowRate Qv_cold = 3.82;

  // Generator
  parameter Units.Yield generator_eta = 0.99;

  // Quantities definition
  output Real LP_turbine1_ext_P;
  output Real LP_turbine2_ext_P;
  output Real condenser_Psat;
  output Real cooling_sink_T;

  output Real W_tot;

  // Calibrated parameters
  // Turbines
  parameter Units.Cst LP_turbine1_Cst = 3072.6655;
  parameter Units.Cst LP_turbine2_Cst = 583.2574;
  parameter Units.Cst LP_turbine3_Cst = 85.04979;

  parameter Units.Yield LP_turbine3_eta_nz = 0.99673736;

  parameter Units.HeatExchangeCoefficient condenser_Kth = 1627299.0;

  // Components
  // Boundary conditions
  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-180,-10},{-160,10}})));
  Sensors.WaterSteam.PressureSensor source_P_sensor(Q_0=1150)
                                                    annotation (Placement(transformation(extent={{-160,-10},{-140,10}})));
  Sensors.WaterSteam.FlowSensor source_Q_sensor(Q_0=1150)
                                                annotation (Placement(transformation(extent={{-134,-10},{-114,10}})));

  // Turbines
  WaterSteam.Machines.StodolaTurbine LP_turbine1(
    P_in_0=1500000,
    P_out_0=600000,
    h_in_0=2.9e6,
    Q_0=1150)                                    annotation (Placement(transformation(extent={{-106,-10},{-86,10}})));
  WaterSteam.Machines.StodolaTurbine LP_turbine2(
    P_in_0=600000,
    P_out_0=150000,
    Q_0=1150)                                    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  WaterSteam.Machines.StodolaTurbine LP_turbine3(P_in_0=150000, P_out_0=5080)
                                                 annotation (Placement(transformation(extent={{38,-10},{58,10}})));

  // Extractions
  WaterSteam.Pipes.SteamExtractionSplitter LP_turbine1_ext annotation (Placement(transformation(extent={{-76,-10},{-56,8}})));
  WaterSteam.Pipes.SteamExtractionSplitter LP_turbine2_ext annotation (Placement(transformation(extent={{-2,-10},{18,8}})));
  Sensors.WaterSteam.FlowSensor LP_turbine1_ext_Q_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-66,-50})));
  Sensors.WaterSteam.FlowSensor LP_turbine2_ext_Q_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={8,-50})));
  Sensors.WaterSteam.PressureSensor LP_turbine1_ext_P_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-66,-25})));
  Sensors.WaterSteam.PressureSensor condenser_Psat_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,0})));
  Sensors.WaterSteam.PressureSensor LP_turbine2_ext_P_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={8,-25})));
  WaterSteam.BoundaryConditions.Sink LP_turbine1_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-66,-76})));
  WaterSteam.BoundaryConditions.Sink LP_turbine2_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={8,-78})));

  // Electricity
  Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{122,30},{142,50}})));
  Power.Machines.Generator generator annotation (Placement(transformation(extent={{68,28},{108,52}})));
  Sensors.Power.PowerSensor W_tot_sensor annotation (Placement(transformation(extent={{108,34},{120,46}})));
  // Condenser
  WaterSteam.HeatExchangers.Condenser condenser(faulty=false,
    Psat_0=5000,
    P_cold_in_0=500000)                                       annotation (Placement(transformation(extent={{114,-60.6667},{146,-34}})));
  WaterSteam.BoundaryConditions.Source cooling_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={50,-42})));
  WaterSteam.BoundaryConditions.Sink cooling_sink annotation (Placement(transformation(extent={{170,-60},{190,-40}})));
  Sensors.WaterSteam.TemperatureSensor cooling_source_T_sensor annotation (Placement(transformation(extent={{60,-52},{80,-32}})));
  Sensors.WaterSteam.PressureSensor cooling_source_P_sensor annotation (Placement(transformation(extent={{86,-52},{106,-32}})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={130,-102})));
  Sensors.WaterSteam.TemperatureSensor cooling_sink_T_sensor annotation (Placement(transformation(extent={{150,-60},{170,-40}})));
equation
  // Boundary conditions
  source_P_sensor.P_barA = source_P;
  source_Q_sensor.Q = source_Q;
  source.h_out = source_h;

  LP_turbine1_ext_Q_sensor.Q = LP_turbine1_ext_Q;
  LP_turbine2_ext_Q_sensor.Q = LP_turbine2_ext_Q;

  // Turbine 1
  LP_turbine1.Cst = LP_turbine1_Cst;
  LP_turbine1.eta_is = LP_turbines_eta_is;

  // Hypothesis : no nozzle
  LP_turbine1.eta_nz = 1;
  LP_turbine1.area_nz = LP_turbines_area_nz;

  // Extraction 1
  LP_turbine1_ext_P_sensor.P_barA = LP_turbine1_ext_P; // Calibrates LP_turbine1_Cst
  LP_turbine1_ext.alpha = LP_turbine1_ext_alpha;

  // Turbine 2
  LP_turbine2.Cst = LP_turbine2_Cst;
  LP_turbine2.eta_is = LP_turbines_eta_is;

  // Hypothesis : no nozzle
  LP_turbine2.eta_nz = 1;
  LP_turbine2.area_nz = LP_turbines_area_nz;

  // Extraction 2
  LP_turbine2_ext_P_sensor.P_barA = LP_turbine2_ext_P; // Calibrates LP_turbine2_Cst
  LP_turbine2_ext.alpha = LP_turbine2_ext_alpha;

  // Turbine 3
  LP_turbine3.Cst = LP_turbine3_Cst;
  LP_turbine3.eta_is = LP_turbines_eta_is;

  // Hypothesis : no nozzle
  LP_turbine3.eta_nz = LP_turbine3_eta_nz;
  LP_turbine3.area_nz = LP_turbines_area_nz;

  // Generator
  W_tot_sensor.W_MW = W_tot; // Calibrates LP_turbines_eta_is
  // Hypothesis
  generator.eta = generator_eta;

  // Condenser
  // Boundary conditions
  cooling_source_P_sensor.P_barA = P_cold_source;
  cooling_source_T_sensor.T_degC = T_cold_source;

  // Parameters
  condenser.S = S;
  condenser.water_height = water_height;
  condenser.C_incond = C_incond;
  condenser.P_offset = P_offset;
  condenser.Kfr_cold = Kfr_cold;

  // Calibrated Parameters
  condenser.Kth = condenser_Kth;
  //condenser.Q_cold = condenser_Q_cold;

  // Quantities definition
  condenser_Psat_sensor.P_mbar = condenser_Psat;
  cooling_sink_T_sensor.T_degC = cooling_sink_T;

  connect(LP_turbine1.C_out, LP_turbine1_ext.C_in) annotation (Line(points={{-86,0},{-76.6,0}},  color={28,108,200}));
  connect(LP_turbine1_ext.C_main_out, LP_turbine2.C_in) annotation (Line(points={{-55.4,0},{-40,0}}, color={28,108,200}));
  connect(LP_turbine2.C_out, LP_turbine2_ext.C_in) annotation (Line(points={{-20,0},{-2.6,0}},  color={28,108,200}));
  connect(LP_turbine2_ext.C_main_out, LP_turbine3.C_in) annotation (Line(points={{18.6,0},{38,0}}, color={28,108,200}));
  connect(LP_turbine3.C_W_out, generator.C_in) annotation (Line(points={{58,8.4},{68,8.4},{68,40},{75.6,40}},  color={244,125,35}));
  connect(LP_turbine2.C_W_out, generator.C_in) annotation (Line(points={{-20,8.4},{-12,8.4},{-12,40},{75.6,40}},  color={244,125,35}));
  connect(LP_turbine1.C_W_out, generator.C_in) annotation (Line(points={{-86,8.4},{-74,8.4},{-74,40},{75.6,40}},   color={244,125,35}));
  connect(LP_turbine1_ext.C_ext_out, LP_turbine1_ext_P_sensor.C_in) annotation (Line(points={{-66,-6.8},{-66,-15}}, color={28,108,200}));
  connect(LP_turbine2_ext.C_ext_out, LP_turbine2_ext_P_sensor.C_in) annotation (Line(points={{8,-6.8},{8,-15}}, color={28,108,200}));
  connect(powerSink.C_in, W_tot_sensor.C_out) annotation (Line(points={{127,40},{119.88,40}}, color={244,125,35}));
  connect(W_tot_sensor.C_in, generator.C_out) annotation (Line(points={{108,40},{102,40}}, color={244,125,35}));
  connect(source_P_sensor.C_in, source.C_out) annotation (Line(points={{-160,0},{-165,0}}, color={28,108,200}));
  connect(source_P_sensor.C_out, source_Q_sensor.C_in) annotation (Line(points={{-140,0},{-134,0}}, color={28,108,200}));
  connect(LP_turbine1.C_in, source_Q_sensor.C_out) annotation (Line(points={{-106,0},{-114,0}}, color={28,108,200}));
  connect(LP_turbine1_ext_P_sensor.C_out, LP_turbine1_ext_Q_sensor.C_in) annotation (Line(points={{-66,-35},{-66,-40}}, color={28,108,200}));
  connect(LP_turbine1_ext_Q_sensor.C_out, LP_turbine1_ext_sink.C_in) annotation (Line(points={{-66,-60},{-66,-71}}, color={28,108,200}));
  connect(LP_turbine2_ext_P_sensor.C_out, LP_turbine2_ext_Q_sensor.C_in) annotation (Line(points={{8,-35},{8,-40}}, color={28,108,200}));
  connect(LP_turbine2_ext_Q_sensor.C_out, LP_turbine2_ext_sink.C_in) annotation (Line(points={{8,-60},{8,-73}}, color={28,108,200}));
  connect(LP_turbine3.C_out, condenser_Psat_sensor.C_in) annotation (Line(points={{58,0},{80,0}}, color={28,108,200}));
  connect(condenser.C_hot_in, condenser_Psat_sensor.C_out) annotation (Line(points={{130,-34},{130,0},{100,0}},      color={28,108,200}));
  connect(cooling_source.C_out, cooling_source_T_sensor.C_in) annotation (Line(points={{55,-42},{60,-42}}, color={28,108,200}));
  connect(cooling_source_T_sensor.C_out, cooling_source_P_sensor.C_in) annotation (Line(points={{80,-42},{86,-42}}, color={28,108,200}));
  connect(condenser.C_cold_in, cooling_source_P_sensor.C_out) annotation (Line(points={{114,-42.8889},{112.68,-42.8889},{112.68,-42},{106,-42}},    color={28,108,200}));
  connect(sink.C_in, condenser.C_hot_out) annotation (Line(points={{130,-97},{130,-60.6667}}, color={28,108,200}));
  connect(condenser.C_cold_out, cooling_sink_T_sensor.C_in) annotation (Line(points={{146,-48.8148},{149,-48.8148},{149,-50},{150,-50}}, color={28,108,200}));
  connect(cooling_sink.C_in, cooling_sink_T_sensor.C_out) annotation (Line(points={{175,-50},{170,-50}}, color={28,108,200}));
  annotation (Diagram(coordinateSystem(extent={{-180,-140},{200,140}})), Icon(coordinateSystem(extent={{-100,-100},{100,100}}), graphics={
                               Polygon(
          points={{-100,60},{-100,40},{-100,-40},{-100,-60},{-80,-66},{80,-100},{100,-100},{100,-80},{100,77.539},{100,100},{80,100},{-80,68},{-100,60}},
          lineColor={63,81,181},
          lineThickness=0.5,
          smooth=Smooth.Bezier),
                               Polygon(
          points={{-92,58},{-92,40},{-92,-40},{-92,-54},{-74,-60},{72,-90},{92,-94},{92,-72},{92,70},{92,92},{72,90},{-72,62},{-92,58}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={207,211,237},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{66,86},{66,-86}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{22,78},{22,-78}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-20,68},{-20,-68}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-60,60},{-60,-58}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Rectangle(
          extent={{74,2},{-76,-2}},
          lineThickness=0.5,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}));
end TurbineLPCondenser_direct;
