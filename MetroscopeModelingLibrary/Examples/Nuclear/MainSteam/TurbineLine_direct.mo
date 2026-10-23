within MetroscopeModelingLibrary.Examples.Nuclear.MainSteam;
model TurbineLine_direct
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions

  input Units.SpecificEnthalpy source_h(start=2.78e6);

  // Generator
  parameter Units.Yield generator_eta = 0.99;

  // Calibrated parameters
  // Components
  // Boundary conditions
  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-180,-10},{-160,10}})));
  SensorsControl.WaterSteam.PressureSensor source_P_sensor(sensor_function="BC",
    causality="",
    P_start=67,
    signal_unit="barA",
    display_unit="barA",
    P_0=6700000,
    Q_0=1600,
    h_0=2780000) annotation (Placement(transformation(extent={{-160,-10},{-140,10}})));
  SensorsControl.WaterSteam.FlowSensor source_Q_sensor(sensor_function="BC",
    causality="",
    Q_start=1600,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=6700000,
    Q_0=1600,
    h_0=2780000) annotation (Placement(transformation(extent={{-134,-10},{-114,10}})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{122,-10},{142,10}})));

  // Turbines
  WaterSteam.Machines.SteamTurbine ST1 annotation (Placement(transformation(extent={{-106,-10},{-86,10}})));
  WaterSteam.Machines.SteamTurbine ST2 annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  WaterSteam.Machines.SteamTurbine ST3 annotation (Placement(transformation(extent={{38,-10},{58,10}})));

  // Extractions
  WaterSteam.Pipes.SteamExtractionSplitter ST1_ext annotation (Placement(transformation(extent={{-76,-10},{-56,8}})));
  WaterSteam.Pipes.SteamExtractionSplitter ST2_ext annotation (Placement(transformation(extent={{-2,-10},{18,8}})));
  WaterSteam.Pipes.SteamExtractionSplitter ST3_ext annotation (Placement(transformation(extent={{74,-10},{94,8}})));
  SensorsControl.WaterSteam.FlowSensor ST1_ext_Q_sensor(sensor_function="BC",
    causality="",
    Q_start=67,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=4100000,
    Q_0=67,
    h_0=2650000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-66,-50})));
  SensorsControl.WaterSteam.FlowSensor ST2_ext_Q_sensor(sensor_function="BC",
    causality="",
    Q_start=95,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=2800000,
    Q_0=95,
    h_0=2550000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={8,-50})));
  SensorsControl.WaterSteam.FlowSensor ST3_ext_Q_sensor(sensor_function="BC",
    causality="",
    Q_start=140,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=1600000,
    Q_0=140,
    h_0=2450000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={84,-50})));
  SensorsControl.WaterSteam.PressureSensor ST1_ext_P_sensor(sensor_function="Calibration",
    causality="ST1_Cst",
    P_start=41,
    signal_unit="barA",
    display_unit="barA",
    P_0=4100000,
    Q_0=67,
    h_0=2650000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-66,-25})));
  SensorsControl.WaterSteam.PressureSensor ST3_ext_P_sensor(sensor_function="Calibration",
    causality="ST3_Cst",
    P_start=16,
    signal_unit="barA",
    display_unit="barA",
    P_0=1600000,
    Q_0=140,
    h_0=2450000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={84,-25})));
  SensorsControl.WaterSteam.PressureSensor ST2_ext_P_sensor(sensor_function="Calibration",
    causality="ST2_Cst",
    P_start=28,
    signal_unit="barA",
    display_unit="barA",
    P_0=2800000,
    Q_0=95,
    h_0=2550000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={8,-25})));
  WaterSteam.BoundaryConditions.Sink ST1_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-66,-78})));
  WaterSteam.BoundaryConditions.Sink ST2_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={8,-78})));
  WaterSteam.BoundaryConditions.Sink ST3_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={84,-78})));

  // Electricity
  Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{122,30},{142,50}})));
  Power.Machines.Generator generator(eta=generator_eta) annotation (Placement(transformation(extent={{68,28},{108,52}})));
  SensorsControl.Power.PowerSensor W_tot_sensor(sensor_function="Calibration",
    causality="STs_eta_is",
    W_start=320,
    signal_unit="MW",
    display_unit="MW") annotation (Placement(transformation(extent={{108,34},{120,46}})));

  // MML4 instrument interfaces and component characteristics.
  Utilities.Interfaces.BoundaryCondition source_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-150,30})));
  Utilities.Interfaces.BoundaryCondition source_Q annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-124,30})));
  Utilities.Interfaces.BoundaryCondition ST1_ext_Q annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-36,-50})));
  Utilities.Interfaces.BoundaryCondition ST2_ext_Q annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={38,-50})));
  Utilities.Interfaces.BoundaryCondition ST3_ext_Q annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={114,-50})));
  Utilities.Interfaces.Observable ST1_ext_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-36,-25})));
  Utilities.Interfaces.Observable ST2_ext_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={38,-25})));
  Utilities.Interfaces.Observable ST3_ext_P annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={114,-25})));
  Utilities.Interfaces.Observable W_tot annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={114,66})));
  Utilities.Interfaces.RealOutput ST1_Cst annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-108,-44})));
  Utilities.Interfaces.RealOutput STs_eta_is annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-115,-110})));
  Utilities.Interfaces.RealOutput ST1_ext_alpha annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-46,24})));
  Utilities.Interfaces.RealOutput ST2_Cst annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={-42,-44})));
  Utilities.Interfaces.RealOutput ST2_ext_alpha annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={28,24})));
  Utilities.Interfaces.RealOutput ST3_Cst annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={36,-44})));
  Utilities.Interfaces.RealOutput ST3_ext_alpha annotation (Placement(iconVisible=false, transformation(extent={{-4,-4},{4,4}}, origin={104,24})));

equation
  // Boundary conditions

  source.h_out = source_h;

  connect(ST1.C_out, ST1_ext.C_in) annotation (Line(points={{-86,0},{-76.6,0},{-76.6,-1}},  color={28,108,200}));
  connect(ST1_ext.C_main_out, ST2.C_in) annotation (Line(points={{-55.4,-1},{-40,-1},{-40,0}}, color={28,108,200}));
  connect(ST2.C_out, ST2_ext.C_in) annotation (Line(points={{-20,0},{-2.6,0},{-2.6,-1}},  color={28,108,200}));
  connect(ST2_ext.C_main_out, ST3.C_in) annotation (Line(points={{18.6,-1},{38,-1},{38,0}}, color={28,108,200}));
  connect(ST3.C_out, ST3_ext.C_in) annotation (Line(points={{58,0},{73.4,0},{73.4,-1}}, color={28,108,200}));
  connect(ST3.C_W_out, generator.C_in) annotation (Line(points={{58,8.4},{68,8.4},{68,40},{75.6,40}},  color={244,125,35}));
  connect(ST2.C_W_out, generator.C_in) annotation (Line(points={{-20,8.4},{-12,8.4},{-12,40},{75.6,40}},  color={244,125,35}));
  connect(ST1.C_W_out, generator.C_in) annotation (Line(points={{-86,8.4},{-84,8.4},{-84,40},{75.6,40}},   color={244,125,35}));
  connect(ST3_ext.C_main_out, sink.C_in) annotation (Line(points={{94.6,-1},{127,-1},{127,0}}, color={28,108,200}));
  connect(ST1_ext.C_ext_out, ST1_ext_P_sensor.C_in) annotation (Line(points={{-66,-7.12},{-66,-15}}, color={28,108,200}));
  connect(ST2_ext.C_ext_out, ST2_ext_P_sensor.C_in) annotation (Line(points={{8,-7.12},{8,-15}}, color={28,108,200}));
  connect(ST3_ext.C_ext_out, ST3_ext_P_sensor.C_in) annotation (Line(points={{84,-7.12},{84,-15}}, color={28,108,200}));
  connect(powerSink.C_in, W_tot_sensor.C_out) annotation (Line(points={{127,40},{119.88,40}}, color={244,125,35}));
  connect(W_tot_sensor.C_in, generator.C_out) annotation (Line(points={{108,40},{102,40}}, color={244,125,35}));
  connect(source_P_sensor.C_in, source.C_out) annotation (Line(points={{-160,0},{-165,0}}, color={28,108,200}));
  connect(source_P_sensor.C_out, source_Q_sensor.C_in) annotation (Line(points={{-140,0},{-134,0}}, color={28,108,200}));
  connect(ST1.C_in, source_Q_sensor.C_out) annotation (Line(points={{-106,0},{-114,0}}, color={28,108,200}));
  connect(ST1_ext_P_sensor.C_out, ST1_ext_Q_sensor.C_in) annotation (Line(points={{-66,-35},{-66,-40}}, color={28,108,200}));
  connect(ST1_ext_Q_sensor.C_out, ST1_ext_sink.C_in) annotation (Line(points={{-66,-60},{-66,-73}}, color={28,108,200}));
  connect(ST3_ext_P_sensor.C_out, ST3_ext_Q_sensor.C_in) annotation (Line(points={{84,-35},{84,-40}}, color={28,108,200}));
  connect(ST3_ext_Q_sensor.C_out, ST3_ext_sink.C_in) annotation (Line(points={{84,-60},{84,-73}}, color={28,108,200}));
  connect(ST2_ext_P_sensor.C_out, ST2_ext_Q_sensor.C_in) annotation (Line(points={{8,-35},{8,-40}}, color={28,108,200}));
  connect(ST2_ext_Q_sensor.C_out, ST2_ext_sink.C_in) annotation (Line(points={{8,-60},{8,-73}}, color={28,108,200}));

  // MML4 signal connections and fixed characteristics.
  connect(source_P_sensor.P_sensor, source_P) annotation (Line(points={{-150,10},{-150,30}}, color={0,0,127}));
  connect(source_Q_sensor.Q_sensor, source_Q) annotation (Line(points={{-124,10},{-124,30}}, color={0,0,127}));
  connect(ST1_ext_Q_sensor.Q_sensor, ST1_ext_Q) annotation (Line(points={{-56,-50},{-36,-50}}, color={0,0,127}));
  connect(ST2_ext_Q_sensor.Q_sensor, ST2_ext_Q) annotation (Line(points={{18,-50},{38,-50}}, color={0,0,127}));
  connect(ST3_ext_Q_sensor.Q_sensor, ST3_ext_Q) annotation (Line(points={{94,-50},{114,-50}}, color={0,0,127}));
  connect(ST1_ext_P_sensor.P_sensor, ST1_ext_P) annotation (Line(points={{-56,-25},{-36,-25}}, color={0,0,127}));
  connect(ST2_ext_P_sensor.P_sensor, ST2_ext_P) annotation (Line(points={{18,-25},{38,-25}}, color={0,0,127}));
  connect(ST3_ext_P_sensor.P_sensor, ST3_ext_P) annotation (Line(points={{94,-25},{114,-25}}, color={0,0,127}));
  connect(W_tot_sensor.W_sensor, W_tot) annotation (Line(points={{114,46},{114,66}}, color={0,0,127}));
  connect(ST1_Cst, ST1.Cst) annotation (Line(points={{-108,-44},{-102,-44},{-102,-21},{-102,-7}}, color={0,0,127}));
  ST1_Cst = 19941.871;
  connect(STs_eta_is, ST1.eta_is) annotation (Line(points={{-115,-110},{-115,-106},{-98,-106},{-98,-8}}, color={0,0,127}));
  connect(STs_eta_is, ST2.eta_is) annotation (Line(points={{-115,-110},{-115,-106},{-32,-106},{-32,-8}}, color={0,0,127}));
  connect(STs_eta_is, ST3.eta_is) annotation (Line(points={{-115,-110},{-115,-106},{46,-106},{46,-8}}, color={0,0,127}));
  STs_eta_is = 0.88584805;
  connect(ST1_ext_alpha, ST1_ext.alpha) annotation (Line(points={{-46,24},{-66,24},{-66,-9.6},{-58,-9.6},{-58,1.16}}, color={0,0,127}));
  ST1_ext_alpha = 1;
  connect(ST2_Cst, ST2.Cst) annotation (Line(points={{-42,-44},{-36,-44},{-36,-21},{-36,-7}}, color={0,0,127}));
  ST2_Cst = 7814.647;
  connect(ST2_ext_alpha, ST2_ext.alpha) annotation (Line(points={{28,24},{8,24},{8,-9.6},{16,-9.6},{16,1.16}}, color={0,0,127}));
  ST2_ext_alpha = 1;
  connect(ST3_Cst, ST3.Cst) annotation (Line(points={{36,-44},{42,-44},{42,-21},{42,-7}}, color={0,0,127}));
  ST3_Cst = 5626.26;
  connect(ST3_ext_alpha, ST3_ext.alpha) annotation (Line(points={{104,24},{84,24},{84,-9.6},{92,-9.6},{92,1.16}}, color={0,0,127}));
  ST3_ext_alpha = 1;
  annotation (Diagram(coordinateSystem(extent={{-180,-140},{160,140}})), Icon(coordinateSystem(extent={{-180,-140},{160,140}}), graphics={
                               Polygon(
          points={{-104,62},{-104,42},{-104,-38},{-104,-58},{-84,-64},{76,-98},{96,-98},{96,-78},{96,79.539},{96,102},{76,102},{-84,70},{-104,62}},
          lineColor={63,81,181},
          lineThickness=0.5,
          smooth=Smooth.Bezier),
                               Polygon(
          points={{-96,60},{-96,42},{-96,-38},{-96,-52},{-78,-58},{68,-88},{88,-92},{88,-70},{88,72},{88,94},{68,92},{-76,64},{-96,60}},
          lineThickness=0.5,
          smooth=Smooth.Bezier,
          fillColor={207,211,237},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{62,88},{62,-84}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{18,80},{18,-76}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-24,70},{-24,-66}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Line(
          points={{-64,62},{-64,-56}},
          color={157,166,218},
          thickness=0.5,
          smooth=Smooth.Bezier),
        Rectangle(
          extent={{70,4},{-80,0}},
          lineThickness=0.5,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}));
end TurbineLine_direct;
