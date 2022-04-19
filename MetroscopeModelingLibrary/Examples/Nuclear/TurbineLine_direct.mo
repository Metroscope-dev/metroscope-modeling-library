within MetroscopeModelingLibrary.Examples.Nuclear;
model TurbineLine_direct
  import MetroscopeModelingLibrary.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=6.05e6);
  input Units.SpecificEnthalpy source_h(start=2.78e6);
  input Units.NegativeMassFlowRate source_Q(start=-1000);

  // Component parameters
  // Turbines
  parameter Units.Cst ST1_Cst = 14820;
  parameter Units.Cst ST2_Cst = 2941;
  parameter Units.Cst ST3_Cst = 1362;

  parameter Units.Yield STs_eta_is = 0.847;
  parameter Units.Yield STs_eta_nz = 1;
  parameter Units.Area STs_area_nz = 1;

  // Extraction splitters
  parameter Units.Fraction ST1_ext_alpha = 0.973;
  parameter Units.Fraction ST2_ext_alpha = 0.964;
  parameter Units.Fraction ST3_ext_alpha = 1;

  // Generator
  parameter Units.Yield generator_eta = 0.99;

  // Components
  // Boundary conditions
  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{118,-10},{138,10}})));

  WaterSteam.BoundaryConditions.Sink ST1_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-66,-50})));
  WaterSteam.BoundaryConditions.Sink ST2_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={8,-50})));
  WaterSteam.BoundaryConditions.Sink ST3_ext_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={84,-50})));

  // Turbines
  WaterSteam.Machines.StodolaTurbine ST1 annotation (Placement(transformation(extent={{-112,-10},{-92,10}})));
  WaterSteam.Machines.StodolaTurbine ST2 annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  WaterSteam.Machines.StodolaTurbine ST3 annotation (Placement(transformation(extent={{38,-10},{58,10}})));

  // Extractions
  WaterSteam.Pipes.SteamExtractionSplitter ST1_ext annotation (Placement(transformation(extent={{-76,-10},{-56,8}})));
  WaterSteam.Pipes.SteamExtractionSplitter ST2_ext annotation (Placement(transformation(extent={{-2,-10},{18,8}})));
  WaterSteam.Pipes.SteamExtractionSplitter ST3_ext annotation (Placement(transformation(extent={{74,-10},{94,8}})));

  // Electricity
  Power.BoundaryConditions.Sink powerSink annotation (Placement(transformation(extent={{118,30},{138,50}})));
  Power.Machines.Generator generator annotation (Placement(transformation(extent={{78,28},{118,52}})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;

  ST1_ext_sink.Q_in = source_Q/6;
  ST2_ext_sink.Q_in = source_Q/6;
  ST3_ext_sink.Q_in = source_Q/6;

  // Turbine 1
  ST1.Cst = ST1_Cst;
  ST1.eta_is = STs_eta_is;

  // Hypothesis : no nozzle
  ST1.eta_nz = STs_eta_nz;
  ST1.area_nz = STs_area_nz;

  // Extraction 1
  ST1_ext.alpha = ST1_ext_alpha;

  // Turbine 2
  ST2.Cst = ST2_Cst;
  ST2.eta_is = STs_eta_is;

  // Hypothesis : no nozzle
  ST2.eta_nz = STs_eta_nz;
  ST2.area_nz = STs_area_nz;

  // Extraction 2
  ST2_ext.alpha = ST2_ext_alpha;

  // Turbine 3
  ST3.Cst = ST3_Cst;
  ST3.eta_is = STs_eta_is;

  // Hypothesis : no nozzle
  ST3.eta_nz = STs_eta_nz;
  ST3.area_nz = STs_area_nz;

  // Extraction 3
  ST3_ext.alpha = ST3_ext_alpha;

  // Generator
  generator.eta = generator_eta;

  connect(ST1.C_in, source.C_out) annotation (Line(points={{-112,0},{-125,0}}, color={28,108,200}));
  connect(ST1.C_out, ST1_ext.C_in) annotation (Line(points={{-92,0},{-76.6,0}},  color={28,108,200}));
  connect(ST1_ext.C_main_out, ST2.C_in) annotation (Line(points={{-55.4,0},{-40,0}}, color={28,108,200}));
  connect(ST2.C_out, ST2_ext.C_in) annotation (Line(points={{-20,0},{-2.6,0}},  color={28,108,200}));
  connect(ST2_ext.C_main_out, ST3.C_in) annotation (Line(points={{18.6,0},{38,0}}, color={28,108,200}));
  connect(ST3.C_out, ST3_ext.C_in) annotation (Line(points={{58,0},{73.4,0}}, color={28,108,200}));
  connect(ST1_ext.C_ext_out, ST1_ext_sink.C_in) annotation (Line(points={{-66,-6.8},{-66,-45}}, color={28,108,200}));
  connect(ST3_ext.C_ext_out, ST3_ext_sink.C_in) annotation (Line(points={{84,-6.8},{84,-45}}, color={28,108,200}));
  connect(ST2_ext.C_ext_out, ST2_ext_sink.C_in) annotation (Line(points={{8,-6.8},{8,-45}},                   color={28,108,200}));
  connect(powerSink.C_in, generator.C_out) annotation (Line(points={{123,40},{112,40}}, color={244,125,35}));
  connect(ST3.C_W_out, generator.C_in) annotation (Line(points={{58,8.4},{68,8.4},{68,40},{85.6,40}},  color={244,125,35}));
  connect(ST2.C_W_out, generator.C_in) annotation (Line(points={{-20,8.4},{-12,8.4},{-12,40},{85.6,40}},  color={244,125,35}));
  connect(ST1.C_W_out, generator.C_in) annotation (Line(points={{-92,8.4},{-84,8.4},{-84,40},{85.6,40}},   color={244,125,35}));
  connect(ST3_ext.C_main_out, sink.C_in) annotation (Line(points={{94.6,0},{123,0}}, color={28,108,200}));
  annotation (Diagram(coordinateSystem(extent={{-140,-140},{140,140}})), Icon(coordinateSystem(extent={{-140,-140},{140,140}}), graphics={
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
