within MetroscopeModelingLibrary.Tests.WaterSteamTests.Pipes;
model SteamExtractionSplitter_direct
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy h_source(start=2.65e6);
  input Real source_P(start=2.64, min=0, nominal=10) "barA";
  input Units.MassFlowRate source_Q(start=2000) "kg/s";
  input Units.MassFlowRate extracted_Q(start=100) "kg/s";

  // Input: Component parameters
  input Units.Fraction alpha(start=0.8);

  // Components
  WaterSteam.Pipes.SteamExtractionSplitter steamExtractionSplitter annotation (Placement(transformation(extent={{-27,-26.6667},{27,21.3333}})));

  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  WaterSteam.BoundaryConditions.Sink main_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-5.55112e-16})));
  WaterSteam.BoundaryConditions.Sink extraction_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,-70})));

  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor source_P_sensor annotation (Placement(transformation(extent={{-76,-6},{-64,6}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor source_Q_sensor annotation (Placement(transformation(extent={{-56,-6},{-44,6}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor extracted_Q_sensor annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={0,-44})));
equation
  // Boundary conditions
  source.h_out = h_source;
  source_P_sensor.P_barA = source_P;
  source_Q_sensor.Q = source_Q;
  extracted_Q_sensor.Q = extracted_Q;

  // Input: Component parameters
  steamExtractionSplitter.alpha = alpha;
  connect(source_P_sensor.C_in, source.C_out) annotation (Line(points={{-76,0},{-80,0},{-80,0},{-85,0}},   color={28,108,200}));
  connect(steamExtractionSplitter.C_in, source_Q_sensor.C_out) annotation (Line(points={{-28.62,-3.33333e-05},{-44.8,-3.33333e-05},{-44.8,0},{-44,0}}, color={28,108,200}));
  connect(source_Q_sensor.C_in, source_P_sensor.C_out) annotation (Line(points={{-56,0},{-58,0},{-58,0.06},{-60,0.06},{-60,0},{-64,0}}, color={28,108,200}));
  connect(main_sink.C_in, steamExtractionSplitter.C_main_out) annotation (Line(points={{85,0},{56,0},{56,-3.33333e-05},{28.62,-3.33333e-05}}, color={28,108,200}));
  connect(steamExtractionSplitter.C_ext_out, extracted_Q_sensor.C_in) annotation (Line(points={{0,-18.1334},{0,-38},{1.10328e-15,-38}}, color={28,108,200}));
  connect(extracted_Q_sensor.C_out, extraction_sink.C_in) annotation (Line(points={{-1.10328e-15,-50},{-1.10328e-15,-60.5},{8.88178e-16,-60.5},{8.88178e-16,-65}}, color={28,108,200}));
end SteamExtractionSplitter_direct;
