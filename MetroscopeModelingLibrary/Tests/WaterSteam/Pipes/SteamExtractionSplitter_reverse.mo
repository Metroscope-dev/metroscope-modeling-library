within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model SteamExtractionSplitter_reverse
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy h_source(start=2.65e6);
  input Units.Pressure source_P(start=2.64e5) "Pa";
  input Units.MassFlowRate source_Q(start=2000) "kg/s";
  input Units.MassFlowRate extracted_Q(start=100) "kg/s";

  // Input: Observables
  input Units.SpecificEnthalpy main_h_out(start=2.67e6);

  // Output: Component parameters
  output Units.Fraction alpha;

  // Components
  .MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter steamExtractionSplitter annotation (Placement(transformation(extent={{-27,-26.6667},{27,21.3333}})));

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink main_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-5.55112e-16})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink extraction_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,-70})));

equation
  // Boundary conditions
  source.h_out = h_source;
  source.P_out = source_P;
  source.Q_out = source_Q;
  extraction_sink.Q_in = extracted_Q;

  // Input: Observables
  steamExtractionSplitter.mainFlow.h_out = main_h_out;

  // Output: Component parameters
  steamExtractionSplitter.alpha = alpha;

  connect(main_sink.C_in, steamExtractionSplitter.C_main_out) annotation (Line(points={{85,0},{56,0},{56,-3.33333e-05},{28.62,-3.33333e-05}}, color={28,108,200}));
  connect(steamExtractionSplitter.C_ext_out, extraction_sink.C_in) annotation (Line(points={{0,-18.1334},{0,-41.5667},{8.88178e-16,-41.5667},{8.88178e-16,-65}}, color={28,108,200}));
  connect(steamExtractionSplitter.C_in, source.C_out) annotation (Line(points={{-28.62,-3.33333e-05},{-56.81,-3.33333e-05},{-56.81,0},{-85,0}}, color={28,108,200}));
end SteamExtractionSplitter_reverse;
