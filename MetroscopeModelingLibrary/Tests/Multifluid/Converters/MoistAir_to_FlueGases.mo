within MetroscopeModelingLibrary.Tests.Multifluid.Converters;
model MoistAir_to_FlueGases
  extends MetroscopeModelingLibrary.Icons.Tests.MultifluidTestIcon;

  import MetroscopeModelingLibrary.Units;

  // Boundary conditinos
  input Units.Pressure source_P(start=5e5) "Pa";
  input Units.Temperature source_T(start=273.15+24) "K";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Units.Fraction source_relative_humidity(start=0.01) "1";
  MultiFluid.Converters.MoistAir_to_FlueGases moistAir_to_FlueGases annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{28,-10},{48,10}})));
  MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-48,-10},{-28,10}})));
equation

  source.P_out = source_P;
  source.T_out = source_T;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  connect(moistAir_to_FlueGases.outlet, sink.C_in) annotation (Line(points={{10,0},{33,0}}, color={95,95,95}));
  connect(moistAir_to_FlueGases.inlet, source.C_out) annotation (Line(points={{-10,0},{-21.5,0},{-21.5,0},{-33,0}}, color={85,170,255}));
end MoistAir_to_FlueGases;
