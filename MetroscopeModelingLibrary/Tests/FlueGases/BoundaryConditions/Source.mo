within MetroscopeModelingLibrary.Tests.FlueGases.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FlueGasesTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;

  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={95,95,95}));
end Source;
