within MetroscopeModelingLibrary.Tests.FlueGases.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Icons.Tests.FlueGasesTestIcon;
  import MetroscopeModelingLibrary.Units;

  // Boundary conditinos
  input Units.Pressure sink_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy sink_h(start=1e6) "J/kg";
  input Units.PositiveMassFlowRate sink_Q(start=100) "kg/s";

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation
  sink.P_in = sink_P;
  sink.h_in = sink_h;
  sink.Q_in = sink_Q;

  sink.Xi_in = {0.768,0.232,0.0,0.0,0.0};

  connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={95,95,95}));
end Sink;
