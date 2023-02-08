within MetroscopeModelingLibrary.Tests.FlueGases.Pipes;
model Leak
  import MetroscopeModelingLibrary.Utilities.Units;
  extends Utilities.Icons.Tests.FlueGasesTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=0.5e6);
  input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
  input Units.Pressure sink_P(start=1e5, min=0, nominal=10e5) "Pa";
  input Units.PositiveMassFlowRate Q(start=10) "kg/s";

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source   source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink   sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  MetroscopeModelingLibrary.FlueGases.Pipes.Leak          leak        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation

  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};
  sink.P_in = sink_P;

  // Leak:
  leak.Q = Q;

  connect(source.C_out, leak.C_in) annotation (Line(points={{-85,0},{-10,0}}, color={95,95,95}));
  connect(leak.C_out, sink.C_in) annotation (Line(points={{10,0.2},{20,0.2},{20,0},{85,0}}, color={95,95,95}));
end Leak;
