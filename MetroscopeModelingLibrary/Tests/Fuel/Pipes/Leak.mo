within MetroscopeModelingLibrary.Tests.Fuel.Pipes;
model Leak
  import MetroscopeModelingLibrary.Utilities.Units;
  extends Utilities.Icons.Tests.FuelTestIcon;

  // Boundary conditions
  input Units.Pressure source_P(start=10e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=0.5e6) "J/kg";
  input Units.Pressure sink_P(start=1e5, min=0, nominal=10e5) "Pa";
  input Units.PositiveMassFlowRate Q(start=10) "kg/s";

  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source   source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink   sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  MetroscopeModelingLibrary.Fuel.Pipes.Leak          leak        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation

  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};
  sink.P_in = sink_P;

  // Leak:
  leak.Q = Q;

  connect(source.C_out, leak.C_in) annotation (Line(points={{-85,0},{-10,0}}, color={95,95,95}));
  connect(leak.C_out, sink.C_in) annotation (Line(points={{10,0},{20,0},{20,0},{85,0}},     color={95,95,95}));
end Leak;
