within MetroscopeModelingLibrary.Tests.Fuel.Pipes;
model PressureCut
  import MetroscopeModelingLibrary.Utilities.Units;
  extends Utilities.Icons.Tests.FuelTestIcon;

  // Boundary conditions
  input Units.Pressure source_P(start=10e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=0.5e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Units.Pressure sink_P(start=1e5, min=0, nominal=10e5) "Pa";

  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source   source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink   sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  MetroscopeModelingLibrary.Fuel.Pipes.PressureCut   pressureCut annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation

  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};
  sink.P_in = sink_P;

  connect(pressureCut.C_out, sink.C_in) annotation (Line(points={{10,0},{85,0}}, color={95,95,95}));
  connect(pressureCut.C_in, source.C_out) annotation (Line(points={{-10,0},{-85,0}}, color={95,95,95}));
end PressureCut;
