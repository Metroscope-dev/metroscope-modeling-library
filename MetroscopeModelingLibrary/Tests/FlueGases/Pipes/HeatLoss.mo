within MetroscopeModelingLibrary.Tests.FlueGases.Pipes;
model HeatLoss
  extends MetroscopeModelingLibrary.Icons.Tests.FlueGasesTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.Pressure source_P(start=2e5, min=0, nominal=2) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
  input Units.Power W(start=1e5) "W";

  .MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  .MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  .MetroscopeModelingLibrary.FlueGases.Pipes.HeatLoss heat_loss annotation (Placement(transformation(extent={{-16.5,-16.3333},{16.5,16.3333}})));

equation

  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  heat_loss.W = W;

  connect(sink.C_in, heat_loss.C_out) annotation (Line(points={{85,0},{16.5,0}}, color={95,95,95}));
  connect(source.C_out, heat_loss.C_in) annotation (Line(points={{-85,0},{-16.5,0}}, color={95,95,95}));
end HeatLoss;
