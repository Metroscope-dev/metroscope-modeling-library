within MetroscopeModelingLibrary.Tests.FlueGases.Pipes;
model HeatLoss
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FlueGasesTestIcon;

      // Boundary conditions
  input Utilities.Units.SpecificEnthalpy source_h(start=0.5e6);
  input Utilities.Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Utilities.Units.Power W(start=5e6) "W";

  MetroscopeModelingLibrary.FlueGases.Pipes.HeatLoss heat_loss annotation (Placement(transformation(extent={{-16,-16},{14,16}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{80,-10},{100,10}})));
equation
  // Boundary Conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};
  heat_loss.W = W;

  connect(heat_loss.C_in, source.C_out) annotation (Line(points={{-16,0},{-85,0}}, color={85,170,255}));
  connect(heat_loss.C_out, sink.C_in) annotation (Line(points={{14,0},{85,0}}, color={85,170,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end HeatLoss;
