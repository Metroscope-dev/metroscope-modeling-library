within MetroscopeModelingLibrary.Tests.FlueGases.Pipes;
model ControlValve_direct
  extends Utilities.Icons.Tests.FlueGasesTestIcon;

  import MetroscopeModelingLibrary.Utilities.Units;

    // Boundary conditions
  input Units.Pressure source_P(start=10e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=0.5e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Units.Pressure sink_P(start=9e5);

  // Parameter
  parameter Units.Cv Cvmax = 7e4;

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
  MetroscopeModelingLibrary.FlueGases.Pipes.ControlValve
                                                 controlValve
                                                      annotation (Placement(transformation(extent={{-10,-4},{10,16}})));
equation

  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};
  sink.P_in = sink_P;

  // Parameters
  controlValve.Cv_max = Cvmax;

  connect(source.C_out, controlValve.C_in) annotation (Line(points={{-23,0},{-16,0},{-16,-0.363636},{-10,-0.363636}}, color={95,95,95}));
  connect(controlValve.C_out, sink.C_in) annotation (Line(points={{10,-0.363636},{16,-0.363636},{16,0},{23,0}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end ControlValve_direct;
