within MetroscopeModelingLibrary.Tests.Power.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Icons.Tests.PowerTestIcon;
  import MetroscopeModelingLibrary.Units;

  // Boundary conditinos
  input Units.InletPower sink_W(start=1e6) "W";

  MetroscopeModelingLibrary.Power.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation
  sink.W_in = sink_W;

  assert(abs(source.W_out + sink.W_in) < 1e-5, "Power should be the same from source to sink");
  connect(source.C_out, sink.C_in) annotation (Line(points={{-23.2,0},{23,0}},
                                                                             color={28,108,200}));
end Sink;
