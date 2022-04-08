within MetroscopeModelingLibrary.Tests.Power.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Icons.Tests.PowerTestIcon;
  import MetroscopeModelingLibrary.Units;

  // Boundary conditions
  input Units.OutletPower source_W(start=-1e6) "W";

  MetroscopeModelingLibrary.Power.BoundaryConditions.PowerSource source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.PowerSink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation
  source.W_out = source_W;

  assert(abs(source.W_out + sink.W_in) < 1e-5, "Power should be the same from source to sink");
  connect(sink.C_in, source.C_out) annotation (Line(points={{23,0},{-23.2,0}}, color={244,125,35}));
end Source;
