within MetroscopeModelingLibrary.Tests.Power.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.PowerTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.NegativePower source_W(start=-1e6) "W";

  MetroscopeModelingLibrary.Power.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation

  source.W_out = source_W;

  connect(sink.C_in, source.C_out) annotation (Line(points={{23,0},{-23.2,0}}, color={244,125,35}));
end Source;
