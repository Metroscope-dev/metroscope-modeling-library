within MetroscopeModelingLibrary.Tests.MoistAir.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
  import MetroscopeModelingLibrary.Units;

  // Boundary conditinos
  input Units.Pressure sink_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy sink_h(start=1e3) "J/kg";
  input Units.PositiveMassFlowRate sink_Q(start=100) "kg/s";
  input Units.Fraction sink_relative_humidity(start=0.5) "1";

  .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation
  sink.P_in = sink_P;
  sink.h_in = sink_h;
  sink.Q_in = sink_Q;
  sink.relative_humidity = sink_relative_humidity;

  connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={85,170,255}));
end Sink;
