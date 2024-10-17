within MetroscopeModelingLibrary.Tests.RefMoistAir.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditinos
  input Units.Pressure sink_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy sink_h(start=1e3) "J/kg";
  input Units.PositiveMassFlowRate sink_Q(start=100) "kg/s";
  input Units.Fraction sink_relative_humidity(start=0.5) "1";

  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation
  sink.P_in = sink_P;
  sink.h_in = sink_h;
  sink.Q_in = sink_Q;
  sink.relative_humidity = sink_relative_humidity;

  connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={0,255,128}));
end Sink;
