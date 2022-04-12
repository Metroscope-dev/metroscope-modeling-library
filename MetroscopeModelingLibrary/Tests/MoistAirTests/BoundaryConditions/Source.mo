within MetroscopeModelingLibrary.Tests.MoistAirTests.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
  import MetroscopeModelingLibrary.Units;

  // Boundary conditinos
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
  input Units.OutletMassFlowRate source_Q(start=-100) "kg/s";
  input Units.Fraction source_relative_humidity(start=0.5) "1";

  MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
  assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
  assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
  assert(abs(source.relative_humidity - sink.relative_humidity) < 1e-5, "relative_humidity should be the same from source to sink");
  connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={85,170,255}));
end Source;
