within MetroscopeModelingLibrary.Tests.RefMoistAir.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditinos
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Units.Fraction source_relative_humidity(start=0.5) "1";

  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.relative_humidity = source_relative_humidity;

  connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={0,255,128}));
end Source;
