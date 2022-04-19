within MetroscopeModelingLibrary.Tests.WaterSteamTests.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;
  import MetroscopeModelingLibrary.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;

  assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
  assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
  assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
  connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={28,108,200}));
end Source;
