within MetroscopeModelingLibrary.Tests.WaterSteam.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;

  connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={28,108,200}));
end Source;
