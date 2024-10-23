within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model Manifold
  import MetroscopeModelingLibrary.Utilities.Units;
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy source1_h(start=1e6);
  input Units.Pressure source1_P(start=10e5, min=0, nominal=10e5) "Pa";
  input Units.NegativeMassFlowRate source1_Q(start=-100) "kg/s";

  // Boundary conditions
  input Units.SpecificEnthalpy source2_h(start=1e6);
  input Units.Pressure source2_P(start=9e5, min=0, nominal=10e5) "Pa";
  input Units.NegativeMassFlowRate source2_Q(start=-100) "kg/s";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_1 annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={68,0})));

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_2 annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,48})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.Manifold manifold annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation

  // Boundary conditions
  source_1.h_out = source1_h;
  source_1.P_out = source1_P;
  source_1.Q_out = source1_Q;

  source_2.h_out = source2_h;
  source_2.P_out = source2_P;
  source_2.Q_out = source2_Q;

  connect(manifold.C_out, sink.C_in) annotation (Line(points={{10,0},{63,0}}, color={28,108,200}));
  connect(manifold.C_in_1, source_1.C_out) annotation (Line(points={{-10,0},{-55,0}},                  color={28,108,200}));
  connect(source_2.C_out, manifold.C_in_2) annotation (Line(points={{0,43},{0,10}},                        color={28,108,200}));
end Manifold;
