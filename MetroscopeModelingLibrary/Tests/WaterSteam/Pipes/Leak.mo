within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model Leak
  import MetroscopeModelingLibrary.Units;
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e6);
  input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
  input Units.Pressure sink_P(start=1e5, min=0, nominal=10e5) "Pa";
  input Units.PositiveMassFlowRate Q(start=10) "kg/s";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  MetroscopeModelingLibrary.WaterSteam.Pipes.Leak         leak        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation

  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  sink.P_in = sink_P;

  // Leak:
  leak.Q = Q;

  connect(source.C_out, leak.C_in) annotation (Line(points={{-85,0},{-10,0}}, color={28,108,200}));
  connect(leak.C_out, sink.C_in) annotation (Line(points={{10,0.2},{48,0.2},{48,0},{85,0}}, color={28,108,200}));
end Leak;
