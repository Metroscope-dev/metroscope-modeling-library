within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model PressureCut
  import MetroscopeModelingLibrary.Utilities.Units;
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e6);
  input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
  input Units.Pressure sink_P(start=1e5, min=0, nominal=10e5) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  .MetroscopeModelingLibrary.WaterSteam.Pipes.PressureCut pressureCut annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation

  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  sink.P_in = sink_P;

  connect(source.C_out, pressureCut.C_in) annotation (Line(points={{-85,0},{-10,0}}, color={28,108,200}));
  connect(pressureCut.C_out, sink.C_in) annotation (Line(points={{10,0},{85,0}}, color={28,108,200}));
end PressureCut;
