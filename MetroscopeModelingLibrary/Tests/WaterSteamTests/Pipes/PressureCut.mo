within MetroscopeModelingLibrary.Tests.WaterSteamTests.Pipes;
model PressureCut
  import MetroscopeModelingLibrary.Units;
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e6);
  input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
  input Units.Pressure sink_P(start=1e5, min=0, nominal=10e5) "Pa";
  input Units.OutletMassFlowRate source_Q(start=-100) "kg/s";

  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

equation

  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  sink.P_in = sink_P;

  connect(sink.C_in, pressureCut.C_out) annotation (Line(points={{85,0},{16.5,0}}, color={28,108,200}));
  connect(source.C_out, pressureCut.C_in) annotation (Line(points={{-85,0},{-16.5,0}}, color={28,108,200}));
end PressureCut;
