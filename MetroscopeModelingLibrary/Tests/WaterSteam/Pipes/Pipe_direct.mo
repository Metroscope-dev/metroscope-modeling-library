within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model Pipe_direct
  import MetroscopeModelingLibrary.Utilities.Units;
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e6);
  input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  // Parameters
  parameter Units.FrictionCoefficient Kfr=1 "m-4";
  parameter Units.DifferentialHeight delta_z=1 "m";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  .MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-16.5,-16.3333},{16.5,16.3333}})));

equation

  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;

  // Parameters
  pipe.Kfr = Kfr;
  pipe.delta_z = delta_z;

  connect(sink.C_in, pipe.C_out) annotation (Line(points={{85,0},{16.5,0}}, color={28,108,200}));
  connect(source.C_out, pipe.C_in) annotation (Line(points={{-85,0},{-16.5,0}}, color={28,108,200}));
end Pipe_direct;
