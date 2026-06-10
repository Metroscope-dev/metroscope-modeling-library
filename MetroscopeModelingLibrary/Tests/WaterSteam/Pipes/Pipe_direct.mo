within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model Pipe_direct
  import MetroscopeModelingLibrary.Utilities.Units;
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e6);
  input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  .MetroscopeModelingLibrary.WaterSteam.Pipes.FrictionPipe pipe annotation (Placement(transformation(extent={{-16.5,-16.3333},{16.5,16.3333}})));

  Utilities.Interfaces.RealExpression Kfr(y=100) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={0,40}), iconTransformation(extent={{-236,8},{-216,28}})));
equation

  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;


  connect(sink.C_in, pipe.C_out) annotation (Line(points={{85,0},{16.5,0}}, color={28,108,200}));
  connect(source.C_out, pipe.C_in) annotation (Line(points={{-85,0},{-16.5,0}}, color={28,108,200}));
  connect(Kfr.y, pipe.Kfr) annotation (Line(points={{0,38},{0,6.53332}}, color={0,0,127}));
end Pipe_direct;
