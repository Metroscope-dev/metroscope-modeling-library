within MetroscopeModelingLibrary.Tests.WaterSteamTests.Pipes;
model WaterPipeTest_direct
  import MetroscopeModelingLibrary.Units;
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;


  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e6);
  input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
  input Units.OutletMassFlowRate source_Q(start=-100) "kg/s";
  input Units.DifferentialHeight delta_z(start=1) "m";

  // Input: Component parameters
  input Units.FrictionCoefficient Kfr(start=1) "m-4";

  // Components
  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  WaterSteam.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-16.5,-16.3333},{16.5,16.3333}})));

equation
  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  pipe.delta_z = delta_z;

  // Input: Component parameters
  pipe.Kfr = Kfr;
  connect(sink.C_in, pipe.C_out) annotation (Line(points={{85,0},{16.5,0}}, color={28,108,200}));
  connect(source.C_out, pipe.C_in) annotation (Line(points={{-85,0},{-16.5,0}}, color={28,108,200}));
end WaterPipeTest_direct;
