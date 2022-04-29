within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model HeatLoss
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.Pressure source_P(start=2e5, min=0, nominal=2) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
  input Units.Power W(start=1e5) "W";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  .MetroscopeModelingLibrary.WaterSteam.Pipes.HeatLoss heat_loss annotation (Placement(transformation(extent={{-16.5,-16.3333},{16.5,16.3333}})));

equation

  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  heat_loss.W = W;

  connect(sink.C_in, heat_loss.C_out) annotation (Line(points={{85,0},{16.5,0}}, color={28,108,200}));
  connect(source.C_out, heat_loss.C_in) annotation (Line(points={{-85,0},{-16.5,0}}, color={28,108,200}));
end HeatLoss;
