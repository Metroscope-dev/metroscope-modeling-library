within MetroscopeModelingLibrary.Tests.FlueGases.BaseClasses;
model FlowModel
  extends MetroscopeModelingLibrary.Icons.Tests.FlueGasesTestIcon;

  import MetroscopeModelingLibrary.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=10e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  // Parameters
  input Units.DifferentialPressure DP(start=1e5);
  input Units.Power W(start=1e5);

  .MetroscopeModelingLibrary.FlueGases.BaseClasses.FlowModel flowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
  .MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
  .MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
equation

  // Boundary Conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  // Parameters
  flowModel.DP = DP;
  flowModel.W = W;

  connect(flowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={95,95,95}));
  connect(flowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={95,95,95}));
end FlowModel;
