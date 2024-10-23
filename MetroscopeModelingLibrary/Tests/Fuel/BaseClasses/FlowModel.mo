within MetroscopeModelingLibrary.Tests.Fuel.BaseClasses;
model FlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FuelTestIcon;

  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  // Parameters
  input Units.DifferentialPressure DP(start=1e5);
  input Units.Power W(start=1e5);

  .MetroscopeModelingLibrary.Fuel.BaseClasses.FlowModel flowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
  .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
  .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
equation

  // Boundary Conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  // Parameters
  flowModel.DP = DP;
  flowModel.W = W;

  connect(source.C_out, flowModel.C_in) annotation (Line(points={{-70.5,0},{-23,0}}, color={213,213,0}));
  connect(flowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={213,213,0}));
end FlowModel;
