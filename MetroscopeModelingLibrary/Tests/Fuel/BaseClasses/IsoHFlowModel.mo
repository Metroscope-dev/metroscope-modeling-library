within MetroscopeModelingLibrary.Tests.Fuel.BaseClasses;
model IsoHFlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FuelTestIcon;

  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  // Parameters
  input Units.DifferentialPressure DP(start=1e5);

  MetroscopeModelingLibrary.Fuel.BaseClasses.IsoHFlowModel   isoHFlowModel
                                                                       annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
  .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
  .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
equation

  // Boundary Conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  // Parameters
  isoHFlowModel.DP = DP;

  connect(isoHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={213,213,0}));
  connect(isoHFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={213,213,0}));
end IsoHFlowModel;
