within MetroscopeModelingLibrary.Tests.Fuel.BaseClasses;
model FuelFlowModel
  extends MetroscopeModelingLibrary.Icons.Tests.FuelTestIcon;
  MetroscopeModelingLibrary.Fuel.BaseClasses.FlowModel flowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{61,-20},{103,20}})));
equation
  flowModel.W_input = 0;
  flowModel.DP_input = 0;

  source.h_out = 1e6;

  source.P_out = 1e5;
  source.Q_out = -100;

  source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  assert(abs(sink.Q_in + source.Q_out) <= 1e-5, "In flow model, DM should be 0");
  connect(sink.C_in, flowModel.C_out) annotation (Line(points={{71.5,0},{23,0}}, color={95,95,95}));
  connect(flowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={95,95,95}));
end FuelFlowModel;
