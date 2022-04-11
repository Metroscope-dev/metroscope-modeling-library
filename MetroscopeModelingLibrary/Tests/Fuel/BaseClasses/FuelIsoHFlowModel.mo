within MetroscopeModelingLibrary.Tests.Fuel.BaseClasses;
model FuelIsoHFlowModel
  extends MetroscopeModelingLibrary.Icons.Tests.FuelTestIcon;
  MetroscopeModelingLibrary.Fuel.BaseClasses.IsoHFlowModel isoHFlowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-103,-19},{-65,19}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{64,-19.5},{104,19.5}})));
equation
  isoHFlowModel.DP_input = 0;

  source.h_out = 1e6;

  source.P_out = 1e5;
  source.Q_out = -100;

  source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  assert(abs(sink.Q_in + source.Q_out) <= 1e-5, "In IsoHFlowModel, DM should be 0");
  assert(abs(sink.Q_in*sink.h_in + source.Q_out*source.h_out) <= 1e-5, "In IsoHFlowModel, W should be 0");
  connect(source.C_out, isoHFlowModel.C_in) annotation (Line(points={{-74.5,0},{-23,0}}, color={95,95,95}));
  connect(isoHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{74,0}}, color={95,95,95}));
end FuelIsoHFlowModel;
