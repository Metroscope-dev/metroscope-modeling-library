within MetroscopeModelingLibrary.Tests.Fuel.BaseClasses;
model FuelIsoPHFlowModel
  MetroscopeModelingLibrary.Fuel.BaseClasses.FuelIsoPHFlowModel isoPHFlowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.FuelSource source annotation (Placement(transformation(extent={{-103,-19},{-65,19}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.FuelSink sink annotation (Placement(transformation(extent={{66,-19.5},{106,19.5}})));
equation
  source.h_out = 1e6;

  source.P_out = 1e5;
  source.Q_out = -100;

  source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  assert(abs(sink.Q_in + source.Q_out) <= 1e-5, "In IsoPHFlowModel, DM should be 0");
  assert(abs(sink.Qv_in + source.Qv_out) <= 1e-5, "In IsoPHFlowModel, DV should be 0");
  assert(abs(source.P_out - sink.P_in) <= 1e-5, "In IsoPHFlowModel, DP should be 0");
  assert(abs(sink.Q_in*sink.h_in + source.Q_out*source.h_out) <= 1e-5, "In IsoPHFlowModel, W should be 0");
  connect(source.C_out, isoPHFlowModel.C_in) annotation (Line(points={{-74.5,0},{-23,0}}, color={95,95,95}));
  connect(isoPHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{76,0}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}),
       Diagram(coordinateSystem(preserveAspectRatio=false)));
end FuelIsoPHFlowModel;
