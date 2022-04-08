within MetroscopeModelingLibrary.Tests.Fuel.BaseClasses;
model FuelFlowModel
  MetroscopeModelingLibrary.Fuel.BaseClasses.FuelFlowModel flowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.FuelSource source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.FuelSink sink annotation (Placement(transformation(extent={{61,-20},{103,20}})));
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
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end FuelFlowModel;
