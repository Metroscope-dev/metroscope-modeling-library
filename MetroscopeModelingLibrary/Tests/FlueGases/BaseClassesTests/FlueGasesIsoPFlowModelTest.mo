within MetroscopeModelingLibrary.Tests.FlueGases.BaseClassesTests;
model FlueGasesIsoPFlowModelTest
  MetroscopeModelingLibrary.FlueGases.BaseClasses.FlueGasesIsoPFlowModel isoPFlowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.FlueGasesSource source annotation (Placement(transformation(extent={{-103,-19},{-65,19}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.FlueGasesSink sink annotation (Placement(transformation(extent={{64,-19.5},{104,19.5}})));
equation
  isoPFlowModel.W_input = 0;

  source.h_out = 1e6;

  source.P_out = 1e5;
  source.Q_out = -100;

  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  assert(abs(sink.Q_in + source.Q_out) <= 1e-5, "In IsoPFlowModel, DM should be 0");
  assert(abs(source.P_out - sink.P_in) <= 1e-5, "In IsoPFlowModel, DP should be 0");
  connect(source.C_out, isoPFlowModel.C_in) annotation (Line(points={{-74.5,0},{-23,0}}, color={175,175,175}));
  connect(isoPFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{74,0}}, color={175,175,175}));
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
end FlueGasesIsoPFlowModelTest;
