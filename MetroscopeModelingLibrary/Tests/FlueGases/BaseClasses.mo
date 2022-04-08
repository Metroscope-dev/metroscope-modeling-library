within MetroscopeModelingLibrary.Tests.FlueGases;
package BaseClasses
model FlueGasesFlowModel
  MetroscopeModelingLibrary.FlueGases.BaseClasses.FlueGasesFlowModel waterFlowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.FlueGasesSource source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.FlueGasesSink sink annotation (Placement(transformation(extent={{61,-20},{103,20}})));
equation
  waterFlowModel.W_input = 0;
  waterFlowModel.DP_input = 0;

  source.h_out = 1e6;

  source.P_out = 1e5;
  source.Q_out = -100;

  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  assert(abs(sink.Q_in + source.Q_out) <= 1e-5, "In flow model, DM should be 0");
    connect(sink.C_in, waterFlowModel.C_out) annotation (Line(points={{71.5,0},{23,0}}, color={175,175,175}));
    connect(waterFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={175,175,175}));
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
end FlueGasesFlowModel;

model FlueGasesIsoPFlowModel
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
end FlueGasesIsoPFlowModel;

model FlueGasesIsoHFlowModel
  MetroscopeModelingLibrary.FlueGases.BaseClasses.FlueGasesIsoHFlowModel isoHFlowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.FlueGasesSource source annotation (Placement(transformation(extent={{-103,-19},{-65,19}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.FlueGasesSink sink annotation (Placement(transformation(extent={{64,-19.5},{104,19.5}})));
equation
  isoHFlowModel.DP_input = 0;

  source.h_out = 1e6;

  source.P_out = 1e5;
  source.Q_out = -100;

  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  assert(abs(sink.Q_in + source.Q_out) <= 1e-5, "In IsoHFlowModel, DM should be 0");
  assert(abs(sink.Q_in*sink.h_in + source.Q_out*source.h_out) <= 1e-5, "In IsoHFlowModel, W should be 0");
  connect(source.C_out, isoHFlowModel.C_in) annotation (Line(points={{-74.5,0},{-23,0}}, color={175,175,175}));
  connect(isoHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{74,0}}, color={175,175,175}));
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
end FlueGasesIsoHFlowModel;

model FlueGasesIsoPHFlowModel
  MetroscopeModelingLibrary.FlueGases.BaseClasses.FlueGasesIsoPHFlowModel isoPHFlowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.FlueGasesSource source annotation (Placement(transformation(extent={{-103,-19},{-65,19}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.FlueGasesSink sink annotation (Placement(transformation(extent={{66,-19.5},{106,19.5}})));
equation
  source.h_out = 1e6;

  source.P_out = 1e5;
  source.Q_out = -100;

  source.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  assert(abs(sink.Q_in + source.Q_out) <= 1e-5, "In IsoPHFlowModel, DM should be 0");
  assert(abs(sink.Qv_in + source.Qv_out) <= 1e-5, "In IsoPHFlowModel, DV should be 0");
  assert(abs(source.P_out - sink.P_in) <= 1e-5, "In IsoPHFlowModel, DP should be 0");
  assert(abs(sink.Q_in*sink.h_in + source.Q_out*source.h_out) <= 1e-5, "In IsoPHFlowModel, W should be 0");
    connect(source.C_out, isoPHFlowModel.C_in) annotation (Line(points={{-74.5,0},{-23,0}}, color={175,175,175}));
    connect(isoPHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{76,0}}, color={175,175,175}));
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
end FlueGasesIsoPHFlowModel;
annotation (Icon(graphics={
      Rectangle(
        lineColor={200,200,200},
        fillColor={248,248,248},
        fillPattern=FillPattern.HorizontalCylinder,
        extent={{-100,-100},{100,100}},
        radius=25.0),
      Rectangle(
        lineColor={128,128,128},
        extent={{-100,-100},{100,100}},
        radius=25.0),
      Polygon(
        origin={8,14},
        lineColor={78,138,73},
        fillColor={78,138,73},
        pattern=LinePattern.None,
        fillPattern=FillPattern.Solid,
        points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
end BaseClasses;
