within MetroscopeModelingLibrary.Tests.Power.Connectors;
model DummyCut
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.PowerTestIcon;
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source source annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-38,0})));
  MetroscopeModelingLibrary.Power.Connectors.DummyCut dummyCut annotation (Placement(transformation(extent={{-10,10},{10,30}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink_1 annotation (Placement(transformation(extent={{28,10},{48,30}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink_2 annotation (Placement(transformation(extent={{28,-30},{48,-10}})));
equation
  source.W_out = -1;
  sink_1.W_in = sink_2.W_in;
  connect(dummyCut.C_out, sink_1.C_in) annotation (Line(points={{4,20},{33,20}}, color={244,125,35}));
  connect(source.C_out, dummyCut.C_in) annotation (Line(points={{-33.2,0},{-20,0},{-20,20},{-4,20}}, color={244,125,35}));
  connect(sink_2.C_in, dummyCut.C_in) annotation (Line(points={{33,-20},{-20,-20},{-20,20},{-4,20}}, color={244,125,35}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end DummyCut;
