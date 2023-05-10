within MetroscopeModelingLibrary.Tests.Power.Connectors;
model DummyFix
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.PowerTestIcon;
  MetroscopeModelingLibrary.Power.Connectors.DummyFix dummyFix annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Source source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={38,0})));
equation
  connect(dummyFix.W_port, source.C_out) annotation (Line(points={{4,0},{33.2,0}}, color={244,125,35}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end DummyFix;
