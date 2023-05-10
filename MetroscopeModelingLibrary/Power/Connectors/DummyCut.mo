within MetroscopeModelingLibrary.Power.Connectors;
model DummyCut "Component that sets leaves the dummy power connector variable free in both sides"
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Inputs.InputNotUsedVariable dummy; // To keep local balance
  Outlet W_port(dummy=dummy, W=0) annotation (Placement(transformation(extent={{30,-10},{50,10}}), iconTransformation(extent={{30,-10},{50,10}})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-40,30},{40,-30}},
          lineColor={244,125,35},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-40,-60},{0,60}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{0,-60},{40,60}},
          color={0,0,0},
          thickness=1)}),     Diagram(coordinateSystem(preserveAspectRatio=false)));
end DummyCut;
