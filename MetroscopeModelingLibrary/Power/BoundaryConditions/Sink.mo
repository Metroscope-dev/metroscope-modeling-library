within MetroscopeModelingLibrary.Power.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;

  Utilities.Units.PositivePower W_in;
  Connectors.Inlet C_in annotation (Placement(transformation(extent={{-60,-10},{-40,10}}), iconTransformation(extent={{-60,-10},{-40,10}})));
equation
  W_in = C_in.W;
  annotation (Icon(coordinateSystem(initialScale=0.2),
                   graphics={
        Ellipse(
          extent={{-40,60},{80,-60}},
          lineColor={255,128,0},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-30,50},{70,-50}},
          lineColor={255,128,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-18,38},{55,-35}}, color={255,128,0},
          thickness=1),
        Line(points={{-18,-38},{55,35}}, color={255,128,0},
          thickness=1)}), Diagram(coordinateSystem(initialScale=0.2)));
end Sink;
