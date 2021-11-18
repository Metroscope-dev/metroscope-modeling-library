within MetroscopeModelingLibrary.Electrical.BoundaryConditions;
model Source

  Real W;
  Connectors.C_power u annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={-36,0}), iconTransformation(extent={{100,-20},{140,20}},
                                                                       rotation=
           0)));

equation
  W = u.W;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{120,100}}), graphics={
        Line(points={{50,0},{100,0},{82,10}}),
        Line(points={{100,0},{82,-10}}),
        Ellipse(
          extent={{-50,60},{70,-60}},
          lineColor={0,0,0},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid)}),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{120,100}})));
end Source;
