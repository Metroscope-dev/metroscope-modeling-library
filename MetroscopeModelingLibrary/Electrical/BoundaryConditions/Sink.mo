within MetroscopeModelingLibrary.Electrical.BoundaryConditions;
model Sink
  Connectors.C_power u annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={-36,0})));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-40},
            {40,40}}), graphics={Ellipse(
          extent={{-10,32},{52,-30}},
          lineColor={28,108,200},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid), Line(
          points={{-16,0},{-10,0}},
          color={28,108,200},
          thickness=0.5)}), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-40,-40},{40,40}})));
end Sink;
