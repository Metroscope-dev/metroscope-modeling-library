within MetroscopeModelingLibrary.Fuel.BoundaryConditions;
model Sink
      package FuelMedium =
          MetroscopeModelingLibrary.Fuel.Medium.FuelMedium;
      extends MetroscopeModelingLibrary.Common.BoundaryConditions.Sink(redeclare
      package     Medium =
            FuelMedium);
  annotation (Icon(graphics={
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-50,50},{50,-50}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0})}));
end Sink;
