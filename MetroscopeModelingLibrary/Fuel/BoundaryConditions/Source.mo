within MetroscopeModelingLibrary.Fuel.BoundaryConditions;
model Source
      replaceable package FuelMedium =
          MetroscopeModelingLibrary.Fuel.Medium.FuelMedium;
      extends MetroscopeModelingLibrary.Common.BoundaryConditions.Source(redeclare
      package     Medium =
            FuelMedium);
  annotation (Icon(graphics={
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}));
end Source;
