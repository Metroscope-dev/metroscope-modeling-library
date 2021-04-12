within MetroscopeModelingLibrary.Fuel.Connectors;
connector FluidInlet
  package FuelMedium =
          MetroscopeModelingLibrary.Fuel.Medium.FuelMedium;
      extends MetroscopeModelingLibrary.Common.Connectors.FluidInlet(redeclare
      package     Medium =
            FuelMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,100},{100,-100}},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0})}),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FluidInlet;
