within MetroscopeModelingLibrary.Fuel.Connectors;
connector FluidOutlet
  package FuelMedium =
          MetroscopeModelingLibrary.Fuel.Medium.FuelMedium;
      extends MetroscopeModelingLibrary.Common.Connectors.FluidOutlet(redeclare
      package     Medium =
            FuelMedium);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                               Rectangle(
          extent={{-100,100},{100,-100}},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),        Rectangle(
          extent={{-80,80},{80,-80}},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}),                           Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FluidOutlet;
