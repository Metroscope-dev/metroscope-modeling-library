within MetroscopeModelingLibrary.Connectors;
connector FluidInlet
  extends MetroscopeModelingLibrary.Connectors.FluidPort(Q(min=0, start=500)); // Q into component is positive
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end FluidInlet;
