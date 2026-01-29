within MetroscopeModelingLibrary.FlueGases.Pipes;
model Filter
  extends FrictionPipe
               annotation(IconMap(primitivesVisible=false));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, initialScale=0.2),
                   graphics={Rectangle(
          extent={{-60,100},{60,-100}},
          lineColor={28,108,200},
          fillColor={215,215,215},
          fillPattern=FillPattern.CrossDiag)}), Diagram(coordinateSystem(preserveAspectRatio=false, initialScale=0.2)));
end Filter;
