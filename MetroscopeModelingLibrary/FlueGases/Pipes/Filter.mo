within MetroscopeModelingLibrary.FlueGases.Pipes;
model Filter
  extends FrictionPipe
               annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={Rectangle(
          extent={{-60,100},{60,-100}},
          lineColor={28,108,200},
          fillColor={215,215,215},
          fillPattern=FillPattern.CrossDiag)}));
end Filter;
