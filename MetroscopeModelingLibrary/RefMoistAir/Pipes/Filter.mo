within MetroscopeModelingLibrary.RefMoistAir.Pipes;
model Filter
  extends RefMoistAir.Pipes.Pipe
               annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={Rectangle(
          extent={{-60,100},{60,-100}},
          lineColor={28,108,200},
          fillColor={0,160,160},
          fillPattern=FillPattern.CrossDiag)}));
end Filter;
