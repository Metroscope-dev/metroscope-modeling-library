within MetroscopeModelingLibrary.RefMoistAir.Pipes;
model PressureCut
  extends RefMoistAir.BaseClasses.IsoHFlowModel
                                             annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={0,255,128},
          fillColor={0,255,128},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-40,-60},{0,60}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{0,-60},{40,60}},
          color={0,0,0},
          thickness=1)}));
end PressureCut;
