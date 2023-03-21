within MetroscopeModelingLibrary.MoistAir.Pipes;
model PressureCut
  extends MoistAir.BaseClasses.IsoHFlowModel annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={28,108,200},
          fillColor={85,170,255},
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
