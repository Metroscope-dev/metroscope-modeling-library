within MetroscopeModelingLibrary.Fuel.Pipes;
model PressureCut
  extends Fuel.BaseClasses.IsoHFlowModel
                                    annotation(IconMap(primitivesVisible=false));
    annotation (Icon(graphics={Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={28,108,200},
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{-40,-60},{0,60}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{0,-60},{40,60}},
          color={0,0,0},
          thickness=1)}));
end PressureCut;
