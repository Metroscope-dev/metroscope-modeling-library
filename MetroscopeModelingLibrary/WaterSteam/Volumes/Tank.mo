within MetroscopeModelingLibrary.WaterSteam.Volumes;
model Tank
  extends WaterSteam.BaseClasses.IsoPHFlowModel annotation (IconMap(primitivesVisible=false));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={28,108,200},
          fillColor={236,238,248},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Polygon(
          points={{-100,20},{100,20},{100,-40},{-100,-40},{-100,20}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={28,108,200},
          lineThickness=1)}),                                    Diagram(coordinateSystem(preserveAspectRatio=false)));
end Tank;
