within MetroscopeModelingLibrary.RefMoistAir.Pipes;
model AdmiLouver
  extends Pipe annotation(IconMap(primitivesVisible=false));
  annotation (Icon(graphics={                                             Line(
          points={{46,100},{46,-100},{-54,-100},{-14,-60},{-54,-60},{-14,-20},{-54,-20},{-14,20},{-54,20},{-14,60},{-54,60},{-14,100},{46,100}},
          color={0,255,128},
          thickness=0.5), Polygon(
          points={{40,96},{40,-96},{-44,-96},{-4,-56},{-44,-56},{-4,-16},{-44,-16},{-4,24},{-44,24},{-4,64},{-44,64},{-12,96},{40,96}},
          lineColor={0,255,128},
          fillColor={170,255,213},
          fillPattern=FillPattern.Solid)}));
end AdmiLouver;
