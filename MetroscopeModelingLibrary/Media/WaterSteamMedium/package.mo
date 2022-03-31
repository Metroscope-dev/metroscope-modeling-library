within MetroscopeModelingLibrary.Media;
package WaterSteamMedium
  extends Modelica.Media.Water.StandardWater annotation(IconMap(primitivesVisible=false));

  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Ellipse(
          lineColor={102,102,102},
          fillColor={28,108,200},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}})}));
end WaterSteamMedium;
