within MetroscopeModelingLibrary.Icons;
package Sensors
  extends Icons.PackageIcon;

  annotation (Icon(graphics={
        Ellipse(
          fillColor={245,245,245},
          fillPattern=FillPattern.Solid,
          extent={{-70,-70},{70,70}}),
        Line(points={{0,70},{0,40}}),
        Line(points={{22.9,32.8},{40.2,57.3}}),
        Line(points={{-22.9,32.8},{-40.2,57.3}}),
        Line(points={{37.6,13.7},{65.8,23.9}}),
        Line(points={{-37.6,13.7},{-65.8,23.9}}),
        Ellipse(
          lineColor={64,64,64},
          fillColor={255,255,255},
          extent={{-12,-12},{12,12}}),
        Polygon(
          rotation=-17.5,
          fillColor={64,64,64},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-5.0,0.0},{-2.0,60.0},{0.0,65.0},{2.0,60.0},{5.0,0.0}}),
        Ellipse(
          fillColor={64,64,64},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-7,-7},{7,7}})}));
end Sensors;
