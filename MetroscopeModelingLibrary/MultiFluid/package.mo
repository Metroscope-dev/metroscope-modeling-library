within MetroscopeModelingLibrary;
package MultiFluid

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
          fillColor={255,255,255},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-60,-54},{60,66}}),
        Ellipse(
          extent={{56,50},{0,-6}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Line(points={{-44,90}}, color={175,175,175}),
        Ellipse(
          extent={{0,50},{-56,-6}},
          lineColor={213,213,0},
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{28,2},{-28,-54}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-52,38},{-52,38}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),
           Documentation(info="<html>
  <p>Licensed by Metroscope under the Modelica License 2 </p>
<p>Copyright © 2023, Metroscope.</p>
<p>This Modelica package is free software and the use is completely at your own risk; it can be redistributed and/or modified under the terms of the Modelica License 2. </p>
</html>"));

end MultiFluid;
