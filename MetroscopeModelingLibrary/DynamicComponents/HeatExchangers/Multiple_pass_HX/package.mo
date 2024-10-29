within MetroscopeModelingLibrary.DynamicComponents.HeatExchangers;
package Multiple_pass_HX

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
        Polygon(
          points={{-40,-60},{-60,-60},{-60,40},{-90,20},{-50,80},{-10,20},{-40,40},{-40,-60}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={28,108,200}),
        Polygon(
          points={{60,80},{40,80},{40,-20},{10,0},{50,-60},{90,0},{60,-20},{60,80}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={28,108,200})}));
end Multiple_pass_HX;
