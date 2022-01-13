within MetroscopeModelingLibrary.Electrical.Machines;
model Generator "Eletrical generator"

  connector InputPerUnit = input Modelica.Units.SI.PerUnit;

  InputPerUnit eta(start = 99.8) "Efficiency (percent)";
  Modelica.Units.SI.Power Welec "Electrical power produced by the generator";
  Modelica.Units.SI.Power Wmech "Electrical power produced by the generator";
public
  Connectors.C_power C_power
    annotation (Placement(transformation(extent={{-116,-14},{-84,14}}, rotation=
           0)));
  Connectors.C_power C_elec annotation (Placement(transformation(extent={{92,-14},
            {120,14}}),
                   iconTransformation(extent={{92,-14},{120,14}})));
equation

  Wmech = C_power.W;
  Welec + Wmech*eta = 0;
  C_elec.W = Welec;
  annotation (Diagram(coordinateSystem(extent={{-100,-60},{100,60}}),
                      graphics={
        Rectangle(
          extent={{-56,33},{66,-33}},
          lineColor={0,0,255},
          fillColor={0,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-56,-3},{66,1}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={255,0,0}),
        Rectangle(
          extent={{-56,17},{66,21}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={255,0,0}),
        Rectangle(
          extent={{-56,-21},{66,-17}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={255,0,0}),
        Rectangle(
          extent={{66,13},{78,-11}},
          lineColor={0,0,255},
          fillColor={0,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-68,13},{-56,-11}},
          lineColor={0,0,255},
          fillColor={0,255,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-42,-23},{-44,-27},{-46,-29},{-50,-31},{-54,-31},{-58,-29},
              {-62,-23},{-64,-15},{-64,-7},{-64,15},{-62,21},{-60,25},{-58,27},
              {-54,29},{-52,29},{-48,27},{-46,25},{-44,21},{-44,27},{-48,23},{
              -44,21}}),
        Rectangle(
          extent={{-56,31},{66,35}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={255,0,0}),
        Rectangle(
          extent={{-56,-35},{66,-31}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={255,0,0}),
        Line(points={{-26,-11},{-4,13},{16,-15},{42,13}}, color={0,0,255}),
        Polygon(
          points={{42,13},{28,7},{36,-1},{42,13}},
          lineColor={0,0,255},
          lineThickness=1,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-82,0},{-68,0}}, color={0,0,255}),
        Line(points={{-96,0},{-82,0}}, color={0,0,255}),
        Line(points={{86,0},{78,0}}, color={28,108,200})}),
                                Icon(coordinateSystem(extent={{-100,-60},{100,60}}),
                                     graphics={
        Rectangle(
          extent={{-56,33},{66,-33}},
          lineColor={0,0,255},
          fillColor={128,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-56,-3},{66,1}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={255,0,0}),
        Rectangle(
          extent={{-56,17},{66,21}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={255,0,0}),
        Rectangle(
          extent={{-56,-21},{66,-17}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={255,0,0}),
        Rectangle(
          extent={{66,13},{78,-11}},
          lineColor={0,0,255},
          fillColor={0,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-68,13},{-56,-11}},
          lineColor={0,0,255},
          fillColor={0,255,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-42,-23},{-44,-27},{-46,-29},{-50,-31},{-54,-31},{-58,-29},
              {-62,-23},{-64,-15},{-64,-7},{-64,15},{-62,21},{-60,25},{-58,27},
              {-54,29},{-52,29},{-48,27},{-46,25},{-44,21},{-44,27},{-48,23},{
              -44,21}}),
        Rectangle(
          extent={{-56,31},{66,35}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={255,0,0}),
        Rectangle(
          extent={{-56,-35},{66,-31}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={255,0,0}),
        Line(points={{-26,-11},{-4,13},{16,-15},{42,13}}, color={0,0,255}),
        Polygon(
          points={{42,13},{28,7},{36,-1},{42,13}},
          lineColor={0,0,255},
          lineThickness=1,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Line(points={{-74,0},{-68,0}}, color={0,128,255}),
        Line(points={{-82,0},{-68,0}}, color={0,0,255}),
        Line(points={{-96,0},{-82,0}}, color={0,0,255}),
        Line(points={{78,0},{92,0}}, color={28,108,200})}),
    Documentation(info="<html>
<p><b>Copyright &copy; EDF 2002 - 2010</b></p>
</HTML>
<html>
<p><b>ThermoSysPro Version 2.0</b></p>
</HTML>
", revisions="<html>
<u><p><b>Authors</u> : </p></b>
<ul style='margin-top:0cm' type=disc>
<li>
    Baligh El Hefni</li>
<li>
    Daniel Bouskela</li>
</ul>
</html>
"));
end Generator;
