within MetroscopeModelingLibrary.Partial;
package BaseClasses
  Connectors.FluidConnectors.FluidInlet C_in3(redeclare package Medium = Medium)
                                                                                annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-64,-18},{-28,18}})));
  Connectors.FluidConnectors.FluidOutlet C_out3(redeclare package Medium = Medium)
                                                                                  annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{28,-16},{60,16}})));

  annotation (
  Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
          100}}), graphics={
      Rectangle(
        lineColor={215,215,215},
        fillColor={255,255,255},
        fillPattern=FillPattern.HorizontalCylinder,
        extent={{-100,-100},{100,100}},
        radius=25),
      Rectangle(
        lineColor={215,215,215},
        extent={{-100,-100},{100,100}},
        radius=25),
      Ellipse(
        extent={{-80,80},{80,-80}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-55,55},{55,-55}},
        lineColor={255,255,255},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),                                   Rectangle(
          extent={{-44,48},{48,-48}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
      Rectangle(
        extent={{-66.468,15.5563},{70.7107,-14.1421}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid,
        rotation=45)}));
end BaseClasses;
