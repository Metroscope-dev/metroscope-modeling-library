within MetroscopeModelingLibrary.Partial.Media;
package PartialMedium
  extends Modelica.Media.Interfaces.PartialMedium;
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
        fillPattern=FillPattern.Solid),
      Ellipse(
        lineColor={102,102,102},
        fillColor={204,204,204},
        pattern=LinePattern.None,
        fillPattern=FillPattern.Sphere,
        extent={{-43,-42},{43,42}}),
      Rectangle(
        extent={{-60,14},{60,-14}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid,
        rotation=45)}));
end PartialMedium;
