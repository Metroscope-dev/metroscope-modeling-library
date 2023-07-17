within MetroscopeModelingLibrary.Utilities.Icons;
package Machines
  extends Icons.PackageIcon;

  partial record PumpIcon
    extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
    annotation (
      Diagram(coordinateSystem(
          extent={{-100,-100},{100,100}},
          grid={2,2}), graphics={
          Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={127,255,0},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,0},{80,0}}),
          Line(points={{80,0},{2,60}}),
          Line(points={{80,0},{0,-60}})}),
      Icon(coordinateSystem(
          extent={{-100,-100},{100,100}},
          grid={2,2}), graphics={
          Ellipse(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            fillColor={127,255,0},
            fillPattern=FillPattern.Solid),
          Line(points={{-80,0},{80,0}}),
          Line(points={{80,0},{2,60}}),
          Line(points={{80,0},{0,-60}})}));
  end PumpIcon;
  annotation (Icon(graphics={
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={0,0,0},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Line(points={{-32,0},{30,0}},
        color={0,0,0},
        thickness=1),
        Line(points={{30,0},{6,20}},
        color={0,0,0},
        thickness=1),
        Line(points={{30,0},{6,-20}},
        color={0,0,0},
        thickness=1)}));
end Machines;
