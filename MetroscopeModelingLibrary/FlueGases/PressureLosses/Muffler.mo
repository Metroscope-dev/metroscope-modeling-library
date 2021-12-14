within MetroscopeModelingLibrary.FlueGases.PressureLosses;
model Muffler
   package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
  extends
    MetroscopeModelingLibrary.Common.PressureLosses.SingularPressureLoss_NoIcon(    redeclare
      package Medium =
        FlueGasesMedium);
  annotation (Icon(graphics={
        Ellipse(
          extent={{-60,60},{60,-60}},
          lineColor={28,108,200},
          fillColor={167,171,238},
          fillPattern=FillPattern.None,
          lineThickness=0.5),
        Polygon(
          points={{-24,14},{-24,-14},{-8,-14},{20,-40},{20,40},{-8,14},{-24,14}},
          lineColor={28,108,200},
          fillColor={167,171,238},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-42,-42},{42,42}},
          color={28,108,200},
          thickness=0.5),                     Rectangle(extent={{-100,100},{100,
              -100}}, lineColor={28,108,200})}), Documentation(info="<html>
<h4>Copyright &copy; Metroscope</h4>
<h4>Metroscope Modeling Library</h4>
</html>"));
end Muffler;
