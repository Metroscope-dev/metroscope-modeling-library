within MetroscopeModelingLibrary.FlueGases.PressureLosses;
model AdmiLouvers
   package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
  extends
    MetroscopeModelingLibrary.Common.PressureLosses.SingularPressureLoss_NoIcon(    redeclare
      package Medium =
        FlueGasesMedium);
  annotation (Icon(graphics={Polygon(
          points={{42,100},{-20,100},{-60,58},{-20,58},{-60,20},{-20,20},{-60,
              -20},{-18,-20},{-60,-60},{-18,-60},{-62,-100},{42,-100},{42,100}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={167,171,238},
          fillPattern=FillPattern.None), Polygon(
          points={{36,94},{-18,94},{-50,62},{-8,62},{-46,26},{-6,26},{-46,-14},
              {-4,-14},{-46,-54},{-4,-54},{-46,-94},{36,-94},{36,94}},
          lineThickness=0.5,
          fillColor={167,171,238},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}), Documentation(info="<html>
<h4>Copyright &copy; Metroscope</h4>
<h4>Metroscope Modeling Library</h4>
</html>"));
end AdmiLouvers;
