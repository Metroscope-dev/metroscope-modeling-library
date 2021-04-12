within MetroscopeModelingLibrary.FlueGases.PressureLosses;
model Filter
   replaceable package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
  extends
    MetroscopeModelingLibrary.Common.PressureLosses.SingularPressureLoss_NoIcon(    redeclare
      package Medium =
        FlueGasesMedium);
  annotation (Icon(graphics={Rectangle(
          extent={{-60,100},{60,-100}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.CrossDiag), Rectangle(extent={{-100,100},{100,
              -100}}, lineColor={28,108,200})}), Documentation(info="<html>
<h4>Copyright &copy; Metroscope</h4>
<h4>Metroscope Modeling Library</h4>
</html>"));
end Filter;
