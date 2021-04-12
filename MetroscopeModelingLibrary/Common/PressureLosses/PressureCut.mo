within MetroscopeModelingLibrary.Common.PressureLosses;
model PressureCut "Pressure Cut"
  extends MetroscopeModelingLibrary.Common.PressureLosses.PartialPressureLoss;
equation
  Q_in*h_in + Q_out*h_out = 0;
  Q_in*Xi_in =- Q_out*Xi_out;
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={           Polygon(
          points={{-100,-20},{-100,20},{-80,20},{-44,20},{-16,20},{100,20},{100,
              -20},{-100,-20}},
          lineColor={0,0,255},
          fillColor={128,255,0},
          fillPattern=FillPattern.Solid),
        Line(points={{30,40},{-50,-40}}, color={63,81,181}),
        Line(points={{-30,-40},{50,40}}, color={63,81,181})}),
    Window(
      x=0.09,
      y=0.2,
      width=0.66,
      height=0.69),
    Documentation(info="<html>
<h4>Copyright &copy; Metroscope</h4>
<h4>Metroscope Modeling Library</h4>
</html>",
   revisions="<html>

</html>
"));
end PressureCut;
