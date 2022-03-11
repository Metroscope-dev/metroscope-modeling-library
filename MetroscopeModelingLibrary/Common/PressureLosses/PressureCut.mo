within MetroscopeModelingLibrary.Common.PressureLosses;
model PressureCut "Pressure Cut"
  extends MetroscopeModelingLibrary.Common.PressureLosses.PartialPressureLoss;

  connector InputAbsolutePressure = input Modelica.Units.SI.AbsolutePressure;

  InputAbsolutePressure P "Fluid pressure";
equation
  //Q_in*h_in + Q_out*h_out = 0;
  W = 0;
  //Q_in*Xi_in =- Q_out*Xi_out;
  // The extra pressure variable is defined ad hoc with connector input type
  // since you cannot change the type of the the variables in the base class
  P = P_in;
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
    Documentation(info="<html>
<h4>Copyright &copy; Metroscope</h4>
<h4>Metroscope Modeling Library</h4>
</html>",
   revisions="<html>

</html>
"));
end PressureCut;
