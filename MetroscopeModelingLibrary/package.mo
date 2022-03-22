package MetroscopeModelingLibrary
  extends Modelica.Icons.Package;

  annotation (uses(Modelica(version="4.0.0")),
  Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
      Rectangle(
        origin={0,35.1488},
        fillColor={255,255,255},
        extent={{-30.0,-20.1488},{30.0,20.1488}},
        lineColor={0,0,0},
        lineThickness=0.5),
      Rectangle(
        origin={0,-34.8512},
        fillColor={255,255,255},
        extent={{-30.0,-20.1488},{30.0,20.1488}},
        lineColor={0,0,0},
        lineThickness=0.5),
      Line(
        origin={-51.25,0},
        points={{21.25,-35.0},{-13.75,-35.0},{-13.75,35.0},{6.25,35.0}},
        color={0,0,0},
        thickness=0.5),
      Polygon(
        origin={-40,35},
        pattern=LinePattern.None,
        points={{10.0,0.0},{-5.0,5.0},{-5.0,-5.0}},
        lineColor={0,0,0},
        lineThickness=0.5),
      Line(
        origin={51.25,0},
        points={{-21.25,35.0},{13.75,35.0},{13.75,-35.0},{-6.25,-35.0}},
        color={0,0,0},
        thickness=0.5),
      Polygon(
        origin={40,-35},
        pattern=LinePattern.None,
        points={{-10.0,0.0},{5.0,5.0},{5.0,-5.0}},
        lineColor={0,0,0},
        lineThickness=0.5)}));
end MetroscopeModelingLibrary;
