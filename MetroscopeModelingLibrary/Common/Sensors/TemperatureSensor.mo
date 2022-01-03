within MetroscopeModelingLibrary.Common.Sensors;
model TemperatureSensor
  extends BaseSensor;

  Real T; // Temperature in SI Units : K
  Real T_degC;  // Temperature in degC
  Real T_degF;  // Temperature in degF


equation
<<<<<<< HEAD

  T = T_in;
  T_degC = T_in - 273.15; // Conversion de Celsius
  T_degF = (T_in-273.15)*1.8 + 32.0;  // Conversion to Farenheit

  annotation (Icon(graphics={Text(
          extent={{-108,44},{108,-50}},
          textColor={0,0,0},
          textString="T")}));
=======
  state_in = Medium.setState_phX(C_in.P, C_in.h_vol,C_in.Xi_vol);
  T = Medium.temperature(state_in);
    annotation (defaultComponentName = "pressure",
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-40},
            {60,120}}),graphics={
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0}),
        Ellipse(
          extent={{-40,80},{40,0}},
          fillColor={255,85,85},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Text(
          extent={{-36,72},{38,4}},
          pattern=LinePattern.None,
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          textString="T")}),    Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-40,-40},{60,120}}),graphics={
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Ellipse(
          extent={{-40,80},{40,0}},
          lineColor={0,0,0}),
        Ellipse(
          extent={{-40,80},{40,0}},
          fillColor={255,85,85},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Text(
          extent={{-36,72},{38,4}},
          pattern=LinePattern.None,
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          textString="T")}),
    Documentation(info="<html>
<p><b>V3</b> Component now only has one port (31/05/2019)</p>
<p><b>V2</b> Creation of the heritage relationship and modification of the component accordingly (23/05/2019)</p>
<p><b>V1</b> Creation of the component and the single test (07/05/2019)</p>
<p><br><b>Parameters</b> :</p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\" width=\"100%\"><tr>
<td><p>Symbol</p></td>
<td><p>Meaning</p></td>
<td><p>Unit</p></td>
</tr>
<tr>
<td><p>T</p></td>
<td><p>Temperature measured by the sensor</p></td>
<td><p>K</p></td>
</tr>
</table>
<p><br><br><br><b>Direct mode</b>l : No fixed value. Gives temperature as an output</p>
<p><b>Modelling choices</b> : - No pressure loss in the component</p>
</html>"));
>>>>>>> main
end TemperatureSensor;
