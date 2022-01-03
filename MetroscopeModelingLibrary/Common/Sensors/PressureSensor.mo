within MetroscopeModelingLibrary.Common.Sensors;
model PressureSensor
  extends BaseSensor;

  Real P; // Absolute pressure in Pa (SI units)
  Real P_barA; // Absolute pressure in bar
  Real P_barG; // Relative (gauge) pressure in bar
  Real P_mbar; // Pressure in mbar
  Real P_psi; // Pressure in PSI


equation
<<<<<<< HEAD

  P = P_in;
  P_barA = P_in * 1e-5;
  P_mbar = P_in * 1e-2;
  P_barG = P_in*1e-5 - 1;
  P_psi = P_in*0.000145038;

  annotation (Icon(graphics={Text(
          extent={{-102,46},{114,-48}},
          textColor={0,0,0},
          textString="P")}));
=======
  C_in.P = P;
    annotation (defaultComponentName = "pressure",
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-40},
            {60,120}}),graphics={
        Ellipse(
          extent={{-40,80},{40,0}},
          fillColor={81,211,100},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Text(
          extent={{-36,72},{38,4}},
          pattern=LinePattern.None,
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          textString="P",
          lineColor={0,0,0}),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0})}),
                                Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-40,-40},{60,120}}),graphics={
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Ellipse(
          extent={{-40,80},{40,0}},
          lineColor={0,0,0},
          fillColor={81,211,100},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-34,74},{40,6}},
          pattern=LinePattern.None,
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          textString="P",
          lineColor={0,0,0})}),
    Documentation(info="<html>
<p><b>V2</b> Creation of the heritage relationship and modification of the component accordingly (23/05/2019)</p>
<p><b>V1</b> Creation of the component and the single test (07/05/2019)</p>
<p><br><b>Parameters</b> :</p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\" width=\"100%\"><tr>
<td><p>Symbol</p></td>
<td><p>Meaning</p></td>
<td><p>Unit</p></td>
</tr>
<tr>
<td><p>P</p></td>
<td><p>Pressure measured by the sensor</p></td>
<td><p>Pa</p></td>
</tr>
</table>
<p><br><br><b>Direct mode</b>l : No fixed value. Gives pressure as an output</p>
<p><b>Modelling choices</b> : - No pressure loss in the component</p>
</html>"));
>>>>>>> main
end PressureSensor;
