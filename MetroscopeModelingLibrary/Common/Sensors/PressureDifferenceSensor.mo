within MetroscopeModelingLibrary.Common.Sensors;
model PressureDifferenceSensor
    replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium;
    extends
    MetroscopeModelingLibrary.Common.Sensors.BaseSensors.BaseDifferenceSensor;
  Common.Connectors.RealOutput DeltaP(final quantity="Pressure",
                                          final unit="Pa",
                                          displayUnit="Pa",
                                          min=0) "Pressure difference between the ports" annotation (Placement(transformation(extent={{40,-10},
            {60,10}}),     iconTransformation(extent={{40,-6},{52,6}})));
equation
 DeltaP = C_out.P - C_in.P;
    annotation (defaultComponentName = "pressureDifference",
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-60},
            {80,60}}), graphics={
        Line(
          points={{-18,14},{28,-32},{54,4}},
          color={0,0,0},
          pattern=LinePattern.None),
        Ellipse(
          extent={{-40,40},{40,-40}},
          fillColor={81,211,100},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Text(
          extent={{-14,22},{18,-24}},
          lineColor={0,0,0},
          textString="P")}),    Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-60,-60},{80,60}}), graphics={
        Ellipse(
          extent={{-40,40},{40,-40}},
          fillColor={81,211,100},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,70},{0,60}}, color={0,0,0}),
        Text(
          extent={{-22,26},{28,-32}},
          lineColor={0,0,0},
          textString="P")}),
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
<td><p>DeltaP</p></td>
<td><p>Pressure difference measured by the sensor</p></td>
<td><p>Pa</p></td>
</tr>
</table>
<p><br><br><b>Direct mode</b>l : No fixed value. Gives pressure difference as an output</p>
<p><b>Modelling choices</b> : - No pressure loss in the component</p>
<p>- Pressure difference is measured as P_in - P_out</p>
</html>"));
end PressureDifferenceSensor;
