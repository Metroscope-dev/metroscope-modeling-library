within MetroscopeModelingLibrary.Common.Sensors;
model MassFlowSensor
    replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium;
          extends
    MetroscopeModelingLibrary.Common.Sensors.BaseSensors.BaseFlowSensor;
  Common.Connectors.RealOutput Q(final quantity="Mass flow",
                                          final unit="kg/s",
                                          displayUnit="kg/s",
                                          min=0) "Mass flow at port" annotation (Placement(transformation(extent={{40,
            30},{60,50}}), iconTransformation(extent={{40,34},{52,46}})));
equation
  C_in.Q = Q;
  connect(C_out, C_out)
    annotation (Line(points={{30,-40},{30,-40}}, color={238,46,47}));
    annotation (defaultComponentName = "massFlow",Placement(transformation(extent={{40,30},{60,50}})),
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-40},
            {60,120}}),graphics={
        Ellipse(
          extent={{-40,80},{40,0}},
          fillColor={0,145,234},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Text(
          extent={{-36,72},{38,4}},
          pattern=LinePattern.None,
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          textString="Q"),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0}),
        Line(points={{-20,-40},{22,-40}}, color={0,0,0})}),
                                Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-40,-40},{60,120}}),graphics={
        Ellipse(
          extent={{-40,80},{40,0}},
          fillColor={0,145,234},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Text(
          extent={{-36,72},{38,4}},
          pattern=LinePattern.None,
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          textString="Q"),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0})}),
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
<p><br><br><br><br><b>Direct mode</b>l : No fixed value. Gives enthalpy as an output</p>
<p><b>Modelling choices</b> : - No pressure loss in the component</p>
</html>"));
end MassFlowSensor;
