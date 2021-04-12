within MetroscopeModelingLibrary.Common.Sensors.BaseSensors;
model BaseFlowSensor "Partial component to model a sensor that measures a flow variable"
    replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium "Medium in the sensor";
  Common.Connectors.FluidOutlet C_out(redeclare package Medium =
       Medium)
    annotation (Placement(transformation(extent={{20,-50},{40,-30}}),
        iconTransformation(extent={{20,-50},{40,-30}})));
  Common.Connectors.FluidInlet C_in(redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-40,-50},{-20,-30}}),
        iconTransformation(extent={{-40,-50},{-20,-30}})));
  Common.Partial.BasicTransportModel transport(redeclare package Medium =
       Medium)
    annotation (Placement(transformation(extent={{-8,-50},{12,-30}})));
equation
 transport.Q_in = -transport.Q_out;
 transport.h_in = transport.h_out;
 transport.P_in = transport.P_out;
 transport.Xi_in = transport.Xi_out;
 //transport.Qi_in = transport.Qi_out;
  connect(C_out, C_out)
    annotation (Line(points={{30,-40},{30,-40}}, color={238,46,47}));
  connect(C_in, transport.C_in)
    annotation (Line(points={{-30,-40},{-8,-40}}, color={0,0,255}));
  connect(transport.C_out, C_out)
    annotation (Line(points={{12.2,-40},{30,-40}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,-40},
            {40,80}}), graphics={
        Ellipse(
          extent={{-40,80},{40,0}},
          lineColor={0,0,0}),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0}),
        Line(points={{-20,-40},{22,-40}}, color={0,0,0})}),
                                Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-40,-40},{40,80}}), graphics={
        Ellipse(
          extent={{-40,80},{40,0}},
          lineColor={0,0,0}),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0})}),
    Documentation(info="<html>
<p><b>V1</b> Creation of the component (23/05/2019)</p>
<p>This component can be used to create sensors that measure the value of a flow in a circuit.</p>
</html>"),   Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-40,80},{40,0}},
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0})}), Diagram(coordinateSystem(
          preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-40,80},{40,0}},
          fillColor={80,158,47},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Line(
          points={{-18,-12},{28,-58},{54,-22}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{0,0},{0,-40}}, color={0,0,0})}));
end BaseFlowSensor;
