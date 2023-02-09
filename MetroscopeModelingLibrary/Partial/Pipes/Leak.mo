within MetroscopeModelingLibrary.Partial.Pipes;
partial model Leak

  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  Real Q;
  Real Q_th;
  Real Q_lbs;
  Real Q_Mlbh;

  // Dummy input for local balance:
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputDifferentialPressure DP_input(start=0);

  replaceable Partial.Connectors.FluidInlet C_in(
    redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
  replaceable Partial.Connectors.FluidOutlet C_out(
    redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{90,-10},{110,10}})));
  replaceable Sensors.FlowSensor flow_sensor annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  replaceable BaseClasses.IsoHFlowModel flow_model annotation (Placement(transformation(extent={{14,-10},{34,10}})));
equation

  Q = flow_sensor.Q;
  Q_th = flow_sensor.Q_th;
  Q_lbs = flow_sensor.Q_lbs;
  Q_Mlbh = flow_sensor.Q_Mlbh;

  // For local balance:
  flow_model.DP = DP_input;

  connect(flow_model.C_out, C_out) annotation (Line(points={{34,0},{100,0}}, color={0,0,0}));
  connect(flow_model.C_in, flow_sensor.C_out) annotation (Line(points={{14,0},{-30,0}}, color={95,95,95}));
  connect(flow_sensor.C_in, C_in) annotation (Line(points={{-50,0},{-100,0}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                            Rectangle(
          extent={{-100,40},{0,-40}},
          lineColor={0,140,72},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid),
                             Rectangle(
          extent={{0,40},{100,-40}},
          lineColor={0,140,72},
          fillColor={170,255,213},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{12,16},{36,6}},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{8,0},{24,-6}},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{36,2},{60,-6}},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{56,10},{80,2}},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{60,-6},{84,-14}},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{18,-12},{42,-20}},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Leak;
