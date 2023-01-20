within MetroscopeModelingLibrary.FlueGases.Pipes;
model Leak

  Real Q;
  Real Q_th;
  Real Q_lbs;
  Real Q_Mlbh;

  // Dummy input for local balance:
  import MetroscopeModelingLibrary.Units.Inputs;
  Inputs.InputDifferentialPressure DP_input(start=0);

  Connectors.Inlet C_in annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Connectors.Outlet C_out annotation (Placement(transformation(extent={{90,-8},{110,12}})));
  PressureCut pressureCut annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Sensors.FlueGases.FlowSensor  leak_flow annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation

  Q = leak_flow.Q;
  Q_th = leak_flow.Q_th;
  Q_lbs = leak_flow.Q_lbs;
  Q_Mlbh = leak_flow.Q_Mlbh;

  // For local balance:
  pressureCut.DP = DP_input;

  connect(pressureCut.C_in, C_in) annotation (Line(points={{-60,0},{-100,0}}, color={95,95,95}));
  connect(leak_flow.C_in, pressureCut.C_out) annotation (Line(points={{18,0},{-40,0}}, color={95,95,95}));
  connect(leak_flow.C_out, C_out) annotation (Line(points={{38,0},{70,0},{70,2},{100,2}}, color={95,95,95}));
 annotation (Icon(graphics={Rectangle(
          extent={{-100,40},{0,-40}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
                             Rectangle(
          extent={{0,40},{100,-40}},
          lineColor={175,175,175},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{12,16},{36,6}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{8,0},{24,-6}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{36,2},{60,-6}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{56,10},{80,2}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,-6},{84,-14}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{18,-12},{42,-20}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid)}));
end Leak;
