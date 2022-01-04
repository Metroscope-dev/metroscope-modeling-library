within MetroscopeModelingLibrary.Common.Sensors;
model BaseSensor
  extends MetroscopeModelingLibrary.Common.Partial.BasicTransportModel;

equation

  P_out = P_in;
  Q_in + Q_out = 0;
  Q_in*h_in + Q_out*h_out = 0;
  Xi_in = Xi_out;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-82,84},{84,-82}},
          lineColor={0,0,0},
          lineThickness=0.5),
        Line(
          points={{-82,0},{-94,0}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{84,0},{92,0},{100,0}},
          color={0,0,0},
          thickness=0.5)}),                                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BaseSensor;
