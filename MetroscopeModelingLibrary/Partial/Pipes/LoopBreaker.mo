within MetroscopeModelingLibrary.Partial.Pipes;
partial model LoopBreaker
  replaceable Connectors.FluidInlet C_in annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  replaceable Connectors.FluidOutlet C_out annotation (Placement(transformation(extent={{90,-10},{110,10}})));

  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  Units.Inputs.InputReal loop_flow_error;


equation

  // Pressure is transmitted
  C_in.P = C_out.P;

  // Enthalpy is transmitted
  inStream(C_in.h_outflow) = C_out.h_outflow;
  C_in.h_outflow = 0;

  // Composition is transmitted
  inStream(C_in.Xi_outflow) = C_out.Xi_outflow;
  C_in.Xi_outflow = zeros(Medium.nXi);

  // Flow rate is not transmitted
  C_in.Q + C_out.Q = loop_flow_error;


  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-38,40},{42,-40}},
          lineColor={0,0,0},
          fillColor={128,255,0},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Line(
          points={{42,0},{94,0}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{-38,0},{-96,0},{-94,0}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{-22,-4},{-22,10},{18,10}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{14,14},{18,10},{14,6}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{-20,-7},{-20,7},{20,7}},
          color={0,0,0},
          thickness=1,
          origin={4,-5},
          rotation=180),
        Line(
          points={{-2,4},{2,0},{-2,-4}},
          color={0,0,0},
          thickness=1,
          origin={-14,-12},
          rotation=180)}),                                       Diagram(coordinateSystem(preserveAspectRatio=false)));
end LoopBreaker;
