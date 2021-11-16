within MetroscopeModelingLibrary.Common.Sensors;
model OpeningSensor

  Real Op; // Opening (0 to 1)
  Real Op_pc; // Opening in percent
  Connectors.RealInput Op_input annotation (Placement(transformation(extent={{-80,-20},
            {-40,20}}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={-108,0})));

equation

  Op = Op_input;  // Opening (0 to 1)
  Op_pc = Op_input * 100;  // Opening (percentage)

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-66,80},{100,-86}},
          lineColor={0,0,0},
          lineThickness=0.5),
        Line(
          points={{-88,0},{-80,0},{-66,0}},
          color={0,0,0},
          thickness=0.5),    Text(
          extent={{-66,42},{98,-48}},
          textColor={0,0,0},
          textString="O")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end OpeningSensor;
