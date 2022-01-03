within MetroscopeModelingLibrary.Common.Sensors;
model RotSpeedSensor

  Real VRot; // rpm
  Connectors.RealInput VRot_input annotation (Placement(transformation(extent={{-80,-20},
            {-40,20}}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={-108,0})));

equation

  VRot = VRot_input;  // rpm

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-66,80},{100,-86}},
          lineColor={0,0,0},
          lineThickness=0.5),
        Line(
          points={{-88,0},{-80,0},{-66,0}},
          color={0,0,0},
          thickness=0.5),    Text(
          extent={{-44,34},{76,-44}},
          textColor={0,0,0},
          textString="V")}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end RotSpeedSensor;
