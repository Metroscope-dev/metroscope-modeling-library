within MetroscopeModelingLibrary.Electrical.Sensors;
model PowerSensor

  Real W;  // Power in W
  Real W_MW; // Power in MW
  Connectors.C_power C_out annotation (Placement(transformation(extent={{92,-20},
            {132,20}}),
                   iconTransformation(extent={{92,-20},{132,20}})));
  Connectors.C_power C_in annotation (Placement(transformation(extent={{20,-20},
            {-20,20}},
        rotation=180,
        origin={-36,0}),
                   iconTransformation(
        extent={{20,-20},{-20,20}},
        rotation=180,
        origin={-112,0})));
equation

  // Conservation of power
  C_in.W + C_out.W = 0;

  // Measure
  W = - C_in.W;
  W_MW = W/1e6;

    annotation (Placement(transformation(extent={{76,-20},{116,20}})),
                                        Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={-96,0})),
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,100}}), graphics={
        Ellipse(
          extent={{-82,84},{84,-82}},
          lineColor={0,0,0},
          lineThickness=0.5),Text(
          extent={{-108,44},{108,-50}},
          textColor={0,0,0},
          textString="W"),
        Line(
          points={{-82,0},{-94,0}},
          color={0,0,0},
          thickness=0.5),
        Line(
          points={{84,0},{100,0}},
          color={0,0,0},
          thickness=0.5)}),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end PowerSensor;
