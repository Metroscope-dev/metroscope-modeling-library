within MetroscopeModelingLibrary.Power.Connectors;
model DummyFix_2 "Component that sets the dummy power connector variable in both sides"
  Outlet C_out(dummy=0) annotation (Placement(transformation(extent={{30,-10},{50,10}}), iconTransformation(extent={{30,-10},{50,10}})));
  Inlet C_in(dummy=0) annotation (Placement(transformation(extent={{-50,-10},{-30,10}}), iconTransformation(extent={{-50,-10},{-30,10}})));
equation
  C_out.W + C_in.W = 0;
   annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{-40,30},{40,-30}},
          lineColor={244,125,35},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}),
                              Diagram(coordinateSystem(preserveAspectRatio=false)));
end DummyFix_2;
