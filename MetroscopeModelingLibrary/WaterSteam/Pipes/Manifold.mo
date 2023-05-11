within MetroscopeModelingLibrary.WaterSteam.Pipes;
model Manifold

  // Dummy input for local balance:
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  Inputs.InputDifferentialPressure DP_input(start=0);

  Connectors.Inlet C_in_1 annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
  Connectors.Inlet C_in_2 annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
  Connectors.Outlet C_out annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  PressureCut pressure_cut_1 annotation (Placement(transformation(extent={{-56,-10},{-36,10}})));
  PressureCut pressure_cut_2 annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,42})));
equation

  // Pressure Average
  C_out.P * C_out.Q + C_in_1.P * C_in_1.Q + C_in_2.P * C_in_2.Q = 0;

  // Dummy input for local balance
  DP_input = pressure_cut_1.DP_input;


  connect(pressure_cut_1.C_in, C_in_1) annotation (Line(points={{-56,0},{-100,0}},   color={28,108,200}));
  connect(pressure_cut_1.C_out, C_out) annotation (Line(points={{-36,0},{100,0}}, color={28,108,200}));
  connect(pressure_cut_2.C_in, C_in_2) annotation (Line(points={{1.77636e-15,52},{1.77636e-15,76},{0,76},{0,100}},
                                                                                       color={28,108,200}));
  connect(pressure_cut_2.C_out, C_out) annotation (Line(points={{-1.77636e-15,32},{-1.77636e-15,0},{100,0}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                             Rectangle(
          extent={{-100,20},{100,-22}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
                             Rectangle(
          extent={{-22,100},{20,20}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end Manifold;
