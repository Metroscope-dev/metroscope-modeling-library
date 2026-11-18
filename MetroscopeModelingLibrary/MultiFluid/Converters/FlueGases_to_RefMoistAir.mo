within MetroscopeModelingLibrary.MultiFluid.Converters;
model FlueGases_to_RefMoistAir

  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

  FlueGases.Connectors.Inlet   inlet annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  FlueGases.BoundaryConditions.Sink   sink annotation (Placement(transformation(extent={{-30,-10},
            {-10,10}})));
  RefMoistAir.Connectors.Outlet
                              outlet annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  RefMoistAir.BoundaryConditions.Source
                                      source(h_out(start=1e6)) annotation (Placement(transformation(extent={{12,-10},{32,10}})));
equation

  source.P_out = sink.P_in;
  source.Q_out = - sink.Q_in;

  sink.Xi_in[3] = source.Xi_out[1];

  sink.T_in = source.T_out;

  connect(source.C_out, outlet) annotation (Line(points={{27,0},{100,0}}, color={0,127,
          127}));
  connect(sink.C_in, inlet) annotation (Line(points={{-25,0},{-100,0}},                 color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{0,40},{100,-40}},
          fillColor={0,127,127},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None), Rectangle(
          extent={{-100,40},{2,-40}},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end FlueGases_to_RefMoistAir;
