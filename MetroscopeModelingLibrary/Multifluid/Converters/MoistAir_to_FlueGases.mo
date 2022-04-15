within MetroscopeModelingLibrary.MultiFluid.Converters;
model MoistAir_to_FlueGases

  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  constant Real Hlat=2501.5999019e3 "Phase transition energy";
  FlueGasesMedium.ThermodynamicState state0;
  MoistAir.Connectors.Inlet inlet annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
  FlueGases.Connectors.Outlet outlet annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  FlueGases.BoundaryConditions.Source source annotation (Placement(transformation(extent={{12,-10},{32,10}})));
equation

  source.P_out = sink.P_in;
  source.Q_out = - sink.Q_in;

  source.Xi_out[1] = (1 - sink.Xi_in[1])*0.768;
  source.Xi_out[2] = (1 - sink.Xi_in[1])*0.232;
  source.Xi_out[3] = sink.Xi_in[1];
  source.Xi_out[4] = 0;
  source.Xi_out[5] = 0;

  state0.p = 0.006112*1e5;
  state0.T = 273.16;
  state0.X = source.Xi_out;
  source.h_out = sink.h_in + FlueGasesMedium.specificEnthalpy(state0) - sink.Xi_in[1]*Hlat;


  connect(sink.C_in, inlet) annotation (Line(points={{-27,0},{-100,0}},                 color={85,170,255}));
  connect(source.C_out, outlet) annotation (Line(points={{27,0},{100,0}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{-100,40},{0,-40}},
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None), Rectangle(
          extent={{0,40},{102,-40}},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end MoistAir_to_FlueGases;
