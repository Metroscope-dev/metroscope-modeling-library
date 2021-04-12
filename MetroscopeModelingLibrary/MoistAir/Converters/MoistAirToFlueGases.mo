within MetroscopeModelingLibrary.MoistAir.Converters;
model MoistAirToFlueGases
  package MoistAirMedium =
      MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
   package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
  constant Real Hlat=2501.5999019e3 "Phase transition energy";
  FlueGasesMedium.ThermodynamicState state0;
  Common.Connectors.FluidInlet C_in(redeclare package Medium = MoistAirMedium)
    annotation (Placement(transformation(extent={{-108,-10},{-88,10}})));
  Common.Connectors.FluidOutlet C_out(redeclare package Medium =
        FlueGasesMedium)
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  FlueGases.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{36,-10},{56,10}})));
  BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-42,-10},{-22,10}})));
equation
  source.P_out = sink.P_in;
  source.Q_out = -sink.Q_in;
  source.Xi_vol[1] = (1 - sink.Xi_in[1])*0.768;
  source.Xi_vol[2] = (1 - sink.Xi_in[1])*0.232;
  source.Xi_vol[3] = sink.Xi_in[1];
  source.Xi_vol[4] = 0;
  source.Xi_vol[5] = 0;
  state0.p = 0.006112*1e5;
  state0.T = 273.16;
  state0.X = source.Xi_vol;
  source.h_vol = sink.h_in + FlueGasesMedium.specificEnthalpy(state0) - sink.Xi_in[1]*Hlat;
  sink.h_vol = 24e3;
  sink.Xi_vol[1] = 0.01;
  connect(source.C_out, C_out)
    annotation (Line(points={{56,0},{100,0}}, color={63,81,181}));
  connect(sink.C_in, C_in)
    annotation (Line(points={{-42,0},{-98,0}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,100},{0,-100}},
          lineColor={0,0,255},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid), Rectangle(
          extent={{0,100},{100,-100}},
          lineColor={0,0,255},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end MoistAirToFlueGases;
