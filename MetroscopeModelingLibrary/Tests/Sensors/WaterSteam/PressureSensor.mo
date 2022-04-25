within MetroscopeModelingLibrary.Tests.Sensors.WaterSteam;
model PressureSensor
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{38,-10},{58,10}})));
equation
  source.h_out = source_h;
  source.Q_out = source_Q;
  P_sensor.P = 1e5;

  assert(abs(P_sensor.P_barA - 1) < 1e-5, "P_sensor should detect 1 barA");
  assert(abs(P_sensor.P_barG) < 1e-5, "P_sensor should detect 0 barG");
  assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
  assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
  assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
  connect(P_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-43,0}}, color={28,108,200}));
  connect(P_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{43,0}}, color={28,108,200}));
end PressureSensor;
