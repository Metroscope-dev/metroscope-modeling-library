within MetroscopeModelingLibrary.Tests.WaterSteam.Machines;
model SteamTurbineWithNozzle_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=20e5);
  input Utilities.Units.SpecificEnthalpy source_h(start=2.7718e6);
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-500);

  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbineWithNozzle
                                                              turbine annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-66,-10},{-46,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink power_sink annotation (Placement(transformation(extent={{62,20},{82,40}})));
equation

  // Boundary conditions
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;

  // Turbine parameters
  turbine.eta_nz = 0.95;
  turbine.area_nz = 25;

  // Component parameters
  turbine.Cst = 30000;
  turbine.eta_is = 0.8;

  connect(sink.C_in, turbine.C_out) annotation (Line(points={{67,0},{10,0}}, color={28,108,200}));
  connect(turbine.C_in, source.C_out) annotation (Line(points={{-10,0},{-51,0}}, color={28,108,200}));
  connect(turbine.C_W_out, power_sink.C_in) annotation (Line(points={{6,8},{56,8},{56,30},{67,30}},      color={244,125,35}));
end SteamTurbineWithNozzle_direct;
