within MetroscopeModelingLibrary.Tests.WaterSteam.Machines;
model FixedSpeedPump_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Utilities.Units.Pressure source_P(start=20e5);
  input Utilities.Units.Temperature source_T(start=150 + 273.15);
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-1000);

  // Component parameters
  parameter Utilities.Units.Height pump_hn = 10 "Pump head";
  parameter Utilities.Units.Yield pump_rh = 1.0 "Hydraulic efficiency";

  MetroscopeModelingLibrary.WaterSteam.Machines.FixedSpeedPump
                                                      fixedSpeedPump
                                                           annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{66,-10},{86,10}})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.T_out = source_T;
  source.Q_out = source_Q;

  // Component parameters
  fixedSpeedPump.hn = pump_hn;
  fixedSpeedPump.rh = pump_rh;

  connect(fixedSpeedPump.C_in, source.C_out) annotation (Line(points={{-10,0},{-61,0}}, color={28,108,200}));
  connect(fixedSpeedPump.C_out, sink.C_in) annotation (Line(points={{10,0},{71,0}}, color={28,108,200}));
end FixedSpeedPump_direct;
