within MetroscopeModelingLibrary.Tests.WaterSteamTests.Machines;
model WaterPump_direct
  extends Modelica.Icons.Example;

  // Boundary conditions
  input Units.Pressure source_P(start=2e5);
  input Units.Temperature source_T(start=20 + 273.15);
  input Units.OutletMassFlowRate source_Q(start=-100);

  // Component parameters
  parameter Real pump_VRot = 1400;
  parameter Real pump_VRotn = 1400;
  parameter Real pump_rm = 0.85;
  parameter Real pump_a1 = -88.67;
  parameter Real pump_a2 = 0;
  parameter Real pump_a3 = 43.15;
  parameter Real pump_b1 = -3.7751;
  parameter Real pump_b2 = 3.61;
  parameter Real pump_b3 = -0.0075464;
  parameter Units.Yield pump_rhmin = 0.20;

  WaterSteam.Machines.WaterPump pump annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  WaterSteam.BoundaryConditions.WaterSource source annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  WaterSteam.BoundaryConditions.WaterSink waterSink annotation (Placement(transformation(extent={{66,-10},{86,10}})));
equation
  // Boundary conditions
  source.P_out = source_P;
  source.T_out = source_T;
  source.Q_out = source_Q;

  // Component parameters
  pump.VRot = pump_VRot;
  pump.VRotn = pump_VRotn;
  pump.rm = pump_rm;
  pump.a1 = pump_a1;
  pump.a2 = pump_a2;
  pump.a3 = pump_a3;
  pump.b1 = pump_b1;
  pump.b2 = pump_b2;
  pump.b3 = pump_b3;
  pump.rhmin = pump_rhmin;

  connect(pump.C_in, source.C_out) annotation (Line(points={{-10,0},{-61,0}}, color={28,108,200}));
  connect(pump.C_out, waterSink.C_in) annotation (Line(points={{10,0},{71,0}}, color={28,108,200}));
end WaterPump_direct;
