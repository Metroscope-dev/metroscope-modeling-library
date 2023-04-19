within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model ControlValve_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  /* In direct mode, control valves are usually used as a sort of pressure cut in the system. The pressure
  will be known on both sides (for instance on a NPP, it will come from the condenser through the turbine line and
  reach the high pressure steam valve, and on the other side the pressure will come from the steam generator.
  As the characteristics of the valve is known and the mass flow rate is known, its opening will adapt to match the pressure
  difference. */

  // Boundary conditions
  input Utilities.Units.SpecificEnthalpy source_h(start=1e6);
  input Utilities.Units.Pressure source_P(
    start=10e5,
    min=0,
    nominal=10e5) "Pa";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Utilities.Units.Pressure sink_P(start=9e5) "Pa";

  // Parameters
  parameter Utilities.Units.Cv Cvmax=3e4 "Cvmax";

  // Components
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-68,-9.99996},{-48,9.99996}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,-6.10623e-16})));

  .MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve control_valve annotation (Placement(transformation(extent={{-16.5,-5.93938},{16.5,26.7272}})));

equation

  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;
  sink.P_in = sink_P;

  // Parameters
  control_valve.Cv_max = Cvmax;

  connect(control_valve.C_out, sink.C_in) annotation (Line(points={{16.5,-1.81818e-06},{34.75,-1.81818e-06},{34.75,0},{53,0}}, color={28,108,200}));
  connect(control_valve.C_in, source.C_out) annotation (Line(points={{-16.5,-1.81818e-06},{-34.75,-1.81818e-06},{-34.75,0},{-53,0}},color={28,108,200}));
end ControlValve_direct;
