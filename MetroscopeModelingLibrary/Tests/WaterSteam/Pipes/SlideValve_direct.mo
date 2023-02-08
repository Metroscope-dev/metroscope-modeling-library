within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model SlideValve_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  /* Note
  The slide valve is fully open, therefore the causality is different from a control valve.
  The slide valve cannot act as a pressure cut.
  Pressure does propagate through the slide valve in the direct mode.
  */

  // Boundary conditions
  input Utilities.Units.SpecificEnthalpy source_h(start=1e6);
  input Utilities.Units.Pressure source_P(
    start=10e5,
    min=0,
    nominal=10e5) "Pa";
  input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  // Calibrated parameter
  parameter Utilities.Units.Cv Cv=11e3;

  // Components
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-68,-9.99996},{-48,9.99996}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,-6.10623e-16})));

  MetroscopeModelingLibrary.WaterSteam.Pipes.SlideValve slide_valve annotation (Placement(transformation(extent={{-16.5,-5.93938},{16.5,26.7272}})));

equation
  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;

  // Calibrated Parameters
  slide_valve.Cv = Cv;

  connect(slide_valve.C_in, source.C_out) annotation (Line(points={{-16.5,-1.81818e-06},{-34.75,-1.81818e-06},{-34.75,0},{-53,0}}, color={28,108,200}));
  connect(slide_valve.C_out, sink.C_in) annotation (Line(points={{16.5,-1.81818e-06},{34.75,-1.81818e-06},{34.75,0},{53,0}}, color={28,108,200}));
end SlideValve_direct;
