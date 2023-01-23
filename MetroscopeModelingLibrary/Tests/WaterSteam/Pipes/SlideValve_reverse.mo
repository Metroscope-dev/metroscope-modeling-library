within MetroscopeModelingLibrary.Tests.WaterSteam.Pipes;
model SlideValve_reverse
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  /* Note
  The slide valve is fully open, therefore the causality is different from a control valve.
  The slide valve cannot act as a pressure cut.
  Pressure does propagate through the slide valve in the direct mode.
  */

  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e6);
  input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  // Inputs for calibration
  input Units.Pressure outlet_pressure(start=9e5) "Pa";

  // Calibrated parameter
  output Units.Cv Cvmax "Cvmax";

  // Components
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-68,-9.99996},{-48,9.99996}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={58,-6.10623e-16})));

  MetroscopeModelingLibrary.WaterSteam.Pipes.SlideValve slide_valve annotation (Placement(transformation(extent={{-16.5,-5.93938},{16.5,26.7272}})));

  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor outlet_pressure_sensor annotation (Placement(transformation(extent={{26,-10},{46,10}})));
equation
  // Boundary conditions
  source.h_out = source_h;
  source.P_out = source_P;
  source.Q_out = source_Q;

  // Inputs for calibration
  outlet_pressure_sensor.P = outlet_pressure;

  // Calibrated Parameters
  slide_valve.Cvmax = Cvmax;

  connect(slide_valve.C_in, source.C_out) annotation (Line(points={{-16.5,-1.81818e-06},{-34.75,-1.81818e-06},{-34.75,0},{-53,0}}, color={28,108,200}));
  connect(slide_valve.C_out, outlet_pressure_sensor.C_in) annotation (Line(points={{16.5,-1.81818e-06},{22,-1.81818e-06},{22,0},{26,0}}, color={28,108,200}));
  connect(sink.C_in, outlet_pressure_sensor.C_out) annotation (Line(points={{53,0},{46,0}}, color={28,108,200}));
end SlideValve_reverse;
