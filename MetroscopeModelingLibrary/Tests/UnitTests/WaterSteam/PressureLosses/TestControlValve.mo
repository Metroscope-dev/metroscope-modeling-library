within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.PressureLosses;
model TestControlValve
  input Modelica.Units.SI.AbsolutePressure P_source(start=2e5);
  input Modelica.Units.SI.SpecificEnthalpy T_source(start=20 + 273.15);
  input Modelica.Units.SI.MassFlowRate Q(start=100);
  // For reverse causality :
  //input Modelica.SIunits.AbsolutePressure P_sink(start = 1e5);
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve controlValve
    annotation (Placement(transformation(extent={{-16,-4},{4,18}})));
equation

  // Unit test for Control Valve
  // Causality is forward which means Cvmax is specified.
  // In order to calibrate Cvmax, simply give the output pressure.

  source.P_out = P_source;
  source.T_vol = T_source;
  source.Q_out = -Q;

  sink.h_vol = 1e6;

  controlValve.Opening = 0.15;

  //For forward causality :
  controlValve.Cvmax = 8005.42;

  // For reverse causality :
  //sink.P_in =  P_sink;

  connect(controlValve.C_out, sink.C_in)
    annotation (Line(points={{4.2,0},{54,0}}, color={238,46,47}));
  connect(source.C_out, controlValve.C_in)
    annotation (Line(points={{-60,0},{-16,0}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},
            {80,20}})),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},{80,20}})));
end TestControlValve;
