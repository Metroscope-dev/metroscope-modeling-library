within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.PressureLosses;
model TestPipePressureLoss
  extends Modelica.Icons.Example;

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss
    pipePressureLoss
    annotation (Placement(transformation(extent={{-16,-10},{4,10}})));
equation

  // Unit test for Control Valve

  // Causality is forward which means Kfr is specified.
  // In order to calibrate Kfr, simply give the output pressure.

  source.P_out = 2e5;
  source.T_vol = 20+273.15;
  source.Q_out = -100;

  sink.h_vol = 1e6;

  pipePressureLoss.z1 = 23;
  pipePressureLoss.z2 = 25;

  // For forward causality :
  pipePressureLoss.Kfr = 1e-3;

  // For reverse causality :
  //sink.P_in =  P_sink;

  connect(pipePressureLoss.C_out, sink.C_in)
    annotation (Line(points={{4.2,0},{54,0}}, color={238,46,47}));
  connect(source.C_out, pipePressureLoss.C_in)
    annotation (Line(points={{-60,0},{-16,0}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},{80,20}})));
end TestPipePressureLoss;
