within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.PressureLosses;
model TestSingularPressureLoss
  input Modelica.SIunits.AbsolutePressure P_source(start = 2e5);
  input Modelica.SIunits.SpecificEnthalpy T_source(start = 20+273.15);
  input Modelica.SIunits.MassFlowRate Q(start=100);
  // For reverse causality :
  //input Modelica.SIunits.AbsolutePressure P_sink(start = 1e5);
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureLoss
    singularPressureLoss
    annotation (Placement(transformation(extent={{-18,-10},{2,10}})));
equation

  // Unit test for a Singular Pressure loss
  // Causality is forward, which means the Kfr is specified.
  // In order to calibrate the Kfr, simply give the output pressure instead of the Kfr.

  source.P_out = P_source;
  source.T_vol = T_source;
  source.Q_out = -Q;

  sink.h_vol = 1e6;

  // For forward causality :
  singularPressureLoss.Kfr = 1e3;

  // For reverse causality :
  //sink.P_in =  P_sink;

  connect(singularPressureLoss.C_out, sink.C_in)
    annotation (Line(points={{2.2,0},{54,0}}, color={238,46,47}));
  connect(source.C_out, singularPressureLoss.C_in)
    annotation (Line(points={{-60,0},{-18,0}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},
            {80,20}})),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},{80,20}})));
end TestSingularPressureLoss;
