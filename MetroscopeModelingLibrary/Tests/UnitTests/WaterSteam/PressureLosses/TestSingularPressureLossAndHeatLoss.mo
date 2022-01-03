within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.PressureLosses;
model TestSingularPressureLossAndHeatLoss
  extends Modelica.Icons.Example;
  input Modelica.Units.SI.AbsolutePressure P_source(start=2e5);
  input Modelica.Units.SI.Temperature T_source(start=20 + 273.15);
  input Modelica.Units.SI.MassFlowRate Q(start=100);
  // For reverse causality, add the following inputs :
  //input Modelica.SIunits.AbsolutePressure P_sink(start = 1e5);
  //input Modelica.SIunits.Temperature T_sink(start=18+273.15);
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureAndHeatLoss
    singularPressureAndHeatLoss
    annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
equation

  // Unit test for Singular Pressure Loss and heat loss

  // Causality is forward which means Kfr and heat loss W are specified.
  // In order to calibrate Kfr, simply give the output pressure.
  // In order to calibrate W, simply give the output temperature or enthalpy

  source.P_out = P_source;
  source.T_vol = T_source;
  source.Q_out = -Q;

  sink.h_vol=1e6;

  //For forward causality :
  singularPressureAndHeatLoss.Kfr = 1e0;
  singularPressureAndHeatLoss.W = -1e6;

  //For reverse causality :
  //sink.T_in = T_sink;
  //sink.P_in =  P_sink;

  connect(singularPressureAndHeatLoss.C_out, sink.C_in)
    annotation (Line(points={{8.2,0},{54,0}}, color={238,46,47}));
  connect(source.C_out, singularPressureAndHeatLoss.C_in)
    annotation (Line(points={{-60,0},{-12,0}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},{80,20}})),
    experiment(__Dymola_Algorithm="Euler"));
end TestSingularPressureLossAndHeatLoss;
