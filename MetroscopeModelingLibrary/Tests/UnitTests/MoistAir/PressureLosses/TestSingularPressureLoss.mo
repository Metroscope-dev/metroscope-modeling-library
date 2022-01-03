within MetroscopeModelingLibrary.Tests.UnitTests.MoistAir.PressureLosses;
model TestSingularPressureLoss
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  MetroscopeModelingLibrary.MoistAir.PressureLosses.SingularPressureLoss singularPressureLoss(rhom(
        start=1.2))
    annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
equation

  // Forward causality
  source.Q_out = -10;
  source.P_out = 0.989314e5;
  source.T_vol = 6.6585226+273.15;
  source.relative_humidity = 0.100;

  singularPressureLoss.Kfr = 1;

  sink.Xi_vol[1] = 0.01;
  sink.h_vol = 1e5;


  // Reverse causality
  // Kfr is determined using the sink inlet pressure
  /*
  source.Q_out = -10 ;
  source.P_out = 0.989314e5;
  source.T_vol = 6.6585226+273.15;
  source.relative_humidity = 0.100;
  
  sink.P_in = 0.9e5;
  
  sink.Xi_vol[1] = 0.01;
  sink.h_vol = 1e5;
  */

  connect(source.C_out, singularPressureLoss.C_in)
    annotation (Line(points={{-56,0},{-12,0}}, color={63,81,181}));
  connect(singularPressureLoss.C_out, sink.C_in)
    annotation (Line(points={{8.2,0},{62,0}},  color={63,81,181}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},{80,20}})));
end TestSingularPressureLoss;
