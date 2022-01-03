within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.PressureLosses;
model TestPressureCut
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PressureCut
    pressureCut
    annotation (Placement(transformation(extent={{-18,-10},{2,10}})));
equation

  source.P_out = 2e5;
  source.T_vol = 273.15 + 20;
  source.Q_out = -100;

  sink.h_vol = 1e6;

  sink.P_in =  1.9e5;

  connect(pressureCut.C_out, sink.C_in)
    annotation (Line(points={{2.2,0},{54,0}}, color={238,46,47}));
  connect(source.C_out, pressureCut.C_in)
    annotation (Line(points={{-60,0},{-18,0}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},{80,20}})));
end TestPressureCut;
