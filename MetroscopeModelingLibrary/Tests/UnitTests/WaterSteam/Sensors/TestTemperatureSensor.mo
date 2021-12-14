within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.Sensors;
model TestTemperatureSensor
  extends Modelica.Icons.Example;
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-70,-50},{-50,-30}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (
     Placement(visible=true, transformation(
        origin={80,-40},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.SingularPressureLoss PL
    annotation (Placement(transformation(extent={{-24,-50},{-4,-30}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.TemperatureWater TemperatureSensor
    annotation (Placement(transformation(extent={{28,-40},{38,-24}})));
equation

  // Use sensor as a sensor
  source.Q_out = -10;
  source.P_out = 2e5;
  source.T_out = 273.15 + 50;
  PL.Kfr = 1e-1;
  sink.h_vol=1e6;

  // Use sensor as a boundary condition
  /*
  source.Q_out = -10 ;
  source.P_out = 2e5;
  PL.Kfr = 1e-1;
  sink.h_vol=1e6;
  
  TemperatureSensor.T = 273.15 + 50 ;
  */

  connect(source.C_out, PL.C_in)
    annotation (Line(points={{-50,-40},{-24,-40}}, color={238,46,47}));
  connect(PL.C_out, sink.C_in)
    annotation (Line(points={{-3.8,-40},{70,-40}}, color={238,46,47}));
  connect(PL.C_out, TemperatureSensor.C_in)
    annotation (Line(points={{-3.8,-40},{32,-40}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-60},{100,-20}})));
end TestTemperatureSensor;
