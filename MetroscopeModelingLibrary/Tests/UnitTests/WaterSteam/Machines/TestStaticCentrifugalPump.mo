within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.Machines;
model TestStaticCentrifugalPump
  extends Modelica.Icons.Example;
  input Modelica.Units.SI.AbsolutePressure P_source(start=2e5);
  input Modelica.Units.SI.Temperature T_source(start=20 + 273.15);
  input Modelica.Units.SI.MassFlowRate Q(start=100);
  MetroscopeModelingLibrary.WaterSteam.Machines.StaticCentrifugalPump
    staticCentrifugalPump
    annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  Common.Sensors.RotSpeedSensor rotSpeedSensor
    annotation (Placement(transformation(extent={{12,-32},{32,-12}})));
equation

  // Forward causality
  source.P_out = P_source;
  source.T_vol = T_source;
  source.Q_out = -Q;
  sink.h_vol=1e6;
  staticCentrifugalPump.VRot=1400;
  staticCentrifugalPump.VRotn=1400;
  staticCentrifugalPump.rm=0.85;
  staticCentrifugalPump.a1=-88.67;
  staticCentrifugalPump.a2=0;
  staticCentrifugalPump.a3=43.15;
  staticCentrifugalPump.b1=-3.7751;
  staticCentrifugalPump.b2=3.61;
  staticCentrifugalPump.b3=-0.0075464;
  staticCentrifugalPump.rhmin=0.20;

  // Reverse causality
  // To determine a3, give the outlet pressure
  // To determine b3, give the outlet enthalpy or temperature
  /*
  source.P_out = P_source;
  source.T_vol = T_source;
  source.Q_out = -Q;
  sink.h_vol=1e6;
  sink.P_in = 6e5;
  sink.T_in = 273.15+20;
  staticCentrifugalPump.VRot=1400;
  staticCentrifugalPump.VRotn=1400;
  staticCentrifugalPump.rm=0.85;
  staticCentrifugalPump.a1=-88.67;
  staticCentrifugalPump.a2=0;
  staticCentrifugalPump.b1=-3.7751;
  staticCentrifugalPump.b2=3.61;
  staticCentrifugalPump.rhmin=0.20;
  */


  connect(staticCentrifugalPump.C_out, sink.C_in)
    annotation (Line(points={{8.2,0},{54,0}}, color={238,46,47}));
  connect(source.C_out, staticCentrifugalPump.C_in)
    annotation (Line(points={{-60,0},{-12,0}}, color={238,46,47}));
  connect(staticCentrifugalPump.VRot, rotSpeedSensor.VRot) annotation (Line(
        points={{-2,-12},{4,-12},{4,-22},{11.2,-22}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,-20},{80,20}})));
end TestStaticCentrifugalPump;
