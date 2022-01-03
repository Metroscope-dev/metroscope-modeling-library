within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.BoundaryConditions;
model TestLoopCloser
  extends Modelica.Icons.Example;
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.LoopCloser
                                       loopCloser
   annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={2,34})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-74,24},{-54,44}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{60,24},{80,44}})));
equation

  // Forward causality
  source.P_out = 1e5;
  source.T_vol = 20+273.15;
  source.Q_out = -20;

  // The loop closer does not maintain the flow rate so you have to specify the flow rate in the sink
  sink.Q_in =  10;

  sink.h_vol = 1e6;

  connect(loopCloser.C_out, sink.C_in)
    annotation (Line(points={{12.2,34},{60,34}}, color={238,46,47}));
  connect(source.C_out, loopCloser.C_in)
    annotation (Line(points={{-54,34},{-8,34}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                           Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-80,0},{80,60}})));
end TestLoopCloser;
