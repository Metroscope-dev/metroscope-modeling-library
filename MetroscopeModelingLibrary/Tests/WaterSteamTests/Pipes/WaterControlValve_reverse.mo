within MetroscopeModelingLibrary.Tests.WaterSteamTests.Pipes;
model WaterControlValve_reverse
  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e3);
  input Real source_P(start=2, min=0, nominal=2) "barA";
  input Units.MassFlowRate source_Q(start=100) "kg/s";
  input Real Opening(start=0.15) "Cv";

  // Input: Observables
  input Real CV_P_out(start=1.8, min=0, nominal=2) "barA";

  // Output: Component parameters
  output Units.Cv Cv;
  output Units.Cv Cvmax;

  // Components
  WaterSteam.BoundaryConditions.WaterSource source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  WaterSteam.BoundaryConditions.WaterSink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  WaterSteam.Pipes.WaterControlValve control_valve annotation (Placement(transformation(extent={{-16.5,-5.93938},{16.5,26.7272}})));

  Sensors.WaterSteam.WaterPressureSensor source_P_sensor annotation (Placement(transformation(extent={{-66,-6},{-54,6}})));
  Sensors.WaterSteam.WaterFlowSensor source_Q_sensor annotation (Placement(transformation(extent={{-42,-6},{-30,6}})));
  Sensors.WaterSteam.WaterPressureSensor CV_P_out_sensor annotation (Placement(transformation(extent={{44,-6},{56,6}})));
equation
  // Boundary conditions
  source.h_out = source_h;
  source_P_sensor.P_barA = source_P;
  source_Q_sensor.Q = source_Q;
  control_valve.Opening = Opening;

  // Input: Observables
  CV_P_out_sensor.P_barA = CV_P_out;

  // Output: Component parameters
  control_valve.Cvmax = Cvmax;
  control_valve.Cv = Cv;
  connect(source_P_sensor.C_in, source.C_out) annotation (Line(points={{-66,0},{-84.2,0},{-84.2,7.5e-06},{-85.6,7.5e-06}}, color={28,108,200}));
  connect(control_valve.C_in, source_Q_sensor.C_out) annotation (Line(points={{-16.5,-1.81818e-06},{-23.25,-1.81818e-06},{-23.25,0},{-30,0}},
                                                                                        color={28,108,200}));
  connect(source_Q_sensor.C_in, source_P_sensor.C_out) annotation (Line(points={{-42,0},{-54,0}}, color={28,108,200}));
  connect(CV_P_out_sensor.C_out, sink.C_in) annotation (Line(points={{56,0},{85,0}}, color={28,108,200}));
  connect(CV_P_out_sensor.C_in, control_valve.C_out) annotation (Line(points={{44,0},{33.25,0},{33.25,-1.81818e-06},{16.5,-1.81818e-06}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(lineColor = {0,0,255},
                fillColor = {75,138,73},
                pattern = LinePattern.None,
                fillPattern = FillPattern.Solid,
                points={{-36,60},{64,0},{-36,-60},{-36,60}}),
        Polygon(
          points={{26,-92},{60,-84},{26,-74},{26,-92},{26,-92}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{60,-84},{94,-74},{94,-92},{60,-84},{60,-84}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{60,-84},{78,-42},{44,-42},{60,-84}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{78,-42},{44,-42},{44,-28},{48,-20},{56,-16},{66,-16},{74,-20},{78,-28},{78,-42},{78,-28},{78,-28},{78,-42}},
          lineColor={0,0,255},
          fillColor={127,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{22,-78},{32,-88}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{88,-78},{98,-88}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}),
                                Diagram(coordinateSystem(preserveAspectRatio=false)));
end WaterControlValve_reverse;
