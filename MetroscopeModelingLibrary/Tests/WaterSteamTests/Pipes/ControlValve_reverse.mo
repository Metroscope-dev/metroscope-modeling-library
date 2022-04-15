within MetroscopeModelingLibrary.Tests.WaterSteamTests.Pipes;
model ControlValve_reverse
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy source_h(start=1e3);
  input Real source_P(start=2, min=0, nominal=2) "barA";
  input Units.MassFlowRate source_Q(start=100) "kg/s";

  // Input: Observables
  input Real CV_P_out(start=1.8, min=0, nominal=2) "barA"; // Could be a boundary condition, and replaced by mass flow rate
  input Real CV_opening(start=0.15) "Cv";

  // Output: Component parameters
  output Units.Cv Cvmax;
  output Units.Cv Cv;

  // Components
  WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
  WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={90,-6.10623e-16})));

  WaterSteam.Pipes.ControlValve control_valve annotation (Placement(transformation(extent={{-16.5,-5.93938},{16.5,26.7272}})));

  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor source_P_sensor annotation (Placement(transformation(extent={{-66,-6},{-54,6}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterFlowSensor source_Q_sensor annotation (Placement(transformation(extent={{-42,-6},{-30,6}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor CV_P_out_sensor annotation (Placement(transformation(extent={{44,-6},{56,6}})));
  MetroscopeModelingLibrary.Sensors.Other.OpeningSensor CV_opening_sensor annotation (Placement(transformation(extent={{-10,50},{10,70}})));
equation
  // Boundary conditions
  source.h_out = source_h;
  source_P_sensor.P_barA = source_P;
  source_Q_sensor.Q = source_Q;
  CV_opening_sensor.Opening = CV_opening;

  // Input: Observables
  CV_P_out_sensor.P_barA = CV_P_out;

  // Output: Component parameters
  control_valve.Cvmax = Cvmax;
  control_valve.Cv = Cv;
  connect(source_P_sensor.C_in, source.C_out) annotation (Line(points={{-66,0},{-84.2,0},{-84.2,7.5e-06},{-85,7.5e-06}},   color={28,108,200}));
  connect(control_valve.C_in, source_Q_sensor.C_out) annotation (Line(points={{-16.5,-1.81818e-06},{-23.25,-1.81818e-06},{-23.25,0},{-30,0}},
                                                                                        color={28,108,200}));
  connect(source_Q_sensor.C_in, source_P_sensor.C_out) annotation (Line(points={{-42,0},{-54,0}}, color={28,108,200}));
  connect(CV_P_out_sensor.C_out, sink.C_in) annotation (Line(points={{56,0},{85,0}}, color={28,108,200}));
  connect(CV_P_out_sensor.C_in, control_valve.C_out) annotation (Line(points={{44,0},{33.25,0},{33.25,-1.81818e-06},{16.5,-1.81818e-06}}, color={28,108,200}));
  connect(control_valve.Opening, CV_opening_sensor.Opening) annotation (Line(points={{0,23.7575},{0,49.8}}, color={0,0,127}));
end ControlValve_reverse;
