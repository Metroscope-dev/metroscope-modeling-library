within MetroscopeModelingLibrary.Tests.Fuel.Pipes;
model Pipe_direct "MML4 isolated natural-gas pipe direct test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FuelTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Preserve the legacy direct test point; these are illustrative test data.
  MetroscopeModelingLibrary.Fuel.Pipes.Pipe pipe(
    faulty=false,
    P_in_0=100000,
    P_out_0=97697.289,
    T_0=462.70651,
    Q_0=100,
    h_0=1000000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={0,0})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source(
    P_0=100000,
    T_0=462.70651,
    Q_0=100,
    h_0=1000000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-160,0})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={120,0})));

  // No enthalpy sensor exists: retain an ordinary SI enthalpy input.
  input Units.SpecificEnthalpy source_h(start=source.h_0)
    "Inlet specific enthalpy (J/kg)";

  MetroscopeModelingLibrary.SensorsControl.Fuel.PressureSensor source_P_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=1,
    signal_unit="barA",
    display_unit="barA",
    P_0=100000,
    Q_0=100,
    h_0=1000000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-120,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_P
    "Inlet pressure (barA)"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-120,30}), iconTransformation(extent={{-140,60},{-100,100}})));
  MetroscopeModelingLibrary.SensorsControl.Fuel.FlowSensor source_Q_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=100,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=100000,
    Q_0=100,
    h_0=1000000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition source_Q
    "Positive inlet mass flow rate (kg/s)"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-80,30}), iconTransformation(extent={{-140,0},{-100,40}})));

  MetroscopeModelingLibrary.SensorsControl.Fuel.PressureSensor P_out_sensor(
    sensor_function="Calibration",
    causality="Kfr",
    show_causality=true,
    P_start=0.97697289,
    signal_unit="barA",
    display_unit="barA",
    P_0=97697.289,
    Q_0=100,
    h_0=1000000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={40,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable P_out
    "Outlet pressure (barA): identifies Kfr in reverse"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={40,30}), iconTransformation(extent={{100,60},{140,100}})));
  MetroscopeModelingLibrary.SensorsControl.Fuel.TemperatureSensor T_out_sensor(
    sensor_function="Unidentified",
    causality="",
    show_causality=true,
    T_start=189.55651,
    signal_unit="degC",
    display_unit="degC",
    T_0=462.70651,
    P_0=97697.289,
    Q_0=100,
    h_0=1000000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable T_out
    "Outlet temperature (degC)"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={80,30}), iconTransformation(extent={{100,0},{140,40}})));

  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal Kfr(start=0.1)
    "Friction coefficient: identified in reverse, fixed in direct/faulty"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={0,30}), iconTransformation(extent={{-20,-100},{20,-60}})));
equation
  source.h_out = source_h;
  source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};
  Kfr = 0.1;

  connect(source.C_out, source_P_sensor.C_in)
    annotation (Line(points={{-155,0},{-130,0}}, color={213,213,0}));
  connect(source_P_sensor.C_out, source_Q_sensor.C_in)
    annotation (Line(points={{-110,0},{-90,0}}, color={213,213,0}));
  connect(source_Q_sensor.C_out, pipe.C_in)
    annotation (Line(points={{-70,0},{-10,0}}, color={213,213,0}));
  connect(pipe.C_out, P_out_sensor.C_in)
    annotation (Line(points={{10,0},{30,0}}, color={213,213,0}));
  connect(P_out_sensor.C_out, T_out_sensor.C_in)
    annotation (Line(points={{50,0},{70,0}}, color={213,213,0}));
  connect(T_out_sensor.C_out, sink.C_in)
    annotation (Line(points={{90,0},{115,0}}, color={213,213,0}));
  connect(source_P_sensor.P_sensor, source_P)
    annotation (Line(points={{-120,10},{-120,30}}, color={0,0,127}));
  connect(source_Q_sensor.Q_sensor, source_Q)
    annotation (Line(points={{-80,10},{-80,30}}, color={0,0,127}));
  connect(P_out_sensor.P_sensor, P_out)
    annotation (Line(points={{40,10},{40,30}}, color={0,0,127}));
  connect(T_out_sensor.T_sensor, T_out)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(Kfr, pipe.Kfr)
    annotation (Line(points={{0,30},{0,4}}, color={0,0,127}));

  assert(abs(source.Q_out + sink.Q_in) < 1e-6,
    "The pipe test must conserve mass.");
  assert(abs(source.h_out - sink.h_in) < 1e-4,
    "The adiabatic pipe test must conserve specific enthalpy.");
  assert(sink.P_in > 0 and sink.P_in < source.P_out,
    "The passive pipe must have a positive pressure loss and outlet pressure.");
  assert(Kfr > 0, "The identified friction coefficient must be positive.");
  annotation (
    experiment(StopTime=1, Tolerance=1e-6),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-190,-50},{150,70}})),
    Documentation(info="<html>
<p>MML4 isolated test with exactly one Fuel.Pipes.Pipe. The legacy direct
operating point is preserved: 1 barA, 100 kg/s, 1 MJ/kg and the original
six-species natural-gas composition. This is an illustrative regression point,
not plant data. The corresponding gas temperature is about 189.56 degC.</p>
<p>Source pressure and positive mass flow use graphical BoundaryCondition
connectors. Enthalpy remains an ordinary SI input because no enthalpy sensor
is available. Outlet pressure calibrates Kfr in reverse and is observable in
direct/faulty; its Calibration metadata and causality are retained.</p>
<p>Kfr=0.1 gives approximately 0.97697289 barA at the nominal point.
The faulty test activates the supported fouling failure at 20 percent,
increasing the pressure loss by 20 percent. delta_z is not part of the pipe API.</p>
<p>API: local MML 4.0.0-beta, commit
f35e387e76e4bdf87b97cc9dd7ce4ff3961428e6, Modelica 4.1.0.
Graphical conventions follow the approved MML4 AirCompressor family.</p>
</html>"));
end Pipe_direct;
