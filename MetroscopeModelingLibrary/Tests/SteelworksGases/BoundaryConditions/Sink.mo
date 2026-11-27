within MetroscopeModelingLibrary.Tests.SteelworksGases.BoundaryConditions;
model Sink
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.SteelworksGasesTestIcon;
  import Interfaces = MetroscopeModelingLibrary.Utilities.Interfaces;
  import Medium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  import MetroscopeModelingLibrary.Utilities.Units;

  MetroscopeModelingLibrary.SteelworksGases.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-85,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.PressureSensor sink_P_sensor(
    sensor_function="BC",
    P_start=1,
    P_0=100000,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-46,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.FlowSensor sink_Q_sensor(
    sensor_function="BC",
    Q_start=100,
    Q_0=100,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-14,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.TemperatureSensor T_out_sensor(
    sensor_function="Unidentified",
    T_start=658.9,
    T_0=932.05,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={20,0})));
  MetroscopeModelingLibrary.SteelworksGases.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={57,0})));

  input Units.SpecificEnthalpy sink_h(start=1e6) "J/kg";

  Interfaces.BoundaryCondition sink_P(start=sink_P_sensor.P_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-46,27})));

  Interfaces.BoundaryCondition sink_Q(start=sink_Q_sensor.Q_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-14,27})));

  Interfaces.Observable T_out
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={20,27})));
equation
  sink.h_in = sink_h;
  sink.Xi_in = Medium.reference_X;

  connect(source.C_out, sink_P_sensor.C_in)
    annotation (Line(points={{-80,0},{-56,0}}, color={213,170,255}));
  connect(sink_P_sensor.C_out, sink_Q_sensor.C_in)
    annotation (Line(points={{-36,0},{-24,0}}, color={213,170,255}));
  connect(sink_Q_sensor.C_out, T_out_sensor.C_in)
    annotation (Line(points={{-4,0},{10,0}}, color={213,170,255}));
  connect(T_out_sensor.C_out, sink.C_in)
    annotation (Line(points={{30,0},{52,0}}, color={213,170,255}));

  connect(sink_P_sensor.P_sensor, sink_P)
    annotation (Line(points={{-46,10},{-46,27}}, color={0,0,127}));
  connect(sink_Q_sensor.Q_sensor, sink_Q)
    annotation (Line(points={{-14,10},{-14,27}}, color={0,0,127}));
  connect(T_out_sensor.T_sensor, T_out)
    annotation (Line(points={{20,10},{20,27}}, color={0,0,127}));

  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-110,-40},{90,56}})));
end Sink;
