within MetroscopeModelingLibrary.Tests.SteelworksGases.Pipes;
model PressureCut
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.SteelworksGasesTestIcon;
  import Interfaces = MetroscopeModelingLibrary.Utilities.Interfaces;
  import Medium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  import MetroscopeModelingLibrary.Utilities.Units;

  MetroscopeModelingLibrary.SteelworksGases.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-108,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.PressureSensor source_P_sensor(
    sensor_function="BC",
    P_start=10,
    P_0=1e+06,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-74,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.FlowSensor source_Q_sensor(
    sensor_function="BC",
    Q_start=100,
    Q_0=100,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-40,0})));
  MetroscopeModelingLibrary.SteelworksGases.Pipes.PressureCut pressureCut
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={0,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.PressureSensor sink_P_sensor(
    sensor_function="BC",
    P_start=1,
    P_0=100000,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={40,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.TemperatureSensor T_out_sensor(
    sensor_function="Unidentified",
    T_start=225.2,
    T_0=498.35,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={74,0})));
  MetroscopeModelingLibrary.SteelworksGases.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={108,0})));

  input Units.SpecificEnthalpy source_h(start=0.5e6) "J/kg";

  Interfaces.BoundaryCondition source_P(start=source_P_sensor.P_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-74,27})));

  Interfaces.BoundaryCondition source_Q(start=source_Q_sensor.Q_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-40,27})));

  Interfaces.BoundaryCondition sink_P(start=sink_P_sensor.P_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={40,27})));

  Interfaces.Observable T_out
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={74,27})));
equation
  source.h_out = source_h;
  source.Xi_out = Medium.reference_X;

  connect(source.C_out, source_P_sensor.C_in)
    annotation (Line(points={{-103,0},{-84,0}}, color={213,170,255}));
  connect(source_P_sensor.C_out, source_Q_sensor.C_in)
    annotation (Line(points={{-64,0},{-50,0}}, color={213,170,255}));
  connect(source_Q_sensor.C_out, pressureCut.C_in)
    annotation (Line(points={{-30,0},{-10,0}}, color={213,170,255}));
  connect(pressureCut.C_out, sink_P_sensor.C_in)
    annotation (Line(points={{10,0},{30,0}}, color={213,170,255}));
  connect(sink_P_sensor.C_out, T_out_sensor.C_in)
    annotation (Line(points={{50,0},{64,0}}, color={213,170,255}));
  connect(T_out_sensor.C_out, sink.C_in)
    annotation (Line(points={{84,0},{103,0}}, color={213,170,255}));

  connect(source_P_sensor.P_sensor, source_P)
    annotation (Line(points={{-74,10},{-74,27}}, color={0,0,127}));
  connect(source_Q_sensor.Q_sensor, source_Q)
    annotation (Line(points={{-40,10},{-40,27}}, color={0,0,127}));
  connect(sink_P_sensor.P_sensor, sink_P)
    annotation (Line(points={{40,10},{40,27}}, color={0,0,127}));
  connect(T_out_sensor.T_sensor, T_out)
    annotation (Line(points={{74,10},{74,27}}, color={0,0,127}));

  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-130,-48},{130,58}})));
end PressureCut;
