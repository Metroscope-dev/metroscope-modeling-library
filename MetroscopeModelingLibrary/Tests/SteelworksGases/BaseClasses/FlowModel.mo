within MetroscopeModelingLibrary.Tests.SteelworksGases.BaseClasses;
model FlowModel
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.SteelworksGasesTestIcon;
  import Interfaces = MetroscopeModelingLibrary.Utilities.Interfaces;
  import Medium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  import MetroscopeModelingLibrary.Utilities.Units;

  MetroscopeModelingLibrary.SteelworksGases.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-19,-19},{19,19}}, origin={-136,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.PressureSensor source_P_sensor(
    sensor_function="BC",
    P_start=1,
    P_0=100000,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-95,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.FlowSensor source_Q_sensor(
    sensor_function="BC",
    Q_start=100,
    Q_0=100,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-57,0})));
  MetroscopeModelingLibrary.SteelworksGases.BaseClasses.FlowModel flowModel
    annotation (Placement(transformation(extent={{-23,-23},{23,23}}, origin={0,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.TemperatureSensor T_out_sensor(
    sensor_function="Unidentified",
    T_start=659.7,
    T_0=932.85,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={56,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.PressureSensor P_out_sensor(
    sensor_function="Unidentified",
    P_start=0.9,
    P_0=90000,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={92,0})));
  MetroscopeModelingLibrary.SteelworksGases.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-21,-20},{21,20}}, origin={134,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.DeltaPressureSensor pressure_drop_sensor(
    sensor_function="BC",
    DP_start=0.1,
    DP_0=10000,
    signal_unit="bar",
    display_unit="bar")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={0,-49})));

  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";

  input Units.Power W(start=1e5) "Heat transfer into the gas";

  Interfaces.BoundaryCondition source_P(start=source_P_sensor.P_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-95,27})));

  Interfaces.BoundaryCondition source_Q(start=source_Q_sensor.Q_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-57,27})));

  Interfaces.Observable T_out
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={56,27})));

  Interfaces.Observable P_out
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={92,27})));

  Interfaces.BoundaryCondition pressure_drop(start=pressure_drop_sensor.DP_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={0,-76})));
equation
  source.h_out = source_h;
  source.Xi_out = Medium.reference_X;
  flowModel.W = W;

  connect(source.C_out, source_P_sensor.C_in)
    annotation (Line(points={{-126.5,0},{-105,0}}, color={213,170,255}));
  connect(source_P_sensor.C_out, source_Q_sensor.C_in)
    annotation (Line(points={{-85,0},{-67,0}}, color={213,170,255}));
  connect(source_Q_sensor.C_out, flowModel.C_in)
    annotation (Line(points={{-47,0},{-23,0}}, color={213,170,255}));
  connect(flowModel.C_out, T_out_sensor.C_in)
    annotation (Line(points={{23,0},{46,0}}, color={213,170,255}));
  connect(T_out_sensor.C_out, P_out_sensor.C_in)
    annotation (Line(points={{66,0},{82,0}}, color={213,170,255}));
  connect(P_out_sensor.C_out, sink.C_in)
    annotation (Line(points={{102,0},{123.5,0}}, color={213,170,255}));
  connect(flowModel.C_in, pressure_drop_sensor.C_out)
    annotation (Line(points={{-23,0},{-33,0},{-33,-49},{-10,-49}}, color={213,170,255}));
  connect(flowModel.C_out, pressure_drop_sensor.C_in)
    annotation (Line(points={{23,0},{33,0},{33,-49},{10,-49}}, color={213,170,255}));

  connect(source_P_sensor.P_sensor, source_P)
    annotation (Line(points={{-95,10},{-95,27}}, color={0,0,127}));
  connect(source_Q_sensor.Q_sensor, source_Q)
    annotation (Line(points={{-57,10},{-57,27}}, color={0,0,127}));
  connect(T_out_sensor.T_sensor, T_out)
    annotation (Line(points={{56,10},{56,27}}, color={0,0,127}));
  connect(P_out_sensor.P_sensor, P_out)
    annotation (Line(points={{92,10},{92,27}}, color={0,0,127}));
  connect(pressure_drop_sensor.DP_sensor, pressure_drop)
    annotation (Line(points={{0,-59},{0,-76}}, color={0,0,127}));

  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-162,-90},{162,58}})));
end FlowModel;
