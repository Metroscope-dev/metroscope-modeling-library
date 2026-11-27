within MetroscopeModelingLibrary.Tests.SteelworksGases.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.SteelworksGasesTestIcon;
  import Interfaces = MetroscopeModelingLibrary.Utilities.Interfaces;
  import Medium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  import MetroscopeModelingLibrary.Utilities.Units;

  MetroscopeModelingLibrary.SteelworksGases.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-85,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.PressureSensor source_P_sensor(
    sensor_function="BC",
    P_start=1,
    P_0=100000,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-46,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.FlowSensor source_Q_sensor(
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

  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";

  Interfaces.BoundaryCondition source_P(start=source_P_sensor.P_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-46,27})));

  Interfaces.BoundaryCondition source_Q(start=source_Q_sensor.Q_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-14,27})));

  Interfaces.Observable T_out
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={20,27})));
equation
  source.h_out = source_h;
  // Impose the reference mixture on a molar basis.
  {source.X_molar_N2, source.X_molar_CH4, source.X_molar_CO2, source.X_molar_H2,
   source.X_molar_CO, source.X_molar_O2, source.X_molar_H2O} =
    Medium.referenceMolePercent/sum(Medium.referenceMolePercent);

  connect(source.C_out, source_P_sensor.C_in)
    annotation (Line(points={{-80,0},{-56,0}}, color={213,170,255}));
  connect(source_P_sensor.C_out, source_Q_sensor.C_in)
    annotation (Line(points={{-36,0},{-24,0}}, color={213,170,255}));
  connect(source_Q_sensor.C_out, T_out_sensor.C_in)
    annotation (Line(points={{-4,0},{10,0}}, color={213,170,255}));
  connect(T_out_sensor.C_out, sink.C_in)
    annotation (Line(points={{30,0},{52,0}}, color={213,170,255}));

  connect(source_P_sensor.P_sensor, source_P)
    annotation (Line(points={{-46,10},{-46,27}}, color={0,0,127}));
  connect(source_Q_sensor.Q_sensor, source_Q)
    annotation (Line(points={{-14,10},{-14,27}}, color={0,0,127}));
  connect(T_out_sensor.T_sensor, T_out)
    annotation (Line(points={{20,10},{20,27}}, color={0,0,127}));

  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-110,-40},{90,56}})));
end Source;
