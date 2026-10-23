within MetroscopeModelingLibrary.Tests.Multifluid.Machines;
model CombustionChamber
  "MML4 isolated CombustionChamber direct test"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  MetroscopeModelingLibrary.MultiFluid.Machines.CombustionChamber combustionChamber
    annotation (Placement(transformation(extent={{-20,-20},{20,20}}, rotation=0, origin={0,0})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source air_source(
    P_0=1700000,
    Q_0=500,
    T_0=679.15,
    h_0=747065)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,0})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor air_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=17,
    signal_unit="barA",
    display_unit="barA",
    P_0=1700000,
    Q_0=500,
    h_0=747065)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition air_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,30}), iconTransformation(extent={{-140,220},{-100,260}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor air_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=406,
    signal_unit="degC",
    display_unit="degC",
    P_0=1700000,
    Q_0=500,
    h_0=747065,
    T_0=679.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition air_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,30}), iconTransformation(extent={{-140,175},{-100,215}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.FlowSensor air_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=500,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=1700000,
    Q_0=500,
    h_0=747065)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition air_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,30}), iconTransformation(extent={{-140,130},{-100,170}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source fuel_source(
    P_0=2000000,
    Q_0=10,
    T_0=303.15,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-280,-180})));
  MetroscopeModelingLibrary.SensorsControl.Fuel.PressureSensor fuel_source_pressure_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    P_start=20,
    signal_unit="barA",
    display_unit="barA",
    P_0=2000000,
    Q_0=10,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-240,-180})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition fuel_source_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-240,-150}), iconTransformation(extent={{-140,85},{-100,125}})));
  MetroscopeModelingLibrary.SensorsControl.Fuel.TemperatureSensor fuel_source_temperature_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    T_start=30,
    signal_unit="degC",
    display_unit="degC",
    P_0=2000000,
    Q_0=10,
    h_0=500000,
    T_0=303.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-200,-180})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition fuel_source_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-200,-150}), iconTransformation(extent={{-140,40},{-100,80}})));
  MetroscopeModelingLibrary.SensorsControl.Fuel.FlowSensor fuel_source_flow_sensor(
    sensor_function="BC",
    causality="",
    show_causality=true,
    Q_start=10,
    signal_unit="kg/s",
    display_unit="kg/s",
    P_0=2000000,
    Q_0=10,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={-160,-180})));
  MetroscopeModelingLibrary.Utilities.Interfaces.BoundaryCondition fuel_source_flow
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-160,-150}), iconTransformation(extent={{-140,-5},{-100,35}})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal combustionChamber_eta(
    start=0.99)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={-30,65})));
  MetroscopeModelingLibrary.Utilities.Interfaces.GenericReal combustionChamber_Kfr(
    start=0.2)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={40,65})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor sink_pressure_sensor(
    sensor_function="Calibration",
    causality="combustionChamber_Kfr",
    show_causality=true,
    P_start=16.5,
    signal_unit="barA",
    display_unit="barA",
    P_0=1650000,
    Q_0=510,
    h_0=500000)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={80,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable sink_pressure
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={80,30}), iconTransformation(extent={{100,220},{140,260}})));
  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor sink_temperature_sensor(
    sensor_function="Calibration",
    causality="combustionChamber_eta",
    show_causality=true,
    T_start=1200,
    signal_unit="degC",
    display_unit="degC",
    P_0=1650000,
    Q_0=510,
    h_0=500000,
    T_0=1473.15)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={130,0})));
  MetroscopeModelingLibrary.Utilities.Interfaces.Observable sink_temperature
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=0, origin={130,30}), iconTransformation(extent={{100,175},{140,215}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0, origin={220,0})));
equation
  air_source.Xi_out = {0.768,0.232,0,0,0};
  fuel_source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};
  combustionChamber_eta = 0.99;
  combustionChamber_Kfr = 0.2;
  connect(air_source_pressure_sensor.P_sensor, air_source_pressure)
    annotation (Line(points={{-240,10},{-240,30}}, color={0,0,127}));
  connect(air_source.C_out, air_source_pressure_sensor.C_in)
    annotation (Line(points={{-275,0},{-250,0}}, color={28,108,200}));
  connect(air_source_temperature_sensor.T_sensor, air_source_temperature)
    annotation (Line(points={{-200,10},{-200,30}}, color={0,0,127}));
  connect(air_source_pressure_sensor.C_out, air_source_temperature_sensor.C_in)
    annotation (Line(points={{-230,0},{-210,0}}, color={28,108,200}));
  connect(air_source_flow_sensor.Q_sensor, air_source_flow)
    annotation (Line(points={{-160,10},{-160,30}}, color={0,0,127}));
  connect(air_source_temperature_sensor.C_out, air_source_flow_sensor.C_in)
    annotation (Line(points={{-190,0},{-170,0}}, color={28,108,200}));
  connect(air_source_flow_sensor.C_out, combustionChamber.inlet)
    annotation (Line(points={{-150,0},{-20,0}}, color={28,108,200}));
  connect(fuel_source_pressure_sensor.P_sensor, fuel_source_pressure)
    annotation (Line(points={{-240,-170},{-240,-150}}, color={0,0,127}));
  connect(fuel_source.C_out, fuel_source_pressure_sensor.C_in)
    annotation (Line(points={{-275,-180},{-250,-180}}, color={28,108,200}));
  connect(fuel_source_temperature_sensor.T_sensor, fuel_source_temperature)
    annotation (Line(points={{-200,-170},{-200,-150}}, color={0,0,127}));
  connect(fuel_source_pressure_sensor.C_out, fuel_source_temperature_sensor.C_in)
    annotation (Line(points={{-230,-180},{-210,-180}}, color={28,108,200}));
  connect(fuel_source_flow_sensor.Q_sensor, fuel_source_flow)
    annotation (Line(points={{-160,-170},{-160,-150}}, color={0,0,127}));
  connect(fuel_source_temperature_sensor.C_out, fuel_source_flow_sensor.C_in)
    annotation (Line(points={{-190,-180},{-170,-180}}, color={28,108,200}));
  connect(fuel_source_flow_sensor.C_out, combustionChamber.inlet1)
    annotation (Line(points={{-150,-180},{0,-180},{0,-20}}, color={28,108,200}));
  connect(combustionChamber_eta, combustionChamber.eta)
    annotation (Line(points={{-30,65},{-8,65},{-8,20}}, color={0,0,127}));
  connect(combustionChamber_Kfr, combustionChamber.Kfr)
    annotation (Line(points={{40,65},{8,65},{8,20}}, color={0,0,127}));
  connect(sink_pressure_sensor.P_sensor, sink_pressure)
    annotation (Line(points={{80,10},{80,30}}, color={0,0,127}));
  connect(combustionChamber.outlet, sink_pressure_sensor.C_in)
    annotation (Line(points={{20,0},{70,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.T_sensor, sink_temperature)
    annotation (Line(points={{130,10},{130,30}}, color={0,0,127}));
  connect(sink_pressure_sensor.C_out, sink_temperature_sensor.C_in)
    annotation (Line(points={{90,0},{120,0}}, color={28,108,200}));
  connect(sink_temperature_sensor.C_out, sink.C_in)
    annotation (Line(points={{140,0},{215,0}}, color={28,108,200}));
  annotation (experiment(StopTime=1,Tolerance=1e-6), Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-360,-300},{400,300}})), Documentation(info="<html><p>Approved MML4 sensor/interface structure: AirCompressor and Metroscopia CCGT/NPP. Illustrative initialization point, not plant calibration. Constant supported faults only. No correction or retry after first Dymola attempt.</p><p></p></html>"));
end CombustionChamber;
