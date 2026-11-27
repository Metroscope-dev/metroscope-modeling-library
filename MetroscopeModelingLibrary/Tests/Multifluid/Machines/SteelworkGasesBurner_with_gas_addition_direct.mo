within MetroscopeModelingLibrary.Tests.Multifluid.Machines;
model SteelworkGasesBurner_with_gas_addition_direct
  "MML4 direct test with increasing methane content"
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  import Interfaces = MetroscopeModelingLibrary.Utilities.Interfaces;
  import Medium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  import AirMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source air_source
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-190,0})));

  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor air_P_sensor(
    sensor_function="BC",
    P_start=17,
    P_0=120000,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-154,0})));

  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor air_T_sensor(
    sensor_function="BC",
    T_start=406,
    T_0=350,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-120,0})));

  MetroscopeModelingLibrary.SensorsControl.FlueGases.FlowSensor air_Q_sensor(
    sensor_function="BC",
    Q_start=50,
    Q_0=20,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-86,0})));

  MetroscopeModelingLibrary.SteelworksGases.BoundaryConditions.Chromatograph fuel_source(signal_unit="%mol")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-190,-105})));

  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.PressureSensor fuel_P_sensor(
    sensor_function="BC",
    P_start=1.3,
    P_0=130000,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-154,-105})));

  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.TemperatureSensor fuel_T_sensor(
    sensor_function="BC",
    T_start=30,
    T_0=303.15,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-120,-105})));

  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.FlowSensor fuel_Q_sensor(
    sensor_function="BC",
    Q_start=10,
    Q_0=10,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-86,-105})));

  MetroscopeModelingLibrary.MultiFluid.Machines.SteelworkGasesBurner burner(
      useAddedFuel=true,
    h_in_air_0=AirMedium.h_TX(350, AirMedium.reference_X))
    annotation (Placement(transformation(extent={{-20,-20},{20,20}}, origin={0,0})));

  MetroscopeModelingLibrary.SensorsControl.FlueGases.PressureSensor exhaust_P_sensor(
    sensor_function="Calibration",
    causality="burner_Kfr",
    P_start=1.195,
    P_0=119500,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={49,0})));

  MetroscopeModelingLibrary.SensorsControl.FlueGases.TemperatureSensor exhaust_T_sensor(
    sensor_function="Calibration",
    causality="burner_eta",
    T_start=780,
    T_0=1053.15,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={83,0})));

  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={120,0})));

  Modelica.Blocks.Sources.Ramp methane_ramp(
    height=methane_increase,
    duration=10,
    offset=reference_mole_fractions[2],
    startTime=5)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-250,-105})));

  final parameter Modelica.Units.SI.MoleFraction reference_mole_fractions[Medium.nX] = Medium.referenceMolePercent/sum(Medium.referenceMolePercent);
  parameter Modelica.Units.SI.MoleFraction methane_increase( min=0, max=reference_mole_fractions[1]) = 0.01
    "Additional methane mole fraction replacing nitrogen";

  Interfaces.BoundaryCondition air_P(start=air_P_sensor.P_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-154,27})));

  Interfaces.BoundaryCondition air_T(start=air_T_sensor.T_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-120,27})));

  Interfaces.BoundaryCondition air_Q(start=air_Q_sensor.Q_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-86,27})));

  Interfaces.BoundaryCondition fuel_P(start=fuel_P_sensor.P_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-154,-78})));

  Interfaces.BoundaryCondition fuel_T(start=fuel_T_sensor.T_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-120,-78})));

  Interfaces.BoundaryCondition fuel_Q(start=fuel_Q_sensor.Q_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-86,-78})));

  Interfaces.Observable exhaust_P
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={49,27})));

  Interfaces.Observable exhaust_T
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90, origin={83,27})));

  Interfaces.Observable methane_mole_fraction "CH4 mole fraction supplied to the burner"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-222,-105})));

  Interfaces.GenericReal burner_eta
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-8,46})));

  Interfaces.GenericReal burner_Kfr
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={8,46})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Chromatograph chromatograph
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={130,-98})));
  SensorsControl.Fuel.FlowSensor flowSensor(Q_start=1, signal_unit="kg/s")
    annotation (Placement(transformation(extent={{44,-108},{24,-88}})));
  SensorsControl.Fuel.PressureSensor pressureSensor(P_start=20, signal_unit=
        "barA")
    annotation (Placement(transformation(extent={{80,-108},{60,-88}})));
  SensorsControl.Fuel.TemperatureSensor temperatureSensor(T_start=30,
      signal_unit="degC")
    annotation (Placement(transformation(extent={{108,-108},{88,-88}})));
  Interfaces.GasCompositionInput composition annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=180,
        origin={170,-98})));
  Interfaces.BoundaryCondition gas_Q annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={34,-78})));
  Interfaces.BoundaryCondition gas_P annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={70,-78})));
  Interfaces.BoundaryCondition gas_T annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={98,-74})));
equation
  air_source.Xi_out = AirMedium.reference_X;
  // Replace nitrogen with methane on a molar basis.
  fuel_source.composition.N2 = reference_mole_fractions[1] - (methane_mole_fraction - reference_mole_fractions[2]);
  fuel_source.composition.CH4 = methane_mole_fraction;
  fuel_source.composition.CO2 = reference_mole_fractions[3];
  fuel_source.composition.H2 = reference_mole_fractions[4];
  fuel_source.composition.CO = reference_mole_fractions[5];
  fuel_source.composition.O2 = reference_mole_fractions[6];
  fuel_source.composition.H2O = reference_mole_fractions[7];
  burner_eta = 0.895521528491;
  burner_Kfr = 0.234268576507;

  connect(air_source.C_out, air_P_sensor.C_in)
    annotation (Line(points={{-185,0},{-164,0}}, color={95,95,95}));
  connect(air_P_sensor.C_out, air_T_sensor.C_in)
    annotation (Line(points={{-144,0},{-130,0}}, color={95,95,95}));
  connect(air_T_sensor.C_out, air_Q_sensor.C_in)
    annotation (Line(points={{-110,0},{-96,0}}, color={95,95,95}));
  connect(air_Q_sensor.C_out, burner.inlet)
    annotation (Line(points={{-76,0},{-20,0}}, color={95,95,95}));
  connect(fuel_source.C_out, fuel_P_sensor.C_in)
    annotation (Line(points={{-185,-105},{-164,-105}}, color={213,170,255}));
  connect(fuel_P_sensor.C_out, fuel_T_sensor.C_in)
    annotation (Line(points={{-144,-105},{-130,-105}}, color={213,170,255}));
  connect(fuel_T_sensor.C_out, fuel_Q_sensor.C_in)
    annotation (Line(points={{-110,-105},{-96,-105}}, color={213,170,255}));
  connect(fuel_Q_sensor.C_out, burner.inlet1)
    annotation (Line(points={{-76,-105},{-8,-105},{-8,-20}},
                                                           color={213,170,255}));
  connect(burner.outlet, exhaust_P_sensor.C_in)
    annotation (Line(points={{20,0},{39,0}}, color={95,95,95}));
  connect(exhaust_P_sensor.C_out, exhaust_T_sensor.C_in)
    annotation (Line(points={{59,0},{73,0}}, color={95,95,95}));
  connect(exhaust_T_sensor.C_out, sink.C_in)
    annotation (Line(points={{93,0},{115,0}}, color={95,95,95}));

  connect(air_P_sensor.P_sensor, air_P)
    annotation (Line(points={{-154,10},{-154,27}}, color={0,0,127}));
  connect(air_T_sensor.T_sensor, air_T)
    annotation (Line(points={{-120,10},{-120,27}}, color={0,0,127}));
  connect(air_Q_sensor.Q_sensor, air_Q)
    annotation (Line(points={{-86,10},{-86,27}}, color={0,0,127}));
  connect(fuel_P_sensor.P_sensor, fuel_P)
    annotation (Line(points={{-154,-95},{-154,-78}}, color={0,0,127}));
  connect(fuel_T_sensor.T_sensor, fuel_T)
    annotation (Line(points={{-120,-95},{-120,-78}}, color={0,0,127}));
  connect(fuel_Q_sensor.Q_sensor, fuel_Q)
    annotation (Line(points={{-86,-95},{-86,-78}}, color={0,0,127}));
  connect(exhaust_P_sensor.P_sensor, exhaust_P)
    annotation (Line(points={{49,10},{49,27}}, color={0,0,127}));
  connect(exhaust_T_sensor.T_sensor, exhaust_T)
    annotation (Line(points={{83,10},{83,27}}, color={0,0,127}));
  connect(burner_eta, burner.eta)
    annotation (Line(points={{-8,46},{-8,20}}, color={0,0,127}));
  connect(burner_Kfr, burner.Kfr)
    annotation (Line(points={{8,46},{8,20}}, color={0,0,127}));
  connect(methane_ramp.y, methane_mole_fraction)
    annotation (Line(points={{-239,-105},{-222,-105}}, color={0,0,127}));

  connect(flowSensor.C_out, burner.fuel_in)
    annotation (Line(points={{24,-98},{10,-98},{10,-20}}, color={213,213,0}));
  connect(chromatograph.composition, composition) annotation (Line(
      points={{145.2,-98},{170,-98}},
      color={211,211,0},
      thickness=0.5));
  connect(temperatureSensor.C_in, chromatograph.C_out)
    annotation (Line(points={{108,-98},{120,-98}}, color={213,213,0}));
  connect(temperatureSensor.C_out, pressureSensor.C_in)
    annotation (Line(points={{88,-98},{80,-98}}, color={213,213,0}));
  connect(pressureSensor.C_out, flowSensor.C_in)
    annotation (Line(points={{60,-98},{44,-98}}, color={213,213,0}));
  connect(flowSensor.Q_sensor, gas_Q)
    annotation (Line(points={{34,-88},{34,-78}}, color={0,0,127}));
  connect(gas_P, pressureSensor.P_sensor)
    annotation (Line(points={{70,-78},{70,-88}}, color={28,108,200}));
  connect(gas_T, temperatureSensor.T_sensor)
    annotation (Line(points={{98,-74},{98,-88}}, color={28,108,200}));
  annotation (experiment(StopTime=20, Tolerance=1e-6),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-275,-132},{
            260,65}})),
    Documentation(info="<html><p>Fixed burner characteristics are identified from the reverse test before the methane ramp starts.</p>
    <p>Each point is steady state. The methane mole fraction increases from its reference value between 5 s and 15 s as nitrogen decreases. Other fuel mole fractions remain fixed.</p></html>"),
    Icon(coordinateSystem(extent={{-275,-132},{260,65}})));
end SteelworkGasesBurner_with_gas_addition_direct;
