within MetroscopeModelingLibrary.Tests.SteelworksGases.HeatExchangers;
model SteelworksGasesHeater_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.SteelworksGasesTestIcon;
  import Interfaces = MetroscopeModelingLibrary.Utilities.Interfaces;
  import Medium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium;
  import MetroscopeModelingLibrary.Utilities.Units;

  MetroscopeModelingLibrary.SteelworksGases.BoundaryConditions.Source cold_source
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-150,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.PressureSensor P_cold_source_sensor(
    sensor_function="BC",
    P_start=20,
    P_0=2e+06,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-115,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.FlowSensor Q_cold_source_sensor(
    sensor_function="BC",
    Q_start=100,
    Q_0=100,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-81,0})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={20,154})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor P_hot_source_sensor(
    sensor_function="BC",
    P_start=50,
    P_0=5e+06,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={20,121})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.FlowSensor Q_hot_source_sensor(
    sensor_function="BC",
    Q_start=50,
    Q_0=50,
    signal_unit="kg/s",
    display_unit="kg/s")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={20,86})));
  MetroscopeModelingLibrary.MultiFluid.HeatExchangers.SteelworksGasesHeater heater(
    QCp_max_side=QCp_max_side,
    S=S)
    annotation (Placement(transformation(extent={{-50,-50},{50,50}}, origin={0,0})));
  MetroscopeModelingLibrary.SensorsControl.SteelworksGases.PressureSensor P_cold_out_sensor(
    sensor_function="Calibration",
    causality="Kfr_cold",
    P_start=19,
    P_0=1.9e+06,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={81,0})));
  MetroscopeModelingLibrary.SteelworksGases.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={115,0})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.TemperatureSensor T_hot_out_sensor(
    sensor_function="Calibration",
    causality="Kth",
    T_start=125,
    T_0=398.15,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={-20,-72})));
  MetroscopeModelingLibrary.SensorsControl.WaterSteam.PressureSensor P_hot_out_sensor(
    sensor_function="Calibration",
    causality="Kfr_hot",
    P_start=49,
    P_0=4.9e+06,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={-20,-108})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={-20,-143})));

  parameter Units.Area S = 100;

  parameter String QCp_max_side = "hot";

  input Units.SpecificEnthalpy hot_source_h(start=0.7e6) "J/kg";

  input Units.SpecificEnthalpy h_cold_source(start=Medium.specificEnthalpy(
    Medium.setState_pTX(20e5, 300, Medium.reference_X))) "J/kg";

  Interfaces.BoundaryCondition P_cold_source(start=P_cold_source_sensor.P_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-115,27})));

  Interfaces.BoundaryCondition Q_cold_source(start=Q_cold_source_sensor.Q_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={-81,27})));

  Interfaces.BoundaryCondition P_hot_source(start=P_hot_source_sensor.P_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=180, origin={47,121})));

  Interfaces.BoundaryCondition Q_hot_source(start=Q_hot_source_sensor.Q_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=180, origin={47,86})));

  Interfaces.CalibrationInput P_cold_out(start=P_cold_out_sensor.P_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=270, origin={81,27})));

  Interfaces.CalibrationInput T_hot_out(start=T_hot_out_sensor.T_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=180, origin={7,-72})));

  Interfaces.CalibrationInput P_hot_out(start=P_hot_out_sensor.P_start)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=180, origin={7,-108})));

  Interfaces.GenericReal Kth(start=1148.7983356)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-40,50})));

  Interfaces.GenericReal Kfr_hot(start=36202.2046603)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={59,30})));

  Interfaces.GenericReal Kfr_cold(start=243.852865787)
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-70,-20})));
equation
  hot_source.h_out = hot_source_h;
  cold_source.h_out = h_cold_source;
  cold_source.Xi_out = Medium.reference_X;

  connect(cold_source.C_out, P_cold_source_sensor.C_in)
    annotation (Line(points={{-145,0},{-125,0}}, color={213,170,255}));
  connect(P_cold_source_sensor.C_out, Q_cold_source_sensor.C_in)
    annotation (Line(points={{-105,0},{-91,0}}, color={213,170,255}));
  connect(Q_cold_source_sensor.C_out, heater.C_cold_in)
    annotation (Line(points={{-71,0},{-50,0}}, color={213,170,255}));
  connect(heater.C_cold_out, P_cold_out_sensor.C_in)
    annotation (Line(points={{50,0},{71,0}}, color={213,170,255}));
  connect(P_cold_out_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{91,0},{110,0}}, color={213,170,255}));
  connect(hot_source.C_out, P_hot_source_sensor.C_in)
    annotation (Line(points={{20,149},{20,131}}, color={28,108,200}));
  connect(P_hot_source_sensor.C_out, Q_hot_source_sensor.C_in)
    annotation (Line(points={{20,111},{20,96}}, color={28,108,200}));
  connect(Q_hot_source_sensor.C_out, heater.C_hot_in)
    annotation (Line(points={{20,76},{20,40}}, color={28,108,200}));
  connect(heater.C_hot_out, T_hot_out_sensor.C_in)
    annotation (Line(points={{-20,-40},{-20,-62}}, color={28,108,200}));
  connect(T_hot_out_sensor.C_out, P_hot_out_sensor.C_in)
    annotation (Line(points={{-20,-82},{-20,-98}}, color={28,108,200}));
  connect(P_hot_out_sensor.C_out, hot_sink.C_in)
    annotation (Line(points={{-20,-118},{-20,-138}}, color={28,108,200}));

  connect(P_cold_source_sensor.P_sensor, P_cold_source)
    annotation (Line(points={{-115,10},{-115,27}}, color={0,0,127}));
  connect(Q_cold_source_sensor.Q_sensor, Q_cold_source)
    annotation (Line(points={{-81,10},{-81,27}}, color={0,0,127}));
  connect(P_hot_source_sensor.P_sensor, P_hot_source)
    annotation (Line(points={{30,121},{47,121}}, color={0,0,127}));
  connect(Q_hot_source_sensor.Q_sensor, Q_hot_source)
    annotation (Line(points={{30,86},{47,86}}, color={0,0,127}));
  connect(P_cold_out_sensor.P_sensor, P_cold_out)
    annotation (Line(points={{81,10},{81,27}}, color={0,0,127}));
  connect(T_hot_out_sensor.T_sensor, T_hot_out)
    annotation (Line(points={{-10,-72},{7,-72}}, color={0,0,127}));
  connect(P_hot_out_sensor.P_sensor, P_hot_out)
    annotation (Line(points={{-10,-108},{7,-108}}, color={0,0,127}));
  connect(Kth, heater.Kth)
    annotation (Line(points={{-40,50},{-40,30}}, color={0,0,127}));
  connect(Kfr_hot, heater.Kfr_hot)
    annotation (Line(points={{59,30},{40,30}}, color={0,0,127}));
  connect(Kfr_cold, heater.Kfr_cold)
    annotation (Line(points={{-70,-20},{-50,-20}}, color={0,0,127}));

  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-174,-163},{140,172}})));
end SteelworksGasesHeater_reverse;
