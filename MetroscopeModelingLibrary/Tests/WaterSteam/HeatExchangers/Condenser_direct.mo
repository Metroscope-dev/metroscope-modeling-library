within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Condenser_direct

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

    // Boundary conditions
  input Utilities.Units.MassFlowRate Q_turbine(start=150) "kg/s";
  input Utilities.Units.SpecificEnthalpy h_turbine(start=1500e3);

    // Specifications, if "input_specs" is set to true
  input Real hotwell_level( start = 1);

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_outlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,78})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink condensate_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-58})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cooling_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-86,0})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cooling_sink annotation (Placement(transformation(extent={{74,-10},{94,10}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser( input_specs = true)
    annotation (Placement(transformation(extent={{-10,-8},{10,10}})));
  Sensors_Control.WaterSteam.TemperatureSensor Circ_Water_Outlet_Temp_sensor(
    sensor_function="Calibration",
    causality="Qv cold",
    T_start=28.4,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{22,-10},{42,10}})));
  Sensors_Control.WaterSteam.PressureSensor Circ_Water_Outlet_Press_sensor(
    sensor_function="Calibration",
    causality="Kfr cold",
    P_start=1,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{52,-10},{72,10}})));
  Sensors_Control.WaterSteam.PressureSensor Cond_Pressure_sensor(
    sensor_function="Calibration",
    causality="Kth",
    P_start=0.19,
    signal_unit="barA",
    display_unit="barA") annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,48})));
  Sensors_Control.WaterSteam.TemperatureSensor Circ_Water_Inlet_Temp_sensor(
    sensor_function="BC",
    T_start=15,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  Sensors_Control.WaterSteam.PressureSensor Circ_Water_Inlet_Press_sensor(
    sensor_function="BC",
    P_start=1.1,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Utilities.Interfaces.BoundaryCondition Circ_Water_Inlet_Temp annotation (
      Placement(transformation(extent={{-64,18},{-56,26}}), iconTransformation(
          extent={{-268,-24},{-228,16}})));
  Utilities.Interfaces.BoundaryCondition Circ_Water_Inlet_Press
                                                               annotation (
      Placement(transformation(extent={{-34,18},{-26,26}}), iconTransformation(
          extent={{-268,-24},{-228,16}})));
  Utilities.Interfaces.RealOutput Kth annotation (Placement(transformation(
          extent={{-10,18},{-2,26}}), iconTransformation(extent={{-134,-16},{-114,
            6}})));
  Utilities.Interfaces.RealOutput Qv_cold_in annotation (Placement(
        transformation(extent={{-24,28},{-16,36}}), iconTransformation(extent={{
            -134,-16},{-114,6}})));
  Utilities.Interfaces.RealOutput Kfr_cold annotation (Placement(transformation(
          extent={{-26,-28},{-18,-20}}), iconTransformation(extent={{-134,-16},{
            -114,6}})));
  Sensors_Control.WaterSteam.TemperatureSensor Condensate_Temp_sensor
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-32})));
  Utilities.Interfaces.RealOutput Condensate_Temp annotation (Placement(
        transformation(extent={{26,-36},{34,-28}}), iconTransformation(extent={{
            -194,-30},{-174,-10}})));
  Utilities.Interfaces.RealExpression C_incond(y=0)
    annotation (Placement(transformation(extent={{-2,16},{18,36}})));
  Utilities.Interfaces.RealOutput Cond_Pressure annotation (Placement(
        transformation(extent={{18,44},{26,52}}), iconTransformation(extent={{-194,
            -30},{-174,-10}})));
  Utilities.Interfaces.RealOutput Circ_Water_Outlet_Temp annotation (Placement(
        transformation(extent={{26,20},{34,28}}), iconTransformation(extent={{-194,
            -30},{-174,-10}})));
  Utilities.Interfaces.RealOutput Circ_Water_Outlet_Press annotation (Placement(
        transformation(extent={{54,26},{62,34}}), iconTransformation(extent={{-194,
            -30},{-174,-10}})));
equation

  // Boundary Conditions
  turbine_outlet.h_out = h_turbine;
  turbine_outlet.Q_out = -Q_turbine;

  // Calibrated parameters
  Qv_cold_in = 3.4;
  Kfr_cold = 0.89;
  Kth = 102;

  // Specifications, if "input_specs" is set to true
  condenser.water_height = hotwell_level;
  condenser.S = 50000;

  connect(condenser.C_cold_out, Circ_Water_Outlet_Temp_sensor.C_in)
    annotation (Line(points={{9.8,0},{22,0}}, color={28,108,200}));
  connect(Circ_Water_Outlet_Temp_sensor.C_out, Circ_Water_Outlet_Press_sensor.C_in)
    annotation (Line(points={{42,0},{52,0}}, color={28,108,200}));
  connect(cooling_sink.C_in, Circ_Water_Outlet_Press_sensor.C_out)
    annotation (Line(points={{79,0},{72,0}}, color={28,108,200}));
  connect(Cond_Pressure_sensor.C_in, turbine_outlet.C_out)
    annotation (Line(points={{0,58},{0,73}}, color={28,108,200}));
  connect(Cond_Pressure_sensor.C_out, condenser.C_hot_in)
    annotation (Line(points={{0,38},{0,10.2}}, color={28,108,200}));
  connect(Circ_Water_Inlet_Temp_sensor.C_in, cooling_source.C_out)
    annotation (Line(points={{-70,0},{-81,0}}, color={28,108,200}));
  connect(Circ_Water_Inlet_Press_sensor.C_in, Circ_Water_Inlet_Temp_sensor.C_out)
    annotation (Line(points={{-40,0},{-50,0}}, color={28,108,200}));
  connect(Circ_Water_Inlet_Press_sensor.C_out, condenser.C_cold_in)
    annotation (Line(points={{-20,0},{-10,0}}, color={28,108,200}));
  connect(Circ_Water_Inlet_Temp_sensor.T_sensor, Circ_Water_Inlet_Temp)
    annotation (Line(points={{-60,10},{-60,22}}, color={0,0,127}));
  connect(Circ_Water_Inlet_Press_sensor.P_sensor, Circ_Water_Inlet_Press)
    annotation (Line(points={{-30,10},{-30,22}}, color={0,0,127}));
  connect(condenser.Kth, Kth)
    annotation (Line(points={{-6.4,11},{-6,11},{-6,22}}, color={0,0,127}));
  connect(condenser.Qv_cold_in, Qv_cold_in) annotation (Line(points={{-11,8},{-16,
          8},{-16,24},{-20,24},{-20,32}}, color={0,0,127}));
  connect(Kfr_cold, condenser.Kfr_cold) annotation (Line(points={{-22,-24},{-14,
          -24},{-14,4},{-11,4}}, color={0,0,127}));
  connect(condenser.C_hot_out, Condensate_Temp_sensor.C_in)
    annotation (Line(points={{0,-8},{0,-22}}, color={28,108,200}));
  connect(condensate_sink.C_in, Condensate_Temp_sensor.C_out)
    annotation (Line(points={{0,-53},{0,-42}}, color={28,108,200}));
  connect(Condensate_Temp_sensor.T_sensor, Condensate_Temp)
    annotation (Line(points={{10,-32},{30,-32}}, color={0,0,127}));
  connect(condenser.C_incond, C_incond.y)
    annotation (Line(points={{6.4,11},{8,12},{8,21}}, color={0,0,127}));
  connect(Cond_Pressure_sensor.P_sensor, Cond_Pressure)
    annotation (Line(points={{10,48},{22,48}}, color={0,0,127}));
  connect(Circ_Water_Outlet_Temp_sensor.T_sensor, Circ_Water_Outlet_Temp)
    annotation (Line(points={{32,10},{30,10},{30,24}}, color={0,0,127}));
  connect(Circ_Water_Outlet_Press_sensor.P_sensor, Circ_Water_Outlet_Press)
    annotation (Line(points={{62,10},{62,30},{58,30}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,80}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end Condenser_direct;
