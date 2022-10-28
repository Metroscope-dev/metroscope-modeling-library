within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model airCooledCondenser_reverse

  extends MetroscopeModelingLibrary.Icons.Tests.MultifluidTestIcon;

  //Boundary Conditions
  input Units.MassFlowRate Q_turbine(start=21.03) "kg/s";
  input Units.SpecificEnthalpy h_turbine(start=2399e3) "J/kg";

  input Units.MassFlowRate Q_cold(start=437*5) "kg/s";
  input Real P_cold_source(start=1.002,nominal=1.002) "barA";
  input Real T_cold_source(start=16) "degC";
  input Units.Fraction cold_source_relative_humidity=0.80 "1";

   // Parameters
  parameter Units.Pressure P_offset = 0;
  parameter Real C_incond = 0;
  parameter Units.Area S = 129500 "m2";

  // Calibrated parameters
  output Units.HeatExchangeCoefficient Kth;
  parameter Units.FrictionCoefficient Kfr_hot=0;

  //Input for calibration
  input Real T_cond(start=41.5) "degC";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_outlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,50})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink condensate_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-70})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source cold_source(h_out(start=36462.457))
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink cold_sink
    annotation (Placement(transformation(extent={{50,-10},{70,10}})));
  MultiFluid.HeatExchangers.airCooledCondenser airCooledCondenser
    annotation (Placement(transformation(extent={{-16,-16},{16,20}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor
    coolingAir_in_P_sensor
    annotation (Placement(transformation(extent={{-75,-5},{-65,5}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.TemperatureSensor
    coolingAir_in_T_sensor
    annotation (Placement(transformation(extent={{-55,-5},{-45,5}})));
  MetroscopeModelingLibrary.Sensors.MoistAir.FlowSensor coolinAir_Q_in_sensor
    annotation (Placement(transformation(extent={{-35,-5},{-25,5}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cond_sensor
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={0,-30})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.FlowSensor Q_cond_sensor
    annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={0,-40})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_cond_sensor
    annotation (Placement(transformation(
        extent={{-3.5,-3.5},{3.5,3.5}},
        rotation=270,
        origin={0,-50})));
equation

  //Hot source

  turbine_outlet.h_out = h_turbine;
  Q_cond_sensor.Q = Q_turbine;

  //Cold source
  coolingAir_in_P_sensor.P_barA = P_cold_source;

 //cold_source.h_out=h_cold_source;
  coolingAir_in_T_sensor.T_degC =  T_cold_source;
  cold_source.relative_humidity = cold_source_relative_humidity;
  //ACC
    // Parameters
  airCooledCondenser.S = S;
  airCooledCondenser.Q_cold = Q_cold;
  airCooledCondenser.P_offset = P_offset;
  airCooledCondenser.C_incond = C_incond;
    // Calibrater parameter
  airCooledCondenser.Kth = Kth;
  airCooledCondenser.Kfr_hot = Kfr_hot;

    // Inputs for calibration
    T_cond_sensor.T_degC = T_cond;

  connect(cold_source.C_out, coolingAir_in_P_sensor.C_in) annotation (Line(
        points={{-85,0},{-75,0}},                       color={85,170,255}));
  connect(coolingAir_in_P_sensor.C_out,coolingAir_in_T_sensor. C_in)
    annotation (Line(points={{-65,0},{-55,0}},   color={85,170,255}));
  connect(coolingAir_in_T_sensor.C_out, coolinAir_Q_in_sensor.C_in)
    annotation (Line(points={{-45,0},{-35,0}},   color={85,170,255}));
  connect(airCooledCondenser.C_cold_in, coolinAir_Q_in_sensor.C_out)
    annotation (Line(points={{-14.4,0},{-25,0}}, color={85,170,255}));
  connect(airCooledCondenser.C_hot_out, P_cond_sensor.C_in) annotation (Line(
        points={{-4.44089e-16,-14},{-4.44089e-16,-21},{4.44089e-16,-21},{4.44089e-16,
          -26}}, color={28,108,200}));
  connect(Q_cond_sensor.C_in, P_cond_sensor.C_out) annotation (Line(points={{7.77156e-16,
          -36},{7.77156e-16,-34},{-7.77156e-16,-34}},                    color={
          28,108,200}));
  connect(Q_cond_sensor.C_out, T_cond_sensor.C_in) annotation (Line(points={{-7.21645e-16,
          -44},{-7.21645e-16,-46.5},{1.08247e-15,-46.5}},
                                           color={28,108,200}));
  connect(condensate_sink.C_in, T_cond_sensor.C_out) annotation (Line(points={{9.4369e-16,
          -65},{9.4369e-16,-53.5},{-2.22045e-16,-53.5}},
                                           color={28,108,200}));
  connect(airCooledCondenser.C_hot_in, turbine_outlet.C_out)
    annotation (Line(points={{0.32,18},{0,18},{0,45}}, color={28,108,200}));
  connect(airCooledCondenser.C_cold_out, cold_sink.C_in)
    annotation (Line(points={{14.4,0},{55,0}}, color={85,170,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},
            {80,60}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-80},{80,60}})));
end airCooledCondenser_reverse;
