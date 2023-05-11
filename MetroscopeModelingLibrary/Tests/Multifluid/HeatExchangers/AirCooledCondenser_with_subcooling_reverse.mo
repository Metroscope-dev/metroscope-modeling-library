within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model AirCooledCondenser_with_subcooling_reverse

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;

  //Boundary Conditions
  input Utilities.Units.MassFlowRate Q_turbine(start=21) "kg/s";
  input Utilities.Units.SpecificEnthalpy h_turbine(start=2399e3) "J/kg";

  input Utilities.Units.MassFlowRate Q_cold(start=18000) "kg/s";
  input Real P_cold_source(start=1.002,nominal=1.002) "barA";
  input Real T_cold_source(start=10) "degC";
  input Utilities.Units.Fraction cold_source_relative_humidity(start=0.80) "1";

   // Parameters
  parameter Utilities.Units.Pressure P_offset = 0;
  parameter Real C_incond = 0;
  parameter Utilities.Units.Area S_cond = 150000 "m2";
  parameter Utilities.Units.Area S_subc = 15000 "m2";

  // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth_cond(start=10);
  output Utilities.Units.HeatExchangeCoefficient Kth_subc(start=1);
  parameter Utilities.Units.FrictionCoefficient Kfr_hot = 0;

  // Sensors for calibration
  input Real T_subc(start=35) "degC";
  input Real P_cond(start=91) "mbarA";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_outlet(h_out(start=2399000)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,70})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink condensate_sink(h_in(start=146568.66)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-62})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source cold_source(h_out(start=25400))
    annotation (Placement(transformation(extent={{-56,-10},{-36,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink cold_sink(h_in(start=28028.451))    annotation (Placement(transformation(extent={{34,-10},{54,10}})));
  MultiFluid.HeatExchangers.AirCooledCondenser_with_subcooling airCooledCondenser(
    cold_side_condensing(h_out(start=28243.033)),
    cold_side_subcooling(h_out(start=25882.63))) annotation (Placement(transformation(extent={{-16,-16},{16,20}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor
                                                              T_cond_sensor
    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={-1,-37})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cond_sensor
    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={1,37})));
equation

  //Hot source
  turbine_outlet.h_out = h_turbine;
  turbine_outlet.Q_out = - Q_turbine;

  //Cold source
  cold_source.P_out = P_cold_source * 1e5;
  cold_source.T_out = T_cold_source + 273.15;
  cold_source.relative_humidity = cold_source_relative_humidity;

  //ACC
    // Parameters
  airCooledCondenser.S_cond = S_cond;
  airCooledCondenser.S_subc = S_subc;
  airCooledCondenser.Q_cold = Q_cold;
  airCooledCondenser.P_offset = P_offset;
  airCooledCondenser.C_incond = C_incond;
    // Calibrater parameter
  airCooledCondenser.Kth_cond = Kth_cond;
  airCooledCondenser.Kth_subc = Kth_subc;
  airCooledCondenser.Kfr_hot = Kfr_hot;
    // Observable for calibration
  T_cond_sensor.T_degC = T_subc;
  P_cond_sensor.P_mbar = P_cond;


  connect(airCooledCondenser.C_cold_out, cold_sink.C_in)
    annotation (Line(points={{14.4,0},{39,0}}, color={85,170,255}));
  connect(airCooledCondenser.C_cold_in, cold_source.C_out)
    annotation (Line(points={{-14.08,0},{-41,0}},color={85,170,255}));
  connect(airCooledCondenser.C_hot_out, T_cond_sensor.C_in)
    annotation (Line(points={{0,-14},{0,-30},{-1,-30}}, color={28,108,200}));
  connect(condensate_sink.C_in, T_cond_sensor.C_out) annotation (Line(points={{8.88178e-16,
          -57},{0,-57},{0,-44},{-1,-44}}, color={28,108,200}));
  connect(turbine_outlet.C_out, P_cond_sensor.C_in) annotation (Line(points={{-8.88178e-16,
          65},{0,65},{0,44},{1,44}}, color={28,108,200}));
  connect(airCooledCondenser.C_hot_in, P_cond_sensor.C_out) annotation (Line(
        points={{0.32,18},{0,18},{0,30},{1,30}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end AirCooledCondenser_with_subcooling_reverse;
