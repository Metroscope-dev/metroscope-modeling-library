within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model AirCooledCondenser_direct

  extends MetroscopeModelingLibrary.Icons.Tests.MultifluidTestIcon;

  //Boundary Conditions
  input Units.MassFlowRate Q_turbine(start=21) "kg/s";
  input Units.SpecificEnthalpy h_turbine(start=2399e3) "J/kg";

  input Units.MassFlowRate Q_cold(start=1800) "kg/s";
  input Real P_cold_source(start=1.002,nominal=1.002) "barA";
  input Real T_cold_source(start=16) "degC";
  input Units.Fraction cold_source_relative_humidity=0.80 "1";

   // Parameters
  parameter Units.Pressure P_offset = 0;
  parameter Real C_incond = 0;
  parameter Units.Area S = 130000 "m2";

  // Calibrated parameters
  parameter Units.HeatExchangeCoefficient Kth=30;
  parameter Units.FrictionCoefficient Kfr_hot=0;

  //Sensor for calibration
  output Real T_cond(start=44) "degC";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_outlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,48})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink condensate_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-66})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source cold_source(h_out(start=36462.457))
    annotation (Placement(transformation(extent={{-68,-10},{-48,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink cold_sink(h_in(start=60317.96))
    annotation (Placement(transformation(extent={{54,-10},{74,10}})));

  MultiFluid.HeatExchangers.AirCooledCondenser airCooledCondenser
    annotation (Placement(transformation(extent={{-16,-16},{16,20}})));
equation

  //Hot source
  turbine_outlet.h_out = h_turbine;
  turbine_outlet.Q_out = -Q_turbine;

  //Cold source
  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out =  T_cold_source+273.15;
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

    // Observable for calibration
  condensate_sink.T_in = T_cond+273.15;

  connect(airCooledCondenser.C_cold_out, cold_sink.C_in)
    annotation (Line(points={{14.4,0},{59,0}}, color={85,170,255}));
  connect(airCooledCondenser.C_hot_in, turbine_outlet.C_out)
    annotation (Line(points={{0.32,18},{0,18},{0,43}}, color={28,108,200}));
  connect(airCooledCondenser.C_hot_out, condensate_sink.C_in)
    annotation (Line(points={{0,-14},{0,-61}}, color={28,108,200}));
  connect(airCooledCondenser.C_cold_in, cold_source.C_out)
    annotation (Line(points={{-14.4,0},{-53,0}}, color={85,170,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end AirCooledCondenser_direct;
