within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model Condenser_reverse

  extends Modelica.Icons.Example;

    // Boundary conditions
  input Units.MassFlowRate Q_turbine(start=150) "kg/s";
  input Units.SpecificEnthalpy h_turbine(start=1500e3);
  input Real P_cold_source(start=5, min=0, nominal=10) "barA";
  input Real T_cold_source(start = 15, min = 0, nominal = 50) "degC";

    // Parameters
  parameter Units.Area S = 100;
  parameter Units.Height water_height = 2;
  parameter Units.Pressure C_incond = 0;
  parameter Units.Pressure P_offset = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 1;
  parameter Units.VolumeFlowRate Qv_cold = 3.82;

    // Calibrated parameters
  output Units.HeatExchangeCoefficient Kth;

    // Inputs for calibration
  input Real P_cond(start = 0.19e5) "Pa";


  WaterSteam.BoundaryConditions.WaterSource cooling_source
    annotation (Placement(transformation(extent={{-58,-6},{-38,14}})));
  WaterSteam.BoundaryConditions.WaterSink cooling_sink
    annotation (Placement(transformation(extent={{62,-12},{82,8}})));
  WaterSteam.HeatExchangers.Condenser condenser
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  WaterSteam.BoundaryConditions.WaterSource turbine_outlet annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,46})));
  WaterSteam.BoundaryConditions.WaterSink condensate_sink annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-44})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor P_cond_sensor annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=270,
        origin={1,25})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor circulating_water_T_out_sensor annotation (Placement(transformation(extent={{24,-6},{34,4}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterTemperatureSensor condensate_temperature_sensor
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=270,
        origin={-1,-21})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.WaterPressureSensor circulating_water_P_out_sensor annotation (Placement(transformation(extent={{40,-6},{50,4}})));
equation

  // Boundary Conditions
  turbine_outlet.h_out = h_turbine;
  turbine_outlet.Q_out = -Q_turbine;

  cooling_source.P_out = P_cold_source*1e5;
  cooling_source.T_out = 273.15 + T_cold_source;

  // Parameters
  condenser.S = S;
  condenser.water_height = water_height;
  condenser.C_incond = C_incond;
  condenser.P_offset = P_offset;
  condenser.Kfr_cold = Kfr_cold;
  condenser.Qv_cold_in = Qv_cold;

  // Calibrated Parameters
  condenser.Kth = Kth;

  // Inputs for calibration
  P_cond_sensor.P = P_cond;



  connect(cooling_source.C_out, condenser.C_cold_in) annotation (Line(points={{-43,4},
          {-30,4},{-30,3.55556},{-16.64,3.55556}},    color={28,108,200}));
  connect(condenser.C_hot_in, P_cond_sensor.C_out)
    annotation (Line(points={{0,8},{0,20},{1,20}}, color={28,108,200}));
  connect(turbine_outlet.C_out, P_cond_sensor.C_in) annotation (Line(points={{-8.88178e-16,
          41},{0,41},{0,30},{1,30}}, color={28,108,200}));
  connect(condenser.C_cold_out, circulating_water_T_out_sensor.C_in)
    annotation (Line(points={{16,-1.42222},{16,-1},{24,-1}}, color={28,108,200}));
  connect(condensate_temperature_sensor.C_in, condenser.C_hot_out) annotation (
      Line(points={{-1,-16},{0,-16},{0,-8.35556}}, color={28,108,200}));
  connect(condensate_temperature_sensor.C_out, condensate_sink.C_in)
    annotation (Line(points={{-1,-26},{-1,-32.5},{8.88178e-16,-32.5},{8.88178e-16,
          -39}}, color={28,108,200}));
  connect(circulating_water_P_out_sensor.C_in, circulating_water_T_out_sensor.C_out)
    annotation (Line(points={{40,-1},{34,-1}}, color={28,108,200}));
  connect(cooling_sink.C_in, circulating_water_P_out_sensor.C_out) annotation (
      Line(points={{67,-2},{58,-2},{58,-1},{50,-1}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,80}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end Condenser_reverse;
