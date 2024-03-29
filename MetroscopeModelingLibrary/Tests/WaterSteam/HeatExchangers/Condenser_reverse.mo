within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Condenser_reverse

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

    // Boundary conditions
  input Utilities.Units.MassFlowRate Q_turbine(start=150) "kg/s";
  input Utilities.Units.SpecificEnthalpy h_turbine(start=1500e3);
  input Real P_cold_source(start=5, min=0, nominal=10) "barA";
  input Real T_cold_source(start = 15, min = 0, nominal = 50) "degC";

    // Parameters
  parameter Utilities.Units.Area S=100;
  parameter Utilities.Units.Height water_height=1;
  parameter Real C_incond = 0;
  parameter Utilities.Units.Pressure P_offset=0;
  parameter Utilities.Units.FrictionCoefficient Kfr_cold=1;
  parameter Utilities.Units.VolumeFlowRate Qv_cold=3.82;

    // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth;

    // Inputs for calibration
  input Real P_cond(start = 0.19e5) "Pa";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_outlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,78})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink condensate_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-44})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cooling_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-44,0})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cooling_sink annotation (Placement(transformation(extent={{74,-10},{94,10}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser annotation (Placement(transformation(extent={{-10,-8},{10,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor condensate_temperature_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-24})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor circulating_water_T_out_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={30,0})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor circulating_water_P_out_sensor annotation (Placement(transformation(extent={{50,-10},{70,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cond_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,40})));
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

  connect(condenser.C_cold_in, cooling_source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={28,108,200}));
  connect(condenser.C_hot_out, condensate_temperature_sensor.C_in) annotation (Line(points={{0,-8},{0,-11},{1.77636e-15,-11},{1.77636e-15,-14}}, color={28,108,200}));
  connect(condensate_sink.C_in, condensate_temperature_sensor.C_out) annotation (Line(points={{8.88178e-16,-39},{8.88178e-16,-36.5},{-1.77636e-15,-36.5},{-1.77636e-15,-34}}, color={28,108,200}));
  connect(condenser.C_cold_out, circulating_water_T_out_sensor.C_in) annotation (Line(points={{9.8,0},{20,0}}, color={28,108,200}));
  connect(circulating_water_T_out_sensor.C_out, circulating_water_P_out_sensor.C_in) annotation (Line(points={{40,0},{50,0}}, color={28,108,200}));
  connect(circulating_water_P_out_sensor.C_out, cooling_sink.C_in) annotation (Line(points={{70,0},{79,0}}, color={28,108,200}));
  connect(turbine_outlet.C_out, P_cond_sensor.C_in) annotation (Line(points={{-8.88178e-16,73},{-8.88178e-16,61.5},{1.77636e-15,61.5},{1.77636e-15,50}}, color={28,108,200}));
  connect(condenser.C_hot_in, P_cond_sensor.C_out) annotation (Line(points={{0,10.2},{0,20.1},{-1.77636e-15,20.1},{-1.77636e-15,30}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,80}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end Condenser_reverse;
