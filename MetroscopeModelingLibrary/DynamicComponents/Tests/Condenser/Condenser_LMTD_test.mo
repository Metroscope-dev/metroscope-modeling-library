within MetroscopeModelingLibrary.DynamicComponents.Tests.Condenser;
model Condenser_LMTD_test

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

    // Boundary conditions
  input Utilities.Units.MassFlowRate Q_turbine(start=105.7) "kg/s";
  input Utilities.Units.SpecificEnthalpy h_turbine(start=2417031.8);
  input Real P_cold_source(start=5, min=0, nominal=10) "barA";
  input Real T_cold_source(start = 16.25, min = 0, nominal = 50) "degC";

    // Parameters
  parameter Utilities.Units.VolumeFlowRate Qv_cold=7.438; // 7.438

    // Calibrated parameters

    // Inputs for calibration
  input Real P_cond(start = 0.035e5) "Pa";

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_outlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,78})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink condensate_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-44})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cooling_source annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={84,-20})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cooling_sink annotation (Placement(transformation(extent={{74,-10},{94,10}})));
  HeatExchangers.Condenser.Condenser_LMTD_test                  condenser_LMTD_test(
                                                                               steady_state=false, T_cold_out(start=20+273.15))
                                                                          annotation (Placement(transformation(extent={{-10,-8},{10,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor circulating_water_T_out_sensor annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={34,2})));
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
  condenser_LMTD_test.Qv_cold = Qv_cold;

  // Calibrated Parameters
  condenser_LMTD_test.UA = 25e6;

  // Inputs for calibration
  P_cond_sensor.P = P_cond + 0.01e5*time;

  connect(condenser_LMTD_test.C_cold_in, cooling_source.C_out) annotation (Line(points={{10,-0.8},{20,-0.8},{20,-20},{79,-20}}, color={28,108,200}));
  connect(condenser_LMTD_test.C_cold_out, circulating_water_T_out_sensor.C_in) annotation (Line(points={{10,2.8},{16,2.8},{16,2},{24,2}}, color={28,108,200}));
  connect(circulating_water_T_out_sensor.C_out, circulating_water_P_out_sensor.C_in) annotation (Line(points={{44,2},{48,2},{48,0},{50,0}},
                                                                                                                              color={28,108,200}));
  connect(circulating_water_P_out_sensor.C_out, cooling_sink.C_in) annotation (Line(points={{70,0},{79,0}}, color={28,108,200}));
  connect(turbine_outlet.C_out, P_cond_sensor.C_in) annotation (Line(points={{-8.88178e-16,73},{-8.88178e-16,61.5},{1.77636e-15,61.5},{1.77636e-15,50}}, color={28,108,200}));
  connect(condenser_LMTD_test.C_hot_in, P_cond_sensor.C_out) annotation (Line(points={{0,10},{0,20.1},{0,20.1},{0,30}}, color={28,108,200}));
  connect(condensate_sink.C_in, condenser_LMTD_test.C_hot_out) annotation (Line(points={{0,-39},{0,-8}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,80}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end Condenser_LMTD_test;
