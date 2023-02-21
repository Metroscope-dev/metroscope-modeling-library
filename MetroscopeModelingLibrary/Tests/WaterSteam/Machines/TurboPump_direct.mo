within MetroscopeModelingLibrary.Tests.WaterSteam.Machines;
model TurboPump_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  output Real pump_T_out(start=200);
  output Real pump_P_out(start=60);
  output Real pump_VRot(start=3500);

  parameter Real turbo_pump_hn = 40;
  parameter Real turbo_pump_a3 = 50;
  parameter Real turbo_pump_b3 = 0.8;
  MetroscopeModelingLibrary.WaterSteam.Machines.TurboPump turbo_pump annotation (Placement(transformation(extent={{-26,-12},{26,40}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_source annotation (Placement(transformation(extent={{-98,10},{-78,30}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink turbine_sink annotation (Placement(transformation(extent={{48,30},{68,50}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source pump_source annotation (Placement(transformation(extent={{68,-10},{48,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink pump_sink annotation (Placement(transformation(extent={{-78,-10},{-98,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor pump_P_out_sensor annotation (Placement(transformation(extent={{-50,-10},{-70,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor pump_T_out_sensor annotation (Placement(transformation(extent={{-20,-10},{-40,10}})));
  MetroscopeModelingLibrary.Sensors.Outline.VRotSensor pump_VRot_sensor annotation (Placement(transformation(extent={{-10,-40},{10,-60}})));
equation
  // Boundary conditions
  turbine_source.h_out = 2.9e6;
  turbine_source.P_out = 11e5;
  turbine_sink.P_in = 0.052e5;

  pump_source.T_out = 195 + 273.15;
  pump_source.P_out = 40e5;
  pump_source.Q_out = -1500;

  // Hypothesis on component parameters
    // Pump
      turbo_pump.a1 = 0;
      turbo_pump.a2 = 0;
      turbo_pump.b1 = 0;
      turbo_pump.b2 = 0;
      turbo_pump.rhmin = 0.2;
      turbo_pump.VRotn = 4000;

    // Turbine
      turbo_pump.eta_is = 0.6; // can be calibrated using turbine outlet enthalpy value from HMBD
      // turbo_pump.Cst is left free because both inlet and outlet turbine pressures are local boundary conditions

  // Observables for calibration
    // Turbine : None, all observables are usually local boundary conditions for this system

    // Pump
      pump_T_out_sensor.T_degC = pump_T_out;
      pump_P_out_sensor.P_barA = pump_P_out;
      pump_VRot_sensor.VRot = pump_VRot;

  // Calibrated parameters
    turbo_pump.hn = turbo_pump_hn; // Calibrated by pump_P_out
    turbo_pump.a3 = turbo_pump_a3; // Calibrated by VRot
    turbo_pump.b3 = turbo_pump_b3; // Calibrated by pump_T_out

  connect(turbine_source.C_out, turbo_pump.C_turbine_in) annotation (Line(points={{-83,20},{-6,20}}, color={28,108,200}));
  connect(turbo_pump.C_turbine_out, turbine_sink.C_in) annotation (Line(points={{10,40},{53,40}}, color={28,108,200}));
  connect(turbo_pump.C_pump_in, pump_source.C_out) annotation (Line(points={{10,0},{53,0}}, color={28,108,200}));
  connect(pump_P_out_sensor.C_out, pump_sink.C_in) annotation (Line(points={{-70,0},{-83,0}}, color={28,108,200}));
  connect(turbo_pump.C_pump_out, pump_T_out_sensor.C_in) annotation (Line(points={{-10,0},{-20,0}}, color={28,108,200}));
  connect(pump_T_out_sensor.C_out, pump_P_out_sensor.C_in) annotation (Line(points={{-40,0},{-50,0}}, color={28,108,200}));
  connect(turbo_pump.pump_VRot, pump_VRot_sensor.VRot) annotation (Line(points={{0.1,-10.9},{0,-10.9},{0,-39.8}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end TurboPump_direct;
