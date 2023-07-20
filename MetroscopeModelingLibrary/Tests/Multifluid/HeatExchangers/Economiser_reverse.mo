within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Economiser_reverse
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;

  // Boundary conditions
  input Real P_hot_source(start = 1.1, min = 0, nominal = 10) "barA";
  input Utilities.Units.MassFlowRate Q_hot_source(start = 640) "kg/s";
  input Utilities.Units.Temperature T_hot_source(start = 300) "degC";

  input Real P_cold_source(start = 170, min = 0, nominal = 10) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start = 85) "kg/s";
  input Real T_cold_source(start = 227, min = 0, nominal = 50) "degC";

  // Parameters
  parameter String QCp_max_side = "hot";
  parameter Utilities.Units.Area S = 10000;
  parameter Utilities.Units.Temperature nominal_cold_side_temperature_rise = 43;
  parameter Utilities.Units.Temperature nominal_hot_side_temperature_drop = 27;

  // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth;
  output Utilities.Units.FrictionCoefficient Kfr_hot;
  output Utilities.Units.FrictionCoefficient Kfr_cold;

  // Calibration inputs
  input Real P_cold_out(start = 169.5, min =  0, nominal = 120) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real P_hot_out(start = 1.05, min = 0, nominal = 1) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
  input Real T_cold_out(start = 270, min = 0, nominal = 100) "degC"; // Outlet temperature on cold side, to calibrate Kth

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent = {{10,-10},{-10,10}},
        rotation=0,
        origin={72,40})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent = {{10,-10},{-10,10}},
        rotation=0,
        origin={-86,40})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-96,-10},{-76,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  MultiFluid.HeatExchangers.Economiser economiser annotation (Placement(transformation(extent={{-14,-10},{6,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_cold_out_sensor annotation (Placement(transformation(extent={{-20,30},{-40,50}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cold_out_sensor annotation (Placement(transformation(extent={{-48,30},{-68,50}})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor P_hot_out_sensor annotation (Placement(transformation(extent={{30,-10},{50,10}})));
equation

  // Boundary conditions
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source*1e5;
  hot_source.T_out = T_hot_source + 273.15;
  hot_source.Q_out = - Q_hot_source;
  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  // Parameters
  economiser.S = S;
  economiser.nominal_cold_side_temperature_rise = nominal_cold_side_temperature_rise;
  economiser.nominal_hot_side_temperature_drop = nominal_hot_side_temperature_drop;

  // Inputs for calibration
  T_cold_out_sensor.T_degC = T_cold_out;
  P_cold_out_sensor.P_barA = P_cold_out;
  P_hot_out_sensor.P_barA = P_hot_out;

  // Calibrated parameters
  economiser.Kth = Kth;
  economiser.Kfr_hot = Kfr_hot;
  economiser.Kfr_cold = Kfr_cold;

  connect(economiser.C_hot_in, hot_source.C_out) annotation (Line(
      points={{-14,0},{-81,0}},
      color={95,95,95},
      thickness=1));
  connect(economiser.C_hot_out, P_hot_out_sensor.C_in) annotation (Line(
      points={{6,0},{30,0}},
      color={95,95,95},
      thickness=1));
  connect(P_hot_out_sensor.C_out, hot_sink.C_in) annotation (Line(
      points={{50,0},{67,0}},
      color={95,95,95},
      thickness=1));
  connect(economiser.C_cold_in, cold_source.C_out) annotation (Line(
      points={{0,8},{0,40},{67,40}},
      color={28,108,200},
      thickness=1));
  connect(economiser.C_cold_out, T_cold_out_sensor.C_in) annotation (Line(
      points={{-8,8},{-8,40},{-20,40}},
      color={28,108,200},
      thickness=1));
  connect(T_cold_out_sensor.C_out, P_cold_out_sensor.C_in) annotation (Line(
      points={{-40,40},{-48,40}},
      color={28,108,200},
      thickness=1));
  connect(P_cold_out_sensor.C_out, cold_sink.C_in) annotation (Line(
      points={{-68,40},{-81,40}},
      color={28,108,200},
      thickness=1));
  annotation (Icon(coordinateSystem(preserveAspectRatio = false, extent={{-100,-100},{100,100}}),
                                                                  graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor = {255,255,255},
                fillPattern = FillPattern.Solid,
                extent = {{-100,-100},{100,100}}),
        Polygon(
          origin = {20,14},
          lineColor = {78,138,73},
          fillColor = {95,95,95},
          pattern = LinePattern.None,
          fillPattern = FillPattern.Solid,
          points = {{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}}),
        Polygon(
          origin = {20,14},
          lineColor = {78,138,73},
          fillColor = {213,213,0},
          pattern = LinePattern.None,
          fillPattern = FillPattern.Solid,
          points = {{-58,46},{-4,14},{-58,-14},{-58,46}}),
        Polygon(
          origin = {20,14},
          lineColor = {78,138,73},
          fillColor = {28,108,200},
          pattern = LinePattern.None,
          fillPattern = FillPattern.Solid,
          points = {{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}), Diagram(
        coordinateSystem(preserveAspectRatio = false, extent={{-100,-100},{100,100}})));
end Economiser_reverse;
