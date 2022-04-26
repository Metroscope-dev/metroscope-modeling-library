within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Economiser_reverse
  extends MetroscopeModelingLibrary.Icons.Tests.MultifluidTestIcon;
    // Boundary conditions
  input Real P_hot_source(start=50, min=0, nominal=10) "barA";
  input Units.MassFlowRate Q_hot_source(start=50) "kg/s";
  input Real hot_source_h(start=0.7e6) "J/kg";

  input Real P_cold_source(start=20, min=0, nominal=10) "barA";
  input Units.MassFlowRate Q_cold_source(start=100) "kg/s";
  input Real T_cold_source(start = 50, min = 0, nominal = 50) "degC";

  // Parameters
  parameter String QCp_max_side = "cold";
  parameter Units.Area S = 100;
  parameter Units.Temperature nominal_cold_side_temperature_rise = 20;
  parameter Units.Temperature nominal_hot_side_temperature_rise = 10;

  // Calibrated parameters
  output Units.HeatExchangeCoefficient Kth;
  output Units.FrictionCoefficient Kfr_hot;
  output Units.FrictionCoefficient Kfr_cold;


  // Calibration inputs
  input Real P_cold_out(start = 19, min= 0, nominal = 10) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real P_hot_out(start = 50, min = 0, nominal = 10) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
  input Real T_cold_out(start = 55, min = 0, nominal = 100) "degC"; // Outlet temperature on cold side, to calibrate Kth

  MultiFluid.HeatExchangers.Economiser economiser(QCp_max_side=QCp_max_side)  annotation (Placement(transformation(extent={{-36,-34},
            {34,34}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={10,42})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-12,-70})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-60,-10},
            {-40,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{60,-10},
            {80,10}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_cold_out_sensor annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=90,
        origin={-12,-52})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cold_out_sensor annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=90,
        origin={-12,-38})));
  MetroscopeModelingLibrary.Sensors.FlueGases.PressureSensor P_hot_out_sensor annotation (Placement(transformation(extent={{40,-4},{48,4}})));
equation
  // Boundary conditions
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.h_out = hot_source_h;
  hot_source.Q_out = - Q_hot_source;
  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  // Parameters
  economiser.S = S;
  economiser.nominal_cold_side_temperature_rise = nominal_cold_side_temperature_rise;
  economiser.nominal_hot_side_temperature_rise = nominal_hot_side_temperature_rise;

    // Inputs for calibration
  T_cold_out_sensor.T_degC = T_cold_out;
  P_cold_out_sensor.P_barA = P_cold_out;
  P_hot_out_sensor.P_barA = P_hot_out;

  // Calibrated parameters
  economiser.Kth = Kth;
  economiser.Kfr_hot = Kfr_hot;
  economiser.Kfr_cold = Kfr_cold;

  connect(P_cold_out_sensor.C_out, T_cold_out_sensor.C_in)
    annotation (Line(points={{-12,-42},{-12,-48}},
                                                 color={28,108,200}));
  connect(cold_sink.C_in, T_cold_out_sensor.C_out)
    annotation (Line(points={{-12,-65},{-12,-56}},
                                                 color={28,108,200}));
  connect(economiser.C_hot_in, hot_source.C_out)
    annotation (Line(points={{-25.5,0},{-45,0}}, color={95,95,95}));
  connect(economiser.C_hot_out, P_hot_out_sensor.C_in)
    annotation (Line(points={{23.5,0},{40,0}}, color={95,95,95}));
  connect(hot_sink.C_in, P_hot_out_sensor.C_out)
    annotation (Line(points={{65,0},{48,0}}, color={95,95,95}));
  connect(P_cold_out_sensor.C_in, economiser.C_cold_out) annotation (Line(points={{-12,-34},{-12,-28.9},{-11.5,-28.9},{-11.5,-23.8}}, color={28,108,200}));
  connect(cold_source.C_out, economiser.C_cold_in) annotation (Line(points={{10,37},{9.5,37},{9.5,23.8}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(lineColor = {75,138,73},
                fillColor={255,255,255},
                fillPattern = FillPattern.Solid,
                extent={{-100,-100},{100,100}}),
        Polygon(
          origin={20,14},
          lineColor={78,138,73},
          fillColor={95,95,95},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}}),
        Polygon(
          origin={20,14},
          lineColor={78,138,73},
          fillColor={213,213,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58,46},{-4,14},{-58,-14},{-58,46}}),
        Polygon(
          origin={20,14},
          lineColor={78,138,73},
          fillColor={28,108,200},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Economiser_reverse;
