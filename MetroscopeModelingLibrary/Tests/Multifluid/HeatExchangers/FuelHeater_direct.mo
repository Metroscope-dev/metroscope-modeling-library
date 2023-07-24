within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model FuelHeater_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

          // Boundary conditions
  input Real P_hot_source(start=47, min=0, nominal=10) "barA";
  input Units.MassFlowRate Q_hot_source(start=8) "kg/s";
  input Real T_hot_source(start=230) "J/kg";

  input Real P_cold_source(start=30, min=0, nominal=10) "barA";
  input Units.MassFlowRate Q_cold_source(start=12) "kg/s";
  input Real T_cold_source(start = 30, min = 0, nominal = 50) "degC";

  // Parameters
  parameter String QCp_max_side = "undefined"; // On fuel heater, QCp_hot may be close to QCp_cold
  parameter Units.Area S = 10;
  parameter Units.Temperature nominal_cold_side_temperature_rise = 20;
  parameter Units.Temperature nominal_hot_side_temperature_drop = 10;

  parameter Units.HeatExchangeCoefficient Kth = 8740;
  parameter Units.FrictionCoefficient Kfr_hot = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 1;

  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-74,-10},{-54,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source   annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={64,40})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-64,-40})));
  MultiFluid.HeatExchangers.FuelHeater fuelHeater annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
equation

  hot_source.P_out = P_hot_source * 1e5;
  hot_source.T_out = T_hot_source+273.15;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = T_cold_source+273.15;
  cold_source.Q_out = - Q_cold_source;
  cold_source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  fuelHeater.S = S;
  fuelHeater.nominal_cold_side_temperature_rise = nominal_cold_side_temperature_rise;
  fuelHeater.nominal_hot_side_temperature_drop = nominal_hot_side_temperature_drop;

  fuelHeater.Kth = Kth;
  fuelHeater.Kfr_hot = Kfr_hot;
  fuelHeater.Kfr_cold = Kfr_cold;

  connect(fuelHeater.C_cold_in, cold_source.C_out) annotation (Line(points={{-6,0},{-59,0}}, color={213,213,0}));
  connect(fuelHeater.C_cold_out, cold_sink.C_in) annotation (Line(points={{14,0},{59,0}}, color={213,213,0}));
  connect(hot_source.C_out, fuelHeater.C_hot_in) annotation (Line(points={{59,40},{8,40},{8,8}}, color={28,108,200}));
  connect(fuelHeater.C_hot_out, hot_sink.C_in) annotation (Line(points={{0,-8},{0,-40},{-59,-40}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end FuelHeater_direct;
