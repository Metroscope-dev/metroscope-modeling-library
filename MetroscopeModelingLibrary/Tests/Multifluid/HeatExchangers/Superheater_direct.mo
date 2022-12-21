within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Superheater_direct
    extends MetroscopeModelingLibrary.Icons.Tests.MultifluidTestIcon;

          // Boundary conditions
  input Real P_hot_source(start=1.5, min=0, nominal=1) "barA";
  input Units.MassFlowRate Q_hot_source(start=586) "kg/s";
  input Real hot_source_h(start=600000) "J/kg";

  input Real P_cold_source(start=3.5, min=1.5, nominal=3.5) "barA";
  input Units.MassFlowRate Q_cold_source(start=11) "kg/s";
  input Real T_cold_source(start = 201, min = 130, nominal = 150) "degC";

  // Parameters
  parameter String QCp_max_side = "hot";
  parameter Units.Area S = 10;
  parameter Units.HeatExchangeCoefficient Kth = 8740;
  parameter Units.FrictionCoefficient Kfr_hot = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 1;

  parameter Units.Temperature nominal_cold_side_temperature_rise = 140;
  parameter Units.Temperature nominal_hot_side_temperature_drop = 3;

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={12,44})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-10,-46})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-62,-10},{-42,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  MultiFluid.HeatExchangers.Superheater superheater(QCp_max_side=QCp_max_side)
                                                                             annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
equation
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.h_out = hot_source_h;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  superheater.S = S;
  superheater.Kth = Kth;
  superheater.Kfr_hot = Kfr_hot;
  superheater.Kfr_cold = Kfr_cold;
  superheater.nominal_cold_side_temperature_rise = nominal_cold_side_temperature_rise;
  superheater.nominal_hot_side_temperature_drop = nominal_hot_side_temperature_drop;

  connect(superheater.C_cold_in, cold_source.C_out) annotation (Line(points={{3,7},{2,7},{2,30},{12,30},{12,39}}, color={28,108,200}));
  connect(superheater.C_hot_out, hot_sink.C_in) annotation (Line(points={{7,0},{65,0}}, color={95,95,95}));
  connect(superheater.C_cold_out, cold_sink.C_in) annotation (Line(points={{-3,7},{-3,-41},{-10,-41}},  color={28,108,200}));
  connect(superheater.C_hot_in, hot_source.C_out) annotation (Line(points={{-7,0},{-47,0}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Superheater_direct;
