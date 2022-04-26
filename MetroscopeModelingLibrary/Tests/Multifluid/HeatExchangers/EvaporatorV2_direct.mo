within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model EvaporatorV2_direct

  // Boundary conditions
  input Real P_hot_source(start=1.5, min=0, nominal=1) "barA";
  input Units.MassFlowRate Q_hot_source(start=586) "kg/s";
  input Real hot_source_h(start=600000) "J/kg";

  input Real P_cold_source(start=3.5, min=1.5, nominal=3.5) "barA";
  input Units.MassFlowRate Q_cold_source(start=11) "kg/s";
  input Real T_cold_source(start = 201, min = 130, nominal = 150) "degC";

     // Parameters
  //parameter String QCp_max_side = "hot";
  parameter Units.Area S = 10;
  parameter Units.HeatExchangeCoefficient Kth = 8740;
  parameter Units.FrictionCoefficient Kfr_hot = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 1;
  //parameter Units.Temperature nominal_hot_side_temperature_rise = 3;


  MultiFluid.HeatExchangers.Evaporator_2phases_outlet evaporator_2phases_outlet annotation (Placement(transformation(extent={{-50,-56},{50,58}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={14,70})));
  WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-14,70})));
equation
  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.h_out = hot_source_h;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;


  evaporator_2phases_outlet.S_vaporising = S;
  evaporator_2phases_outlet.Kth = Kth;
  evaporator_2phases_outlet.Kfr_hot = Kfr_hot;
  evaporator_2phases_outlet.Kfr_cold = Kfr_cold;
  //evaporator_2phases_outlet.nominal_hot_side_temperature_rise = nominal_hot_side_temperature_rise;


  connect(evaporator_2phases_outlet.C_hot_in, hot_source.C_out) annotation (Line(points={{-35,-0.14},{-35,0},{-63,0}}, color={95,95,95}));
  connect(evaporator_2phases_outlet.C_hot_out, hot_sink.C_in) annotation (Line(points={{35,-0.14},{56,-0.14},{56,0},{75,0}}, color={95,95,95}));
  connect(cold_sink.C_in, evaporator_2phases_outlet.C_cold_out) annotation (Line(points={{-14,65},{-14,52.95},{-15,52.95},{-15,40.9}}, color={28,108,200}));
  connect(cold_source.C_out, evaporator_2phases_outlet.C_cold_in) annotation (Line(points={{14,65},{14,40.9},{15,40.9}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end EvaporatorV2_direct;
