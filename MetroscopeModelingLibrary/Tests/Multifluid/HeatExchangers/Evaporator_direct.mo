within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Evaporator_direct
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


  parameter Units.Temperature nominal_hot_side_temperature_rise = 3;



  MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-50,-48},{50,52}})));
  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{86,28},{66,48}})));
  WaterSteam.BoundaryConditions.Sink cold_liquid_sink annotation (Placement(transformation(extent={{-60,-62},{-80,-42}})));
  WaterSteam.BoundaryConditions.Sink cold_steam_sink annotation (Placement(transformation(extent={{-58,50},{-78,70}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-86,-8},{-66,12}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{72,-8},{92,12}})));
equation
    hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.h_out = hot_source_h;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  evaporator.S_vaporising = S;
  evaporator.Kth = Kth;
  evaporator.Kfr_hot = Kfr_hot;
  evaporator.Kfr_cold = Kfr_cold;
  evaporator.nominal_hot_side_temperature_rise = nominal_hot_side_temperature_rise;

  connect(evaporator.C_cold_liq_out, cold_liquid_sink.C_in) annotation (Line(points={{-35,-43},{-35,-52},{-65,-52}}, color={28,108,200}));
  connect(evaporator.C_cold_in, cold_source.C_out) annotation (Line(points={{25,37},{48,37},{48,38},{71,38}}, color={28,108,200}));
  connect(evaporator.C_cold_vap_out, cold_steam_sink.C_in) annotation (Line(points={{-35,47},{-35,60},{-63,60}}, color={28,108,200}));
  connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-35,1},{-70,1},{-70,2},{-71,2}}, color={95,95,95}));
  connect(evaporator.C_hot_out, hot_sink.C_in) annotation (Line(points={{35,1},{76,1},{76,2},{77,2}}, color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Evaporator_direct;
