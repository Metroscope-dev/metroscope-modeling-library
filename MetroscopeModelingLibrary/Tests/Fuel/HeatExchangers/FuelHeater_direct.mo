within MetroscopeModelingLibrary.Tests.Fuel.HeatExchangers;
model FuelHeater_direct
  extends MetroscopeModelingLibrary.Icons.Tests.FuelTestIcon;
  import MetroscopeModelingLibrary.Units;

          // Boundary conditions
  input Real P_hot_source(start=1.5, min=0, nominal=1) "barA";
  input Units.MassFlowRate Q_hot_source(start=586) "kg/s";
  input Real hot_source_h(start=600000) "J/kg";

  input Real P_cold_source(start=3.5, min=1.5, nominal=3.5) "barA";
  input Units.MassFlowRate Q_cold_source(start=11) "kg/s";
  input Units.SpecificEnthalpy h_cold_source(start=1e6) "J/kg";

  // Parameters
  parameter String QCp_max_side = "hot";
  parameter Units.Area S = 10;
  parameter Units.HeatExchangeCoefficient Kth = 8740;
  parameter Units.FrictionCoefficient Kfr_hot = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 1;

  MultiFluid.HeatExchangers.FuelHeater fuelHeater(QCp_max_side=QCp_max_side) annotation (Placement(transformation(extent={{-50,-48},{50,48}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-78,-10},{-58,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={16,62})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-14,62})));
equation

  hot_source.P_out = P_hot_source * 1e5;
  hot_source.h_out = hot_source_h;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source *1e5;
  cold_source.h_out = h_cold_source;
  cold_source.Q_out = - Q_cold_source;
  cold_source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  fuelHeater.S = S;
  fuelHeater.Kth = Kth;
  fuelHeater.Kfr_hot = Kfr_hot;
  fuelHeater.Kfr_cold = Kfr_cold;



  connect(fuelHeater.C_cold_in, cold_source.C_out) annotation (Line(points={{-35,0},{-63,0}}, color={213,213,0}));
  connect(fuelHeater.C_cold_out, cold_sink.C_in) annotation (Line(points={{35,0},{65,0}}, color={213,213,0}));
  connect(fuelHeater.C_hot_out, hot_sink.C_in) annotation (Line(points={{-15,33.6},{-14,33.6},{-14,57}}, color={28,108,200}));
  connect(fuelHeater.C_hot_in, hot_source.C_out) annotation (Line(points={{15,33.6},{16,33.6},{16,57}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end FuelHeater_direct;
