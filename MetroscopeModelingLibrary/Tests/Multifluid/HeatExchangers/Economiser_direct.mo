within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Economiser_direct
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
  parameter Units.HeatExchangeCoefficient Kth = 500;
  parameter Units.FrictionCoefficient Kfr_hot = 0;
  parameter Units.FrictionCoefficient Kfr_cold = 20;

  parameter Units.Temperature nominal_cold_side_temperature_rise = 20;
  parameter Units.Temperature nominal_hot_side_temperature_rise = 10;


  MultiFluid.HeatExchangers.Economiser economiser(QCp_max_side=QCp_max_side)  annotation (Placement(transformation(extent={{-36,-34},{34,34}})));
  WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={10,-50})));
  WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-12,46})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-64,-10},{-44,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{58,-10},{78,10}})));
equation

  hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.h_out = hot_source_h;
  hot_source.Q_out = - Q_hot_source;

  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  economiser.S = S;
  economiser.Kth = Kth;
  economiser.Kfr_hot = Kfr_hot;
  economiser.Kfr_cold = Kfr_cold;
  economiser.nominal_cold_side_temperature_rise = nominal_cold_side_temperature_rise;
  economiser.nominal_hot_side_temperature_rise = nominal_hot_side_temperature_rise;

  connect(cold_source.C_out, economiser.C_cold_in) annotation (Line(points={{10,
          -45},{10,-23.8},{9.5,-23.8}}, color={28,108,200}));
  connect(economiser.C_cold_out, cold_sink.C_in) annotation (Line(points={{-11.5,
          23.8},{-11.5,32.4},{-12,32.4},{-12,41}}, color={28,108,200}));
  connect(economiser.C_hot_in, hot_source.C_out)
    annotation (Line(points={{-25.5,0},{-49,0}}, color={95,95,95}));
  connect(economiser.C_hot_out, hot_sink.C_in)
    annotation (Line(points={{23.5,0},{63,0}}, color={95,95,95}));
end Economiser_direct;
