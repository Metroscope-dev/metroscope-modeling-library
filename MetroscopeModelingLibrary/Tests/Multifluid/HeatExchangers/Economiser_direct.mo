within MetroscopeModelingLibrary.Tests.Multifluid.HeatExchangers;
model Economiser_direct
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;

      // Boundary conditions
  input Real P_hot_source(start=50, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_hot_source(start=50) "kg/s";
  input Real hot_source_h(start=0.7e6) "J/kg";

  input Real P_cold_source(start=20, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start=100) "kg/s";
  input Real T_cold_source(start = 50, min = 0, nominal = 50) "degC";

  // Parameters
  parameter String QCp_max_side = "cold";
  parameter Utilities.Units.Area S=100;
  parameter Utilities.Units.HeatExchangeCoefficient Kth=500;
  parameter Utilities.Units.FrictionCoefficient Kfr_hot=0;
  parameter Utilities.Units.FrictionCoefficient Kfr_cold=20;

  parameter Utilities.Units.Temperature nominal_cold_side_temperature_rise=20;
  parameter Utilities.Units.Temperature nominal_hot_side_temperature_drop=10;


  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={10,44})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-12,-46})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source hot_source annotation (Placement(transformation(extent={{-64,-10},{-44,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(extent={{58,-10},{78,10}})));
  MultiFluid.HeatExchangers.Economiser economiser(QCp_max_side=QCp_max_side) annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
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
  economiser.nominal_hot_side_temperature_drop = nominal_hot_side_temperature_drop;

  connect(economiser.C_cold_in, cold_source.C_out) annotation (Line(points={{1,7},{0,7},{0,30},{10,30},{10,39}}, color={28,108,200}));
  connect(economiser.C_hot_out, hot_sink.C_in) annotation (Line(points={{5,0},{63,0}}, color={95,95,95}));
  connect(economiser.C_cold_out, cold_sink.C_in) annotation (Line(points={{-5,7},{-5,-41},{-12,-41}},  color={28,108,200}));
  connect(economiser.C_hot_in, hot_source.C_out) annotation (Line(points={{-9,0},{-49,0}}, color={95,95,95}));
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
end Economiser_direct;
