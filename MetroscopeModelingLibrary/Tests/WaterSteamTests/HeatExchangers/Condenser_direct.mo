within MetroscopeModelingLibrary.Tests.WaterSteamTests.HeatExchangers;
model Condenser_direct

  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

    // Boundary conditions
  input Units.MassFlowRate Q_turbine(start=150) "kg/s";
  input Units.SpecificEnthalpy h_turbine(start=1500e3);

  input Real P_cold_source(start=5, min=0, nominal=10) "barA";
  input Real T_cold_source(start = 15, min = 0, nominal = 50) "degC";

    // Parameters
  parameter Units.Area S = 100;
  parameter Units.HeatExchangeCoefficient Kth = 50000;
  parameter Units.Height water_height = 2;
  parameter Units.FrictionCoefficient Kfr_cold = 1;
  parameter Units.VolumeFlowRate Qv_cold = 3.82;
  parameter Units.Pressure P_offset = 0;
  parameter Units.Pressure C_incond = 0;


  WaterSteam.BoundaryConditions.WaterSource cooling_source
    annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  WaterSteam.BoundaryConditions.WaterSink cooling_sink
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  WaterSteam.HeatExchangers.Condenser condenser
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  WaterSteam.BoundaryConditions.WaterSource turbine_outlet annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,30})));
  WaterSteam.BoundaryConditions.WaterSink condensate_sink annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-30})));
equation

  turbine_outlet.h_out = h_turbine;
  turbine_outlet.Q_out = -Q_turbine;

  cooling_source.P_out = P_cold_source*1e5;
  cooling_source.T_out = 273.15 + T_cold_source;

  condenser.S = S;
  condenser.Kth = Kth;
  condenser.water_height = water_height;
  condenser.Kfr_cold = Kfr_cold;
  condenser.Qv_cold_in = Qv_cold;
  condenser.P_offset = P_offset;
  condenser.C_incond = C_incond;

  connect(condenser.C_cold_out, cooling_sink.C_in) annotation (Line(points={{16,
          -1.42222},{30,-1.42222},{30,0},{45,0}}, color={28,108,200}));
  connect(condensate_sink.C_in, condenser.C_hot_out) annotation (Line(points={{8.88178e-16,
          -25},{8.88178e-16,-20.5},{0,-20.5},{0,-8.35556}}, color={28,108,200}));
  connect(turbine_outlet.C_out, condenser.C_hot_in) annotation (Line(points={{-8.88178e-16,
          25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
  connect(cooling_source.C_out, condenser.C_cold_in) annotation (Line(points={{-43,
          0},{-30,0},{-30,3.55556},{-16.64,3.55556}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,80}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end Condenser_direct;
