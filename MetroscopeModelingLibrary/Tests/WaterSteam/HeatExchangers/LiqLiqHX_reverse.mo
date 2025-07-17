within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model LiqLiqHX_reverse

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

    // Boundary conditions
  input Real P_hot_source(start=50, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_hot_source(start=50) "kg/s";
  input Real T_hot_source(start = 100, min = 0, nominal = 50) "degC";

  input Real P_cold_source(start=20, min=0, nominal=10) "barA";
  input Utilities.Units.MassFlowRate Q_cold_source(start=100) "kg/s";
  input Real T_cold_source(start = 50, min = 0, nominal = 50) "degC";

    // Parameters
  parameter Utilities.Units.Area S=100;

    // Calibrated parameters
  output Utilities.Units.HeatExchangeCoefficient Kth;
  output Utilities.Units.FrictionCoefficient Kfr_hot;
  output Utilities.Units.FrictionCoefficient Kfr_cold;

    // Calibration inputs
  input Real P_cold_out(start = 19, min= 0, nominal = 10) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
  input Real P_hot_out(start = 50, min = 0, nominal = 10) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
  input Real T_cold_out(start = 55, min = 0, nominal = 100) "degC"; // Outlet temperature on cold side, to calibrate Kth

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{52,-10},{72,10}})));
  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.LiqLiqHX liqLiqHX annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,30})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-36})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_cold_out_sensor annotation (Placement(transformation(extent={{22,-6},{34,6}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cold_out_sensor annotation (Placement(transformation(extent={{38,-6},{50,6}})));
  MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_hot_out_sensor annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={0,-18})));
equation

  // Boundary conditions
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.T_out = 273.15 + T_hot_source;
  hot_source.Q_out = - Q_hot_source;
  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;


  // Inputs for calibration
  T_cold_out_sensor.T_degC = T_cold_out;
  P_cold_out_sensor.P_barA = P_cold_out;
  P_hot_out_sensor.P_barA = P_hot_out;

  // Calibrated parameters
  liqLiqHX.Kth = Kth;
  liqLiqHX.Kfr_hot = Kfr_hot;
  liqLiqHX.Kfr_cold = Kfr_cold;

  connect(hot_source.C_out, liqLiqHX.C_hot_in) annotation (Line(points={{-8.88178e-16,
          25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
  connect(cold_source.C_out, liqLiqHX.C_cold_in)
    annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
  connect(P_cold_out_sensor.C_out, cold_sink.C_in)
    annotation (Line(points={{50,0},{57,0}}, color={28,108,200}));
  connect(P_cold_out_sensor.C_in, T_cold_out_sensor.C_out)
    annotation (Line(points={{38,0},{34,0}}, color={28,108,200}));
  connect(T_cold_out_sensor.C_in, liqLiqHX.C_cold_out)
    annotation (Line(points={{22,0},{16,0}}, color={28,108,200}));
  connect(hot_sink.C_in, P_hot_out_sensor.C_out) annotation (Line(points={{8.88178e-16,
          -31},{8.88178e-16,-27.5},{-1.11022e-15,-27.5},{-1.11022e-15,-24}},
        color={28,108,200}));
  connect(liqLiqHX.C_hot_out, P_hot_out_sensor.C_in) annotation (Line(points={{0,
          -8},{0,-10},{1.11022e-15,-10},{1.11022e-15,-12}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{100,100}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end LiqLiqHX_reverse;
