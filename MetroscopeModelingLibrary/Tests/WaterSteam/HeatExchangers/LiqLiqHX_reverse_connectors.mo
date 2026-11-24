within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model LiqLiqHX_reverse_connectors

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

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{58,-10},
            {78,10}})));
  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.LiqLiqHX liqLiqHX annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,30})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-38})));
  Utilities.Interfaces.RealOutput Kfr_cold annotation (Placement(transformation(
          extent={{-42,18},{-34,26}}), iconTransformation(extent={{-224,42},{-204,
            62}})));
  Utilities.Interfaces.RealOutput Kth annotation (Placement(transformation(
          extent={{-26,32},{-18,40}}), iconTransformation(extent={{-216,42},{-196,
            62}})));
  Utilities.Interfaces.RealOutput Kfr_hot annotation (Placement(transformation(
          extent={{12,36},{20,44}}), iconTransformation(extent={{-218,52},{-198,
            72}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_out_sensor(
    T_start=55,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{22,-6},{36,8}})));
  Sensors_Control.WaterSteam.PressureSensor P_hot_out_sensor(
    P_start=50,
    signal_unit="barA",
    display_unit="barA")                                     annotation (
      Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=270,
        origin={1,-19})));
  Utilities.Interfaces.RealInput P_hot_out annotation (Placement(transformation(
          extent={{16,-24},{24,-16}}), iconTransformation(extent={{-156,-8},{-116,
            32}})));
  Utilities.Interfaces.RealInput T_cold_out annotation (Placement(
        transformation(extent={{28,18},{36,26}}), iconTransformation(extent={{-156,
            -8},{-116,32}})));
  Utilities.Interfaces.RealInput P_cold_out annotation (Placement(
        transformation(extent={{44,14},{52,22}}), iconTransformation(extent={{-156,
            -8},{-116,32}})));
equation

  // Boundary conditions
public
  Sensors_Control.WaterSteam.PressureSensor P_cold_out_sensor(
    P_start=19,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{42,-6},{56,8}})));
equation
  hot_source.P_out = P_hot_source * 1e5;
  hot_source.T_out = 273.15 + T_hot_source;
  hot_source.Q_out = - Q_hot_source;
  cold_source.P_out = P_cold_source *1e5;
  cold_source.T_out = 273.15 + T_cold_source;
  cold_source.Q_out = - Q_cold_source;

  connect(hot_source.C_out, liqLiqHX.C_hot_in) annotation (Line(points={{-8.88178e-16,
          25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
  connect(cold_source.C_out, liqLiqHX.C_cold_in)
    annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
  connect(liqLiqHX.Kfr_cold, Kfr_cold) annotation (Line(points={{-18,3.2},{-30,3.2},
          {-30,22},{-38,22}}, color={0,0,127}));
  connect(liqLiqHX.Kth, Kth)
    annotation (Line(points={{-9.6,10},{-22,10},{-22,36}}, color={0,0,127}));
  connect(Kfr_hot, liqLiqHX.Kfr_hot) annotation (Line(points={{16,40},{16,20},{14,
          20},{14,10},{4.8,10}}, color={0,0,127}));
  connect(liqLiqHX.C_cold_out, T_cold_out_sensor.C_in)
    annotation (Line(points={{16,0},{16,1},{22,1}}, color={28,108,200}));
  connect(cold_sink.C_in, P_cold_out_sensor.C_out)
    annotation (Line(points={{63,0},{58,1},{56,1}}, color={28,108,200}));
  connect(T_cold_out_sensor.C_out, P_cold_out_sensor.C_in)
    annotation (Line(points={{36,1},{42,1}}, color={28,108,200}));
  connect(Kfr_hot, Kfr_hot)
    annotation (Line(points={{16,40},{16,40}}, color={0,0,127}));
  connect(hot_sink.C_in, P_hot_out_sensor.C_out)
    annotation (Line(points={{0,-33},{0,-26},{1,-26}}, color={28,108,200}));
  connect(liqLiqHX.C_hot_out, P_hot_out_sensor.C_in)
    annotation (Line(points={{0,-8},{0,-12},{1,-12}}, color={28,108,200}));
  connect(P_hot_out_sensor.P_sensor, P_hot_out)
    annotation (Line(points={{8,-19},{8,-20},{20,-20}}, color={0,0,127}));
  connect(T_cold_out_sensor.T_sensor, T_cold_out)
    annotation (Line(points={{29,8},{32,8},{32,22}}, color={0,0,127}));
  connect(P_cold_out_sensor.P_sensor, P_cold_out) annotation (Line(points={{49,8},
          {50,8},{50,12},{48,12},{48,18}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{100,100}})),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}})));
end LiqLiqHX_reverse_connectors;
