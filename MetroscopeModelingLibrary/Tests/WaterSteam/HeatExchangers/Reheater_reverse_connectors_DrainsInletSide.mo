within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Reheater_reverse_connectors_DrainsInletSide

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

  // This component represents an NPP reheater where the drains from the upstream heater enter the side of the heater
   // and passes through only the subcooling zone. It's identical to Reheater_reverse_connectors_drainsInletTop example except for the position of the drains source.

  // Boundary conditions
  input Real P_hot_source(start=11, min=0, nominal=11) "bar";
  input Real P_cold_source(start=50, min=0, nominal=50) "bar";
  input Utilities.Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
  input Real T_cold_in(start=50) "degC";
  input Utilities.Units.SpecificEnthalpy hot_source_h_out(start=2.9e6) "J/kg";

  input Real Q_drains( start=95) "kg/s";
  input Real h_drains( start = 8e5) "J/kg";


  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{74,-10},
            {94,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source Extr_source
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,30})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-56})));
  Utilities.Interfaces.RealOutput Kth_cond annotation (Placement(transformation(
          extent={{-34,-22},{-26,-14}}), iconTransformation(extent={{-254,-48},{
            -234,-28}})));
  Utilities.Interfaces.RealOutput Kfr_cold annotation (Placement(transformation(
          extent={{-40,12},{-32,20}}), iconTransformation(extent={{-250,-32},{-230,
            -12}})));
  Utilities.Interfaces.RealOutput Kth_subc annotation (Placement(transformation(
          extent={{-22,26},{-14,34}}), iconTransformation(extent={{-252,-40},{-232,
            -20}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_drains_sensor(
    T_start=80,
    signal_unit="degC",
    display_unit="degC") annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-30})));
  Sensors_Control.WaterSteam.TemperatureSensor T_cold_sink_sensor(
    T_start=70,
    signal_unit="degC",
    display_unit="degC")
    annotation (Placement(transformation(extent={{22,-10},{42,10}})));
  Sensors_Control.WaterSteam.PressureSensor P_cold_sink_sensor(
    P_start=49,
    signal_unit="barA",
    display_unit="barA")
    annotation (Placement(transformation(extent={{48,-10},{68,10}})));
  Utilities.Interfaces.RealInput T_drains annotation (Placement(transformation(
          extent={{22,-34},{30,-26}}), iconTransformation(extent={{-250,28},{-210,
            68}})));
  Utilities.Interfaces.RealInput T_cold_sink annotation (Placement(
        transformation(extent={{26,18},{34,26}}), iconTransformation(extent={{-262,
            44},{-222,84}})));
  Utilities.Interfaces.RealInput P_cold_sink annotation (Placement(
        transformation(extent={{52,20},{60,28}}), iconTransformation(extent={{-248,
            48},{-208,88}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater_NPP_drainsInlet
    reheater_NPP_drainsInlet
    annotation (Placement(transformation(extent={{-20,-8},{12,8}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source Drains_source
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={56,-28})));
equation

  // Boundary conditions
  Extr_source.P_out = P_hot_source*1e5;
  Extr_source.h_out = hot_source_h_out;

  cold_source.P_out = P_cold_source*1e5;
  cold_source.T_out = T_cold_in + 273.15;
  cold_source.Q_out = -Q_cold;

  Drains_source.Q_out = -Q_drains;
  Drains_source.h_out = h_drains;


  connect(cold_sink.C_in, P_cold_sink_sensor.C_out)
    annotation (Line(points={{79,0},{68,0}}, color={28,108,200}));
  connect(P_cold_sink_sensor.C_in, T_cold_sink_sensor.C_out)
    annotation (Line(points={{48,0},{42,0}}, color={28,108,200}));
  connect(hot_sink.C_in, T_drains_sensor.C_out)
    annotation (Line(points={{0,-51},{0,-40}}, color={28,108,200}));
  connect(T_drains_sensor.T_sensor, T_drains)
    annotation (Line(points={{10,-30},{26,-30}}, color={0,0,127}));
  connect(T_cold_sink_sensor.T_sensor, T_cold_sink) annotation (Line(points={{32,10},
          {32,14},{30,14},{30,22}},     color={0,0,127}));
  connect(P_cold_sink_sensor.P_sensor, P_cold_sink)
    annotation (Line(points={{58,10},{56,10},{56,24}}, color={0,0,127}));
  connect(reheater_NPP_drainsInlet.Kfr_cold, Kfr_cold)
    annotation (Line(points={{-22,4},{-36,4},{-36,16}}, color={0,0,127}));
  connect(reheater_NPP_drainsInlet.Kth_subc, Kth_subc)
    annotation (Line(points={{-12,10},{-12,30},{-18,30}}, color={0,0,127}));
  connect(reheater_NPP_drainsInlet.Kth_cond, Kth_cond) annotation (Line(points={
          {-11.8,-10},{-12,-10},{-12,-18},{-30,-18}}, color={0,0,127}));
  connect(reheater_NPP_drainsInlet.C_hot_out, T_drains_sensor.C_in) annotation
    (Line(points={{-4,-8},{-4,-14},{0,-14},{0,-20}}, color={28,108,200}));
  connect(reheater_NPP_drainsInlet.C_hot_in, Extr_source.C_out) annotation (
      Line(points={{-4,8},{-4,16},{0,16},{0,25}}, color={28,108,200}));
  connect(reheater_NPP_drainsInlet.C_cold_in, cold_source.C_out)
    annotation (Line(points={{-20.2,0},{-43,0}}, color={28,108,200}));
  connect(Drains_source.C_out, reheater_NPP_drainsInlet.C_drains_in)
    annotation (Line(points={{51,-28},{34,-28},{34,-14},{10.5,-14},{10.5,-6.5}},
        color={28,108,200}));
  connect(T_cold_sink_sensor.C_in, reheater_NPP_drainsInlet.C_cold_out)
    annotation (Line(points={{22,0},{12,0}}, color={28,108,200}));
end Reheater_reverse_connectors_DrainsInletSide;
