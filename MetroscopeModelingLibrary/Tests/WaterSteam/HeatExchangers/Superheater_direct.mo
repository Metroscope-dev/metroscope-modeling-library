within MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers;
model Superheater_direct

  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.WaterSteamTestIcon;

    // Boundary conditions
  input Real P_hot_steam(start=60, min=0, nominal=11) "bar";
  input Real P_cold_steam(start=11, min=0, nominal=50) "bar";
  input Utilities.Units.PositiveMassFlowRate Q_cold(start=1300) "kg/s";
  input Real h_cold_steam(start=2.75e6) "J/kg"; // slightly humid cold steam
  input Real h_hot_steam(start=2.8e6) "J/kg"; // slightly superheated hot steam

  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_steam_source annotation (Placement(transformation(extent={{-72,-10},
            {-52,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink drains_sink annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_steam_source annotation (Placement(transformation(extent={{-42,-50},{-22,-30}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink superheated_steam_sink annotation (Placement(transformation(extent={{16,70},
            {36,90}})));
  .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Superheater superheater(
      input_specs=true)
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink vent_sink annotation (Placement(transformation(extent={{70,-30},{90,-10}})));
  Utilities.Interfaces.RealOutput Kth_sup annotation (Placement(transformation(
          extent={{10,20},{18,28}}),   iconTransformation(extent={{-192,-82},{-172,
            -62}})));
  Utilities.Interfaces.RealOutput Kth_evap annotation (Placement(transformation(
          extent={{-22,22},{-14,30}}),
                                     iconTransformation(extent={{-192,-82},{-172,
            -62}})));
  Sensors_Control.WaterSteam.TemperatureSensor T_superheated_steam_sensor(
    sensor_function="Calibration",
    causality="Kth_sup",
    T_start=224,
    signal_unit="degC",
    display_unit="degC") annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,54})));
  Utilities.Interfaces.RealOutput T_superheated_steam annotation (Placement(
        transformation(extent={{-30,50},{-22,58}}), iconTransformation(extent={
            {-192,-82},{-172,-62}})));
equation

  // Boundary conditions
  hot_steam_source.P_out = P_hot_steam*1e5;
  hot_steam_source.h_out = h_hot_steam;
  cold_steam_source.P_out = P_cold_steam*1e5;
  cold_steam_source.h_out = h_cold_steam;
  cold_steam_source.Q_out = - Q_cold;

  // Specifications, for if "input_specs" is set to true
  superheater.S_evap = 10;
  superheater.S_sup = 100;
  superheater.Q_vent = 1;

  // Calibrated parameters
  Kth_sup = 7e3;

  connect(cold_steam_source.C_out,superheater. C_cold_in)
    annotation (Line(points={{-27,-40},{0,-40},{0,-8}}, color={28,108,200}));
  connect(hot_steam_source.C_out,superheater. C_hot_in) annotation (Line(points={{-57,0},
          {-16,0}},                                 color={28,108,200}));
  connect(vent_sink.C_in, superheater.C_vent) annotation (Line(points={{75,-20},
          {20,-20},{20,-7.8},{16,-7.8}},
                               color={28,108,200}));
  connect(drains_sink.C_in, superheater.C_hot_out)
    annotation (Line(points={{75,0},{16,0}}, color={28,108,200}));
  connect(superheater.Kth_evap, Kth_evap)
    annotation (Line(points={{-8,10},{-8,26},{-18,26}},
                                                     color={0,0,127}));
  connect(T_superheated_steam_sensor.C_in, superheater.C_cold_out)
    annotation (Line(points={{0,44},{0,8}}, color={28,108,200}));
  connect(T_superheated_steam_sensor.C_out, superheated_steam_sink.C_in)
    annotation (Line(points={{0,64},{0,80},{21,80}}, color={28,108,200}));
  connect(superheater.Kth_sup, Kth_sup)
    annotation (Line(points={{6,10},{6,24},{14,24}}, color={0,0,127}));
  connect(T_superheated_steam_sensor.T_sensor, T_superheated_steam)
    annotation (Line(points={{-10,54},{-26,54}}, color={0,0,127}));
end Superheater_direct;
