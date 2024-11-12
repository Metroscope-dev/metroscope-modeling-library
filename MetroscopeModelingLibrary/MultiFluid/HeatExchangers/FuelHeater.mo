within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model FuelHeater
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // Pressure Losses
  Inputs.InputFrictionCoefficient Kfr_cold;
  Inputs.InputFrictionCoefficient Kfr_hot;

  // Cp estimation temperatures: estimated temperature differences for both the hot and cold fluids
  parameter Boolean nominal_DT_default = true;
  Units.Temperature maximum_achiveable_temperature_difference;
  Units.Temperature nominal_cold_side_temperature_rise;
  Units.Temperature nominal_hot_side_temperature_drop;

  // Heating
  parameter String QCp_max_side = "undefined";// On fuel heater, QCp_hot may be close to QCp_cold
  parameter String HX_config = "monophasic_counter_current";
  Inputs.InputArea S;
  Inputs.InputHeatExchangeCoefficient Kth;
  Units.Power W;

  // Definitions
  Units.MassFlowRate Q_cold(start=Q_cold_0);
  Units.MassFlowRate Q_hot(start=Q_hot_0);
  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  // Indicators
  Units.DifferentialTemperature DT_hot_in_side(start=T_hot_in_0-T_cold_out_0) "Temperature difference between hot and cold fluids, hot inlet side";
  Units.DifferentialTemperature DT_hot_out_side(start=T_hot_out_0-T_cold_in_0) "Temperature difference between hot and cold fluids, hot outlet side";
  Units.DifferentialTemperature pinch(start=min(T_hot_in_0-T_cold_out_0,T_hot_out_0-T_cold_in_0)) "Lowest temperature difference";

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling(min = 0, max=100); // Fouling percentage

  // Initialization parameters
  // Flow Rates
  parameter Units.MassFlowRate Q_cold_0 = 15;
  parameter Units.MassFlowRate Q_hot_0 = 11;
  // Temperatures
  parameter Units.Temperature T_cold_in_0 = 8 + 273.15;
  parameter Units.Temperature T_cold_out_0 = 200 + 273.15;
  parameter Units.Temperature T_hot_in_0 = 210 + 273.15;
  parameter Units.Temperature T_hot_out_0 = 68 + 273.15;
  // Pressures
  parameter Units.Pressure P_cold_in_0 = 30e5;
  parameter Units.Pressure P_cold_out_0 = 29.5e5;
  parameter Units.Pressure P_hot_in_0 = 18.5e5;
  parameter Units.Pressure P_hot_out_0 = 18.3e5;
  // Enthalpies
  parameter Units.SpecificEnthalpy h_cold_in_0 = 554708.5;
  parameter Units.SpecificEnthalpy h_cold_out_0 = 292266.22;
  parameter Units.SpecificEnthalpy h_hot_in_0 = 958265.3;
  parameter Units.SpecificEnthalpy h_hot_out_0 = 5.75e5;

  Units.HeatCapacity Cp_cold_min;
  Units.HeatCapacity Cp_cold_max;
  Units.HeatCapacity Cp_hot_min;
  Units.HeatCapacity Cp_hot_max;
  FuelMedium.ThermodynamicState state_cold_out; // estimation of the water outlet thermodynamic state
  WaterSteamMedium.ThermodynamicState state_hot_out; // estimation of the flue gases outlet thermodynamic state

  Fuel.Connectors.Inlet C_cold_in(Q(start=Q_cold_0), P(start=P_cold_in_0)) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),iconTransformation(extent={{-110,-10},{-90,10}})));
  Fuel.Connectors.Outlet C_cold_out(Q(start=-Q_cold_0), P(start=P_cold_out_0), h_outflow(start= h_cold_out_0)) annotation (Placement(transformation(extent={{90,-10},{110,10}}),iconTransformation(extent={{90,-10},{110,10}})));
  WaterSteam.Connectors.Inlet C_hot_in(Q(start=Q_hot_0), P(start=P_hot_in_0)) annotation (Placement(transformation(extent={{30,70},{50,90}}), iconTransformation(extent={{30,70},{50,90}})));
  WaterSteam.Connectors.Outlet C_hot_out(Q(start=-Q_hot_0), P(start=P_hot_out_0), h_outflow(start = h_hot_out_0)) annotation (Placement(transformation(extent={{-50,-90},{-30,-70}}),
                                                                                                           iconTransformation(extent={{-50,-90},{-30,-70}})));
  Power.HeatExchange.NTUHeatExchange HX(config=HX_config, QCp_max_side=QCp_max_side,T_cold_in_0=T_cold_in_0, T_hot_in_0=T_hot_in_0) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={10,14})));
  WaterSteam.BaseClasses.IsoPFlowModel hot_side(Q_0=Q_hot_0, h_in_0=h_hot_in_0, T_out_0=T_hot_out_0, P_0=P_hot_out_0, h_out_0=h_hot_out_0) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={10,28})));
  Fuel.Pipes.Pipe cold_side_pipe(Q_0=Q_cold_0, h_0=h_cold_in_0, T_0=T_cold_in_0, P_in_0=P_cold_in_0, P_out_0=P_cold_out_0) annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  Fuel.BaseClasses.IsoPFlowModel cold_side(Q_0=Q_cold_0, h_in_0=h_cold_in_0, T_in_0=T_cold_in_0, P_0=P_cold_in_0, T_out_0=T_cold_out_0, h_out_0=h_cold_out_0) annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  WaterSteam.Pipes.Pipe hot_side_pipe(Q_0=Q_hot_0, h_0=h_hot_in_0, P_in_0=P_hot_in_0, P_out_0=P_hot_out_0) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={40,44})));

equation

  // Failure modes
  if not faulty then
    fouling = 0;
  end if;

    // Definitions
  Q_cold = cold_side.Q;
  Q_hot = hot_side.Q;
  T_cold_in = cold_side_pipe.T_in;
  T_cold_out = cold_side.T_out;
  T_hot_in = hot_side_pipe.T_in;
  T_hot_out = hot_side.T_out;
  cold_side.W = W;

  // Energy balance
  hot_side.W + cold_side.W = 0;

  // Pressure losses
  cold_side_pipe.delta_z = 0;
  cold_side_pipe.Kfr = Kfr_cold;
  hot_side_pipe.delta_z = 0;
  hot_side_pipe.Kfr = Kfr_hot;

  // Power Exchange
  HX.W = W;
  HX.S = S;
  HX.Kth = Kth*(1-fouling/100);
  HX.Q_cold = Q_cold;
  HX.Q_hot = Q_hot;
  HX.T_cold_in = T_cold_in;
  HX.T_hot_in = T_hot_in;
  HX.Cp_cold = (Cp_cold_min + Cp_cold_max)/2;
  HX.Cp_hot = (Cp_hot_min + Cp_hot_max)/2;

  // Indicators
  DT_hot_in_side = T_hot_in - T_cold_out;
  DT_hot_out_side = T_hot_out - T_cold_in;
  pinch = min(DT_hot_in_side, DT_hot_out_side);
  assert(pinch > 0, "A negative pinch is reached", AssertionLevel.warning); // Ensure a positive pinch
  assert(pinch > 1 or pinch < 0,  "A very low pinch (<1) is reached", AssertionLevel.warning); // Ensure a sufficient pinch

  // Nominal temperature differences
  // The default temperature rise and drop are equal to the maximum achievable temperature difference
  // Maximum achievable temperature difference for both the hot and cold sides = hot_side.T_in - cold_side.T_in
  maximum_achiveable_temperature_difference = hot_side.T_in - cold_side.T_in;
  if nominal_DT_default then
    nominal_cold_side_temperature_rise = maximum_achiveable_temperature_difference;
    nominal_hot_side_temperature_drop = maximum_achiveable_temperature_difference;
  end if;

  Cp_cold_min = FuelMedium.specificHeatCapacityCp(cold_side.state_in); // fuel steam inlet Cp
  state_cold_out = FuelMedium.setState_pTX(cold_side.P_in, cold_side.T_in + nominal_cold_side_temperature_rise,cold_side.Xi);
  Cp_cold_max= FuelMedium.specificHeatCapacityCp(state_cold_out); // fuel steam outlet Cp

  Cp_hot_max=WaterSteamMedium.specificHeatCapacityCp(hot_side.state_in);// fg inlet Cp
  state_hot_out = WaterSteamMedium.setState_pTX(hot_side.P_in, hot_side.T_in - nominal_hot_side_temperature_drop,hot_side.Xi);
  Cp_hot_min =WaterSteamMedium.specificHeatCapacityCp(state_hot_out);  // fg outlet Cp
  connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(points={{-52,0},{-100,0}},color={213,213,0}));
  connect(cold_side_pipe.C_out, cold_side.C_in) annotation (Line(points={{-32,0},{0,0}}, color={213,213,0}));
  connect(cold_side.C_out, C_cold_out) annotation (Line(points={{20,0},{100,0}},color={213,213,0}));
  connect(hot_side.C_in, hot_side_pipe.C_out) annotation (Line(points={{20,28},{40,28},{40,34}}, color={28,108,200}));
  connect(hot_side_pipe.C_in, C_hot_in) annotation (Line(points={{40,54},{40,80}}, color={28,108,200}));
  connect(hot_side.C_out, C_hot_out) annotation (Line(points={{0,28},{-20,28},{-20,-80},{-40,-80}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={0,0,0},
          fillColor={226,230,140},
          fillPattern=FillPattern.Solid), Line(
          points={{40,80},{40,-80},{20,-80},{20,80},{0,80},{0,-80},{-20,-80},{-20,80},{-40,80},{-40,-80}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Line(points={{122,-56}}, color={102,44,145})}),
                          Diagram(coordinateSystem(preserveAspectRatio=false)));
end FuelHeater;
