within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model FuelHeater
  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // Pressure Losses
  Inputs.InputFrictionCoefficient Kfr_cold;
  Inputs.InputFrictionCoefficient Kfr_hot;
  Inputs.InputTemperature nominal_cold_side_temperature_rise; // water reference temperature rise based on H&MB diagramm values
  Inputs.InputTemperature nominal_hot_side_temperature_drop; // flue gases reference temperature rise based on H&MB diagramm values

  // Heating
  parameter String QCp_max_side = "undefined";// On fuel heater, QCp_hot may be close to QCp_cold
  parameter String HX_config = "monophasic_counter_current";
  Inputs.InputArea S;
  Inputs.InputHeatExchangeCoefficient Kth;
  Units.Power W;

  // Definitions
  Units.MassFlowRate Q_cold;
  Units.MassFlowRate Q_hot;
  Units.Temperature T_cold_in;
  Units.Temperature T_cold_out;
  Units.Temperature T_hot_in;
  Units.Temperature T_hot_out;

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling(min = 0, max=100); // Fouling percentage

  // Initialization parameters
  parameter Units.MassFlowRate Q_cold_0 = 500;
  parameter Units.MassFlowRate Q_hot_0 = 50;
  parameter Units.Temperature T_hot_in_0 = 76 + 273.15;
  parameter Units.Pressure P_hot_in_0 = 18 *1e5;
  parameter Real h_hot_in_0 = 1e6;

  Fuel.Connectors.Inlet C_cold_in annotation (Placement(transformation(extent={{-80,-10},{-60,10}}), iconTransformation(extent={{-80,-10},{-60,10}})));
  Fuel.Connectors.Outlet C_cold_out annotation (Placement(transformation(extent={{60,-10},{80,10}}), iconTransformation(extent={{60,-10},{80,10}})));
  WaterSteam.Connectors.Inlet C_hot_in annotation (Placement(transformation(extent={{30,60},{50,80}}), iconTransformation(extent={{30,60},{50,80}})));
  WaterSteam.Connectors.Outlet C_hot_out(h_outflow(start=h_hot_in_0)) annotation (Placement(transformation(extent={{-50,-80},{-30,-60}}),
                                                                                                           iconTransformation(extent={{-50,-80},{-30,-60}})));
  Power.HeatExchange.NTUHeatExchange HX(
    config=HX_config,
    QCp_max_side=QCp_max_side,
    T_hot_in_0=T_hot_in_0)                                                                                                  annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={10,14})));
  WaterSteam.BaseClasses.IsoPFlowModel hot_side(
    Q_0=Q_hot_0,
    T_in_0=T_hot_in_0,
    P_in_0=P_hot_in_0,
    h_in_0=h_hot_in_0) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={10,28})));
  Fuel.Pipes.Pipe cold_side_pipe annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  Fuel.BaseClasses.IsoPFlowModel cold_side annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  WaterSteam.Pipes.Pipe hot_side_pipe(Q_0=Q_hot_0, T_in_0=T_hot_in_0, h_in_0=h_hot_in_0) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={40,44})));


protected
  Units.HeatCapacity Cp_cold_min;
  Units.HeatCapacity Cp_cold_max;
  Units.HeatCapacity Cp_hot_min;
  Units.HeatCapacity Cp_hot_max;
  FuelMedium.ThermodynamicState state_cold_out; // estimation of the water outlet thermodynamic state
  WaterSteamMedium.ThermodynamicState state_hot_out; // estimation of the flue gases outlet thermodynamic state


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



  Cp_cold_min = FuelMedium.specificHeatCapacityCp(cold_side.state_in); // fuel steam inlet Cp
  state_cold_out = FuelMedium.setState_pTX(cold_side.P_in, cold_side.T_in + nominal_cold_side_temperature_rise,cold_side.Xi);
  Cp_cold_max= FuelMedium.specificHeatCapacityCp(state_cold_out); // fuel steam outlet Cp

  Cp_hot_max=WaterSteamMedium.specificHeatCapacityCp(hot_side.state_in);// fg inlet Cp
  state_hot_out = WaterSteamMedium.setState_pTX(hot_side.P_in, hot_side.T_in + nominal_hot_side_temperature_drop,hot_side.Xi);
  Cp_hot_min =WaterSteamMedium.specificHeatCapacityCp(state_hot_out);  // fg outlet Cp
  connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(points={{-52,0},{-70,0}}, color={213,213,0}));
  connect(cold_side_pipe.C_out, cold_side.C_in) annotation (Line(points={{-32,0},{0,0}}, color={213,213,0}));
  connect(cold_side.C_out, C_cold_out) annotation (Line(points={{20,0},{70,0}}, color={213,213,0}));
  connect(hot_side.C_in, hot_side_pipe.C_out) annotation (Line(points={{20,28},{40,28},{40,34}}, color={28,108,200}));
  connect(hot_side_pipe.C_in, C_hot_in) annotation (Line(points={{40,54},{40,70}}, color={28,108,200}));
  connect(hot_side.C_out, C_hot_out) annotation (Line(points={{0,28},{-20,28},{-20,-70},{-40,-70}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-70,50},{70,-50}},
          lineColor={0,0,0},
          fillColor={226,230,140},
          fillPattern=FillPattern.Solid), Line(
          points={{40,66},{40,-60},{20,-60},{20,64},{0,64},{0,-60},{-20,-60},{-20,65.6309},{-40,66},{-40,-66}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Line(points={{122,-56}}, color={102,44,145})}),
                          Diagram(coordinateSystem(preserveAspectRatio=false)));
end FuelHeater;
