within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model LMTDFuelHeater

  extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  // Pressure Losses
  Inputs.InputFrictionCoefficient Kfr_cold;
  Inputs.InputFrictionCoefficient Kfr_hot;

  // Heating
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

  // Initialization parameters
  parameter Units.MassFlowRate Q_cold_0 = 12;
  parameter Units.MassFlowRate Q_hot_0 = 8.6;
  parameter Units.Temperature T_cold_in_0 = 29.5 + 273.15;
  parameter Units.Temperature T_cold_out_0 = 97 + 273.15;
  parameter Units.Temperature T_hot_in_0 = 230 +273.15;
  parameter Units.Temperature T_hot_out_0 = 200 +273.15;
  parameter Units.Pressure P_cold_in_0 = 29.7 *1e5;
  parameter Units.Pressure P_cold_out_0 = 29.7 *1e5;
  parameter Units.Pressure P_hot_in_0 = 47 *1e5;
  parameter Units.Pressure P_hot_out_0 = 47 *1e5;
  parameter Real h_hot_in_0 = 2e4;
  parameter Real h_hot_out_0 = 1e4;
  parameter Real h_cold_in_0 = 6e5;
  parameter Real h_cold_out_0 = 1e6;

  Power.HeatExchange.LMTDHeatExchange HX(
    T_cold_in_0=T_cold_in_0,T_hot_in_0=T_hot_in_0,T_cold_out_0=T_cold_out_0,T_hot_out_0=T_hot_out_0)                                                                                                  annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={10,14})));
  WaterSteam.BaseClasses.IsoPFlowModel hot_side(Q_0=Q_cold_0, P_0 = P_cold_out_0, h_in_0 = h_cold_in_0, h_out_0 = h_cold_out_0, T_in_0 = T_cold_in_0, T_out_0 = T_cold_out_0) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={10,28})));
  WaterSteam.Pipes.Pipe hot_side_pipe(Q_0=Q_hot_0, P_in_0 = P_hot_in_0, P_out_0 = P_hot_out_0, h_0 = h_hot_in_0, T_0 = T_hot_in_0) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-14,-24})));
  Fuel.Pipes.Pipe cold_side_pipe(Q_0=Q_cold_0, P_in_0 = P_cold_in_0, P_out_0 = P_cold_out_0, h_0 = h_cold_in_0, T_0 = T_cold_in_0) annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
  Fuel.BaseClasses.IsoPFlowModel cold_side(Q_0=Q_hot_0, P_0 = P_hot_out_0, h_in_0 = h_hot_in_0, h_out_0 = h_hot_out_0, T_in_0 = T_hot_in_0, T_out_0 = T_hot_out_0) annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  Fuel.Connectors.Inlet C_cold_in(Q(start=Q_cold_0), P(start=P_cold_in_0)) annotation (Placement(transformation(extent={{-80,-10},{-60,10}}), iconTransformation(extent={{-80,-10},{-60,10}})));
  Fuel.Connectors.Outlet C_cold_out(Q(start=-Q_cold_0), P(start=P_cold_out_0), h_outflow(start=h_cold_out_0)) annotation (Placement(transformation(extent={{60,-10},{80,10}}), iconTransformation(extent={{60,-10},{80,10}})));
  WaterSteam.Connectors.Inlet C_hot_in(Q(start=Q_hot_0), P(start=P_hot_in_0)) annotation (Placement(transformation(extent={{30,60},{50,80}}), iconTransformation(extent={{30,60},{50,80}})));
  WaterSteam.Connectors.Outlet C_hot_out(Q(start=Q_cold_0), P(start=P_hot_out_0), h_outflow(start=h_hot_out_0)) annotation (Placement(transformation(extent={{-50,-80},{-30,-60}}),
                                                                                                           iconTransformation(extent={{-50,-80},{-30,-60}})));
equation
    // Definitions
  Q_cold = cold_side.Q;
  Q_hot = hot_side.Q;
  T_cold_in = cold_side.T_in;
  T_cold_out = cold_side.T_out;
  T_hot_in = hot_side.T_in;
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
  HX.Kth = Kth;
  HX.T_cold_in = T_cold_in;
  HX.T_hot_in = T_hot_in;
  HX.T_cold_out = T_cold_out;
  HX.T_hot_out = T_hot_out;
  connect(hot_side_pipe.C_out,C_hot_out)  annotation (Line(points={{-14,-34},{-14,-70},{-40,-70}},       color={28,108,200}));
  connect(hot_side_pipe.C_in,hot_side. C_out) annotation (Line(points={{-14,-14},{-14,28},{0,28}},color={28,108,200}));
  connect(hot_side.C_in,C_hot_in)  annotation (Line(points={{20,28},{40,28},{40,70}}, color={28,108,200}));
  connect(cold_side_pipe.C_in,C_cold_in)  annotation (Line(points={{-52,0},{-70,0}}, color={213,213,0}));
  connect(cold_side_pipe.C_out,cold_side. C_in) annotation (Line(points={{-32,0},{0,0}}, color={213,213,0}));
  connect(cold_side.C_out,C_cold_out)  annotation (Line(points={{20,0},{70,0}}, color={213,213,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-70,50},{70,-50}},
          lineColor={0,0,0},
          fillColor={226,230,140},
          fillPattern=FillPattern.Solid), Line(
          points={{40,66},{40,-60},{20,-60},{20,64},{0,64},{0,-60},{-20,-60},{-20,65.6309},{-40,66},{-40,-66}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end LMTDFuelHeater;
