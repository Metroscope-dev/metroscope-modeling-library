within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model HXmoistAirWater
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  // Pressure Losses
  Inputs.InputFrictionCoefficient Kfr_cold;
  Inputs.InputFrictionCoefficient Kfr_hot;

  // Heating
  parameter String QCp_max_side = "cold";
  parameter String HX_config = "monophasic_cross_current";
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
  parameter Units.MassFlowRate Q_cold_0 = 500;
  parameter Units.MassFlowRate Q_hot_0 = 50;
  parameter Units.Temperature T_cold_in_0 = 8 + 273.15;
  parameter Units.Pressure P_cold_in_0 = 1 *1e5;

  MoistAir.Pipes.Pipe  hot_side_pipe(Q_0=Q_cold_0)
                                                  annotation (Placement(transformation(extent={{-50,-18},{-30,2}})));
  Power.HeatExchange.NTUHeatExchange HX(
    config=HX_config,
    QCp_max_side=QCp_max_side,
    T_cold_in_0=T_cold_in_0)                                                                                                  annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={2,12})));
  MoistAir.BaseClasses.IsoPFlowModel cold_side(Q_0=Q_hot_0) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={2,-8})));
  WaterSteam.BaseClasses.IsoPFlowModel hot_side(
    Q_0=Q_cold_0,
    T_in_0=T_cold_in_0,
    P_in_0=P_cold_in_0) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={2,34})));
  WaterSteam.Pipes.Pipe cold_side_pipe(Q_0=Q_cold_0, T_in_0=T_cold_in_0)  annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-16,-24})));
  MoistAir.Connectors.Inlet C_cold_in(Q(start=Q_cold_0)) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),iconTransformation(extent={{-110,-10},{-90,10}})));
  MoistAir.Connectors.Outlet C_cold_out(Q(start=Q_cold_0)) annotation (Placement(transformation(extent={{88,-10},{108,10}}),iconTransformation(extent={{90,-10},{110,10}})));
  WaterSteam.Connectors.Inlet C_hot_in(Q(start=Q_hot_0)) annotation (Placement(transformation(extent={{-10,70},{10,90}}), iconTransformation(extent={{-10,70},{10,90}})));
  WaterSteam.Connectors.Outlet C_hot_out(Q(start=Q_hot_0)) annotation (Placement(transformation(extent={{-10,-90},{10,-70}}), iconTransformation(extent={{-10,-90},{10,-70}})));
equation
      // Definitions
  Q_cold = cold_side.Q;
  Q_hot = hot_side.Q;
  T_cold_in = cold_side.T_in;
  T_cold_out = cold_side.T_out;
  T_hot_in = hot_side.T_in;
  T_hot_out = hot_side.T_out;
  hot_side.W = -W;

  // Energy balance
  cold_side.W + hot_side.W = 0;

  // Pressure losses
  cold_side_pipe.delta_z = 0;
  cold_side_pipe.Kfr = Kfr_cold;
  hot_side_pipe.delta_z = 0;
  hot_side_pipe.Kfr = Kfr_hot;

  // Power Exchange
  HX.W = W;
  HX.S = S;
  HX.Kth = Kth;
  HX.Q_cold = Q_cold;
  HX.Q_hot = Q_hot;
  HX.T_cold_in = T_cold_in;
  HX.T_hot_in = T_hot_in;
  HX.Cp_hot = WaterSteamMedium.specificHeatCapacityCp(hot_side.state_in);
  HX.Cp_cold = MoistAirMedium.specificHeatCapacityCp(cold_side.state_in);
  connect(C_hot_in, hot_side.C_in) annotation (Line(points={{0,80},{0,48},{16,48},{16,34},{12,34}}, color={28,108,200}));
  connect(hot_side.C_out, cold_side_pipe.C_in) annotation (Line(points={{-8,34},{-16,34},{-16,-14}}, color={28,108,200}));
  connect(cold_side_pipe.C_out, C_hot_out) annotation (Line(points={{-16,-34},{-16,-80},{0,-80}}, color={28,108,200}));
  connect(C_cold_in, hot_side_pipe.C_in) annotation (Line(points={{-100,0},{-54,0},{-54,-8},{-50,-8}},color={85,170,255}));
  connect(hot_side_pipe.C_out, cold_side.C_in) annotation (Line(points={{-30,-8},{-19,-8},{-19,-8},{-8,-8}}, color={85,170,255}));
  connect(cold_side.C_out, C_cold_out) annotation (Line(points={{12,-8},{54,-8},{54,0},{98,0}}, color={85,170,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,80},{100,-80}},
          lineColor={0,0,0},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid), Line(
          points={{78,8},{40,-60},{20,-60},{44,40},{24,54},{-22,-52},{-40,-38},{-20,65.631},{-40,66},{-80,-4}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier,
          rotation=-90,
          origin={0,-2})}),                                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end HXmoistAirWater;
