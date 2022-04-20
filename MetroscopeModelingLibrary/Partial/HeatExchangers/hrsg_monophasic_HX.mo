within MetroscopeModelingLibrary.Partial.HeatExchangers;
partial model hrsg_monophasic_HX

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputArea S;
  Inputs.InputHeatExchangeCoefficient Kth;
  Inputs.InputFrictionCoefficient Kfr_cold;
  Inputs.InputFrictionCoefficient Kfr_hot;
  Inputs.InputTemperature T_cold_in;
  Inputs.InputTemperature T_hot_in;

  parameter String QCp_max_side = "hot";
  // Warning :
  // QCp_max_side = cold only for EC LP (aka condensate preheater)
  // Otherwise, flue gases usually correspond to QCp_max_side
  Units.Temperature nominal_cold_side_temperature_rise; // water reference temperature rise based on H&MB diagramm values
  Units.Temperature nominal_hot_side_temperature_rise; // flue gases reference temperature rise based on H&MB diagramm values
  Units.Power W;
  Units.MassFlowRate Q_cold;
  Units.MassFlowRate Q_hot;

    // Initialization parameters
  parameter Units.MassFlowRate Q_cold_0 = 500;
  parameter Units.MassFlowRate Q_hot_0 = 50;
  parameter Units.Temperature T_cold_in_0 = 76 + 273.15;
  parameter Units.Pressure P_cold_in_0 = 18 *1e5;


  FlueGases.Connectors.Inlet C_hot_in(Q(start=Q_hot_0)) annotation (Placement(transformation(
          extent={{-80,-10},{-60,10}}), iconTransformation(extent={{-80,-10},{-60,10}})));
  FlueGases.Connectors.Outlet C_hot_out(Q(start=Q_hot_0)) annotation (Placement(transformation(
          extent={{60,-10},{80,10}}), iconTransformation(extent={{60,-10},{80,10}})));
  WaterSteam.Connectors.Inlet C_cold_in(Q(start=Q_cold_0)) annotation (Placement(transformation(
          extent={{20,60},{40,80}}),   iconTransformation(extent={{20,60},{40,80}})));
  WaterSteam.Connectors.Outlet C_cold_out(Q(start=Q_cold_0)) annotation (Placement(transformation(
          extent={{-40,-80},{-20,-60}}),
                                       iconTransformation(extent={{-40,-80},{-20,-60}})));
  FlueGases.Pipes.Pipe hot_side_pipe(Q_0=Q_hot_0) annotation (Placement(transformation(extent={{-56,-10},{-36,10}})));
  Power.HeatExchange.NTUHeatExchange HX(config="monophasic_cross_current", QCp_max_side=QCp_max_side,T_cold_in_0=T_cold_in_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={10,-18})));
  FlueGases.BaseClasses.IsoPFlowModel hot_side(Q_0=Q_hot_0) annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-10,-20})));
  WaterSteam.BaseClasses.IsoPFlowModel cold_side(Q_0=Q_cold_0,T_in_0=T_cold_in_0,P_in_0=P_cold_in_0) annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={32,-16})));
  WaterSteam.Pipes.Pipe cold_side_pipe(Q_0=Q_cold_0,T_in_0=T_cold_in_0) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={32,10})));

  // Intermediate variables
protected
  MetroscopeModelingLibrary.Units.HeatCapacity Cp_cold_min;
  MetroscopeModelingLibrary.Units.HeatCapacity Cp_cold_max;
  MetroscopeModelingLibrary.Units.HeatCapacity Cp_hot_min;
  MetroscopeModelingLibrary.Units.HeatCapacity Cp_hot_max;
  MetroscopeModelingLibrary.Media.WaterSteamMedium.ThermodynamicState state_cold_out; // estimation of the water outlet thermodynamic state
  MetroscopeModelingLibrary.Media.FlueGasesMedium.ThermodynamicState state_hot_out; // estimation of the flue gases outlet thermodynamic state

equation
  // Definitions
  Q_cold =cold_side.Q_in;
  Q_hot =hot_side.Q_in;
  T_cold_in =cold_side.T_in;
  T_hot_in =hot_side.T_in;
  cold_side.W = W;

  // Energy balance
  hot_side.W + cold_side.W = 0;

  // Pressure losses
  cold_side_pipe.delta_z=0;
  cold_side_pipe.Kfr = Kfr_cold;
  hot_side_pipe.delta_z=0;
  hot_side_pipe.Kfr = Kfr_hot;

  // Power Exchange
  HX.W = W;
  HX.Kth =  Kth;
  HX.S = S;
  HX.Q_cold = Q_cold;
  HX.Q_hot = Q_hot;
  HX.T_cold_in = T_cold_in;
  HX.T_hot_in = T_hot_in;
  HX.Cp_cold = (Cp_cold_min + Cp_cold_max)/2;
  HX.Cp_hot = (Cp_hot_min + Cp_hot_max)/2;

  // For each medium, an average Cp is calculated beteween Cp inlet and an estimation of Cp outlet.
  // The estimation of the Cp outlet is calculated for an outlet temperature based on the nominal temperature rise of the H&MB diagram.
  // For more details about this hypothesis, please refer the Economiser page of the MML documentation.

  Cp_cold_min =MetroscopeModelingLibrary.Media.WaterSteamMedium.specificHeatCapacityCp(cold_side.state_in); // water steam inlet Cp
  state_cold_out = MetroscopeModelingLibrary.Media.WaterSteamMedium.setState_pTX(cold_side.P_in, cold_side.T_in + nominal_cold_side_temperature_rise,cold_side.Xi_in);
  Cp_cold_max= MetroscopeModelingLibrary.Media.WaterSteamMedium.specificHeatCapacityCp(state_cold_out); // water steam outlet Cp

  Cp_hot_max=MetroscopeModelingLibrary.Media.FlueGasesMedium.specificHeatCapacityCp(hot_side.state_in);// fg inlet Cp
  state_hot_out = MetroscopeModelingLibrary.Media.FlueGasesMedium.setState_pTX(hot_side.P_in, hot_side.T_in + nominal_hot_side_temperature_rise,hot_side.Xi_in);
  Cp_hot_min =MetroscopeModelingLibrary.Media.FlueGasesMedium.specificHeatCapacityCp(state_hot_out);  // fg outlet Cp



  connect(cold_side.C_out, C_cold_out)
    annotation (Line(points={{32,-26},{32,-70},{-30,-70}},
                                                        color={28,108,200}));
  connect(hot_side.C_out, C_hot_out) annotation (Line(points={{-10,-30},{-10,-32},{70,-32},{70,0}},
                       color={95,95,95}));
  connect(C_hot_in,hot_side_pipe. C_in)
    annotation (Line(points={{-70,0},{-56,0}},           color={95,95,95}));
  connect(hot_side_pipe.C_out,hot_side. C_in)
    annotation (Line(points={{-36,0},{-10,0},{-10,-10}},color={95,95,95}));
  connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(points={{32,20},{32,70},{30,70}}, color={28,108,200}));
  connect(cold_side_pipe.C_out, cold_side.C_in) annotation (Line(points={{32,0},{32,-6}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-68,50},{70,-50}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid), Line(
          points={{30,70},{30,-76},{0,-76},{0,72},{-30,72},{-30,-72}},
          color={0,0,0},
          thickness=1,
          smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
end hrsg_monophasic_HX;
