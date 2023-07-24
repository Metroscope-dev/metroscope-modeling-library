within MetroscopeModelingLibrary.Partial.HeatExchangers;
partial model WaterFlueGasesMonophasicHX

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Inputs.InputArea S;
  Inputs.InputHeatExchangeCoefficient Kth;
  Inputs.InputFrictionCoefficient Kfr_cold;
  Inputs.InputFrictionCoefficient Kfr_hot;
  Inputs.InputTemperature nominal_cold_side_temperature_rise; // water reference temperature rise based on H&MB diagramm values
  Inputs.InputTemperature nominal_hot_side_temperature_drop; // flue gases reference temperature rise based on H&MB diagramm values

  parameter String QCp_max_side = "hot";
  // Warning :
  // QCp_max_side = cold only for EC LP (aka condensate preheater)
  // Otherwise, flue gases usually correspond to QCp_max_side
  parameter String config = "monophasic_counter_current";
  parameter String mixed_fluid = "hot";

  Units.Power W;
  Units.MassFlowRate Q_cold(start=Q_cold_0);
  Units.MassFlowRate Q_hot(start=Q_hot_0);
  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  // Indicators
  Units.DifferentialTemperature DT_hot_in_side(start=T_hot_in_0-T_cold_out_0);
  Units.DifferentialTemperature DT_hot_out_side(start=T_hot_out_0-T_cold_in_0);
  Units.DifferentialTemperature pinch(start=min(T_hot_in_0-T_cold_out_0,T_hot_out_0-T_cold_in_0));

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling; // Fouling percentage

  // Initialization parameters (Values for an economizer)
  // Flow Rates
  parameter Units.MassFlowRate Q_cold_0 = 85;
  parameter Units.MassFlowRate Q_hot_0 = 640;
  // Temperatures
  parameter Units.Temperature T_cold_in_0 = 227 + 273.15;
  parameter Units.Temperature T_cold_out_0 = 270 + 273.15;
  parameter Units.Temperature T_hot_in_0 = 300 + 273.15;
  parameter Units.Temperature T_hot_out_0 = 273 + 273.15;
  // Pressures
  parameter Units.Pressure P_cold_in_0 = 170e5;
  parameter Units.Pressure P_cold_out_0 = 169.5e5;
  parameter Units.Pressure P_hot_in_0 = 1.1e5;
  parameter Units.Pressure P_hot_out_0 = 1.05e5;
  // Enthalpies
  parameter Units.SpecificEnthalpy h_cold_in_0 = 977378.7;
  parameter Units.SpecificEnthalpy h_cold_out_0 = 1185904.9;
  parameter Units.SpecificEnthalpy h_hot_in_0 = 6.08e5;
  parameter Units.SpecificEnthalpy h_hot_out_0 = 5.75e5;

  FlueGases.Connectors.Inlet C_hot_in(Q(start=Q_hot_0), P(start=P_hot_in_0)) annotation (Placement(transformation(
          extent={{-110,-10},{-90,10}}),iconTransformation(extent={{-110,-10},{-90,10}})));
  FlueGases.Connectors.Outlet C_hot_out(Q(start=-Q_hot_0), P(start=P_hot_out_0), h_outflow(start = h_hot_out_0)) annotation (Placement(transformation(
          extent={{90,-10},{110,10}}),iconTransformation(extent={{90,-10},{110,10}})));
  WaterSteam.Connectors.Inlet C_cold_in(Q(start=Q_cold_0), P(start=P_cold_in_0)) annotation (Placement(transformation(
          extent={{30,70},{50,90}}),   iconTransformation(extent={{30,70},{50,90}})));
  WaterSteam.Connectors.Outlet C_cold_out(Q(start=-Q_cold_0), P(start=P_cold_out_0), h_outflow(start= h_cold_out_0)) annotation (Placement(transformation(
          extent={{-50,72},{-30,92}}), iconTransformation(extent={{-50,70},{-30,90}})));
  FlueGases.Pipes.Pipe hot_side_pipe(Q_0=Q_hot_0, h_0=h_hot_in_0, P_in_0=P_hot_in_0, P_out_0=P_hot_out_0) annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  Power.HeatExchange.NTUHeatExchange HX(config=config, mixed_fluid=mixed_fluid, QCp_max_side=QCp_max_side,T_cold_in_0=T_cold_in_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={4,10})));
  FlueGases.BaseClasses.IsoPFlowModel hot_side(Q_0=Q_hot_0, h_in_0=h_hot_in_0, T_out_0=T_hot_out_0, P_0=P_hot_out_0, h_out_0=h_hot_out_0) annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={4,-6})));
  WaterSteam.BaseClasses.IsoPFlowModel cold_side(Q_0=Q_cold_0, h_in_0=h_cold_in_0, T_in_0=T_cold_in_0, P_0=P_cold_in_0, T_out_0=T_cold_out_0, h_out_0=h_cold_out_0) annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=0,
        origin={6,24})));
  WaterSteam.Pipes.Pipe cold_side_pipe(Q_0=Q_cold_0, h_0=h_cold_in_0, T_0=T_cold_in_0, P_in_0=P_cold_in_0, P_out_0=P_cold_out_0) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={30,42})));

  // Intermediate variables
protected
  MetroscopeModelingLibrary.Utilities.Units.HeatCapacity Cp_cold_min;
  MetroscopeModelingLibrary.Utilities.Units.HeatCapacity Cp_cold_max;
  MetroscopeModelingLibrary.Utilities.Units.HeatCapacity Cp_hot_min;
  MetroscopeModelingLibrary.Utilities.Units.HeatCapacity Cp_hot_max;
  MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium.ThermodynamicState state_cold_out; // estimation of the water outlet thermodynamic state
  MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium.ThermodynamicState state_hot_out; // estimation of the flue gases outlet thermodynamic state

equation
  // Failure modes
  if not faulty then
    fouling = 0;
  end if;

  // Definitions
  Q_cold =cold_side.Q;
  Q_hot =hot_side.Q;
  T_cold_in = cold_side_pipe.T_in;
  T_cold_out = cold_side.T_out;
  T_hot_in = hot_side_pipe.T_in;
  T_hot_out = hot_side.T_out;
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
  HX.Kth =  Kth*(1-fouling/100);
  HX.S = S;
  HX.Q_cold = Q_cold;
  HX.Q_hot = Q_hot;
  HX.T_cold_in = T_cold_in;
  HX.T_hot_in = T_hot_in;
//   HX.T_cold_out = T_cold_out;
//   HX.T_hot_out = T_hot_out;
  HX.Cp_cold = (Cp_cold_min + Cp_cold_max)/2;
  HX.Cp_hot = (Cp_hot_min + Cp_hot_max)/2;

  DT_hot_in_side = T_hot_in - T_cold_out;
  DT_hot_out_side = T_hot_out - T_cold_in;
  pinch = min(DT_hot_in_side, DT_hot_out_side);

  assert(pinch > 0, "A very low or negative pinch is reached", AssertionLevel.warning); // Ensure a positive pinch

//   HX.DT_hot_in_side = DT_hot_in_side;
//   HX.DT_hot_out_side = DT_hot_out_side;
//   HX.pinch = pinch;

  // For each medium, an average Cp is calculated beteween Cp inlet and an estimation of Cp outlet.
  // The estimation of the Cp outlet is calculated for an outlet temperature based on the nominal temperature rise of the H&MB diagram.
  // For more details about this hypothesis, please refer the Economiser page of the MML documentation.

  Cp_cold_min =MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium.specificHeatCapacityCp(cold_side.state_in); // water steam inlet Cp
  state_cold_out =MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium.setState_pTX(
    cold_side.P_in,
    cold_side.T_in + nominal_cold_side_temperature_rise,
    cold_side.Xi);
  Cp_cold_max=MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium.specificHeatCapacityCp(state_cold_out);// water steam outlet Cp

  Cp_hot_max=MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium.specificHeatCapacityCp(hot_side.state_in); // fg inlet Cp
  state_hot_out =MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium.setState_pTX(
    hot_side.P_in,
    hot_side.T_in - nominal_hot_side_temperature_drop,
    hot_side.Xi);
  Cp_hot_min =MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium.specificHeatCapacityCp(state_hot_out); // fg outlet Cp

  connect(cold_side.C_in, cold_side_pipe.C_out) annotation (Line(points={{16,24},{30,24},{30,32}}, color={28,108,200}));
  connect(cold_side_pipe.C_in, C_cold_in) annotation (Line(points={{30,52},{30,66},{30,80},{40,80}},
                                                                                     color={28,108,200}));
  connect(cold_side.C_out, C_cold_out) annotation (Line(points={{-4,24},{-18,24},{-18,22},{-40,22},{-40,82}}, color={28,108,200}));
  connect(hot_side.C_in, hot_side_pipe.C_out) annotation (Line(points={{-6,-6},{-22,-6},{-22,0},{-30,0}}, color={95,95,95}));
  connect(hot_side_pipe.C_in, C_hot_in) annotation (Line(points={{-50,0},{-100,0}},color={95,95,95}));
  connect(hot_side.C_out, C_hot_out) annotation (Line(points={{14,-6},{100,-6},{100,0}},
                                                                                       color={95,95,95}));
  connect(C_hot_out, C_hot_out) annotation (Line(points={{100,0},{62,0},{62,-6},{100,-6},{100,0}},
                                                                                                color={95,95,95}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid), Line(
          points={{40,80},{40,-80},{14,-80},{12,80},{-14,80},{-16,-80},{-40,-80},{-40,80}},
          color={0,0,0},
          smooth=Smooth.Bezier,
          thickness=1)}),          Diagram(coordinateSystem(preserveAspectRatio=false)));
end WaterFlueGasesMonophasicHX;
