within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model AirCooledCondenser_NTU
  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  package MoistAir = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  Inputs.InputArea S;
  Inputs.InputArea S_subc;

  Units.HeatExchangeCoefficient Kth;
  Units.HeatExchangeCoefficient Kth_subc;
  Inputs.InputFrictionCoefficient Kfr_hot;

  Units.Power W_cond;
  Units.Power W_subc;

  Units.VolumeFlowRate Qv_cold(start=Qv_cold_0);
  Units.MassFlowRate Q_cold(start=Q_cold_0);
  Units.MassFlowRate Q_hot(start=Q_hot_0);

  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  Units.Pressure P_tot(start=Psat_0, nominal=Psat_0);
  Units.Pressure Psat(start=Psat_0, nominal=Psat_0);
  Units.Temperature Tsat(start=Tsat_0);
  Units.Pressure P_incond(start=0.001e5);

  Inputs.InputReal C_incond(unit="mol/m3", min=0) "Incondensable molar concentration";
  Inputs.InputPressure P_offset(start=0) "Offset correction for ideal gas law";
  constant Real R(unit="J/(mol.K)") = Modelica.Constants.R "ideal gas constant";

    // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling(min = 0, max=100); // Fouling percentage
  Real air_intake(unit="mol/m3", min=0); // Air intake

  // Initialization parameters
  parameter Units.VolumeFlowRate Qv_cold_0 = 1800;
  parameter Units.MassFlowRate Q_cold_0 = 1800*1.292;
  parameter Units.MassFlowRate Q_hot_0 = 21;
  parameter Units.Pressure Psat_0 = 0.91e5;
  parameter Units.Pressure P_cold_in_0 = 1.002e5;
  parameter Units.Pressure P_cold_out_0 = 1.001e5;
  parameter Units.Temperature T_cold_in_0 = 273.15 + 15;
  parameter Units.Temperature T_cold_out_0 = 273.15 + 25;
  parameter Units.Temperature T_hot_in_0 = Tsat_0;
  parameter Units.Temperature T_hot_out_0 = Tsat_0;
  parameter Units.SpecificEnthalpy h_cold_in_0 = 0.5e5;
  parameter Units.SpecificEnthalpy h_cold_out_0 = 1e5;
  parameter Units.SpecificEnthalpy h_hot_in_0 = 2.4e6;
  parameter Units.SpecificEnthalpy h_liq_sat_0 = Water.bubbleEnthalpy(Water.setSat_p(Psat_0));
  parameter Units.Temperature Tsat_0 = Water.saturationTemperature(Psat_0);

  MetroscopeModelingLibrary.MoistAir.Connectors.Inlet
                            C_cold_in(Q(start=Q_cold_0)) annotation (
      Placement(transformation(extent={{-100,-10},{-80,10}}),iconTransformation(
          extent={{-100,-10},{-80,10}})));
  MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_hot_in(Q(start=
          Q_hot_0), P(start=Psat_0, nominal=Psat_0)) annotation (Placement(
        transformation(extent={{-10,-10},{10,10}},
        rotation=270,
        origin={2,90}),                              iconTransformation(extent={{-8,80},
            {12,100}})));
  MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_hot_out(Q(start=
          Q_cold_0), P(start=Psat_0)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={0,-90}),iconTransformation(extent={{-10,-80},{10,-60}})));
  MetroscopeModelingLibrary.MoistAir.Connectors.Outlet
                             C_cold_out(Q(start=-Q_cold_0), P(start=
          P_cold_out_0)) annotation (Placement(transformation(extent={{-10,-10},
            {10,10}},
        rotation=270,
        origin={88,0}),
                      iconTransformation(extent={{80,-10},{100,10}})));

  MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPFlowModel hot_side_condensing(
    T_in_0=Tsat_0,
    T_out_0=Tsat_0,
    h_in_0=h_hot_in_0,
    h_out_0=h_liq_sat_0,
    Q_0=Q_hot_0,
    P_0=Psat_0) annotation (Placement(transformation(
        extent={{17,-17},{-17,17}},
        rotation=180,
        origin={-17,29})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPFlowModel cold_side_condensing(
    T_in_0=T_cold_in_0,
    T_out_0=T_cold_out_0,
    h_in_0=h_cold_in_0,
    h_out_0=h_cold_out_0,
    Q_0=Q_cold_0,
    P_0=P_cold_out_0)
    annotation (Placement(transformation(extent={{-30,-24},{0,6}})));
  MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoHFlowModel incondensables_in(
    P_in_0=Psat_0,
    P_out_0=Psat_0,
    Q_0=Q_hot_0,
    T_0=Tsat_0,
    h_0=h_hot_in_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-54,50})));
  MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoHFlowModel incondensables_out(
    P_in_0=Psat_0,
    P_out_0=Psat_0,
    Q_0=Q_hot_0,
    T_0=Tsat_0,
    h_0=h_liq_sat_0) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-50})));

  MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe hot_side_pipe annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-54,76})));
  WaterSteam.BaseClasses.IsoPFlowModel hot_side_subcooling
    annotation (Placement(transformation(extent={{22,18},{46,42}})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPFlowModel
    cold_side_subcooling
    annotation (Placement(transformation(extent={{20,-20},{44,4}})));
  Power.HeatExchange.NTUHeatExchange heatExchange_condensing annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-14,10})));
  Power.HeatExchange.NTUHeatExchange heatExchange_subcooling annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={34,12})));
equation

  // Failure modes
  if not faulty then
    fouling = 0;
    air_intake=0;
  end if;

  // Definition

  Q_hot = hot_side_condensing.Q;
  Q_cold = cold_side_condensing.Q;
  Qv_cold = cold_side_condensing.Q/cold_side_condensing.rho;

  T_hot_in = hot_side_condensing.T_in;
  T_hot_out = hot_side_subcooling.T_out;
  T_cold_in = cold_side_condensing.T_in;
  T_cold_out = cold_side_subcooling.T_out;

  cold_side_condensing.W = W_cond;
  cold_side_subcooling.W = W_subc;

  P_tot = incondensables_in.P_in;

  // Incondensables
  P_incond = P_offset + R * (C_incond + air_intake) * Tsat;  // Ideal gaz law
  /* According to Dalton law, incondensable pressure is substracted first, 
  then water is condensed, then incondensable pressure is added again. */
  incondensables_in.DP = - P_incond;
  incondensables_out.DP = + P_incond;

  /* Condensation */

    // Energy balance
    hot_side_condensing.W + cold_side_condensing.W = 0;

    // Pressure losses
    hot_side_pipe.delta_z = 0;
    hot_side_pipe.Kfr = Kfr_hot;

    // Saturation
    Psat = hot_side_condensing.P_in;
    Tsat = hot_side_condensing.T_out;
    hot_side_condensing.h_out = Water.bubbleEnthalpy(Water.setSat_T(Tsat));

    // Heat Exchange
    heatExchange_condensing.config = "condenser";
    heatExchange_condensing.W = W_cond;
    heatExchange_condensing.S = S;
    heatExchange_condensing.Kth = Kth*(1 - fouling/100);
    heatExchange_condensing.T_cold_in = T_cold_in;
    heatExchange_condensing.T_hot_in = T_hot_in;
    heatExchange_condensing.Cp_cold = MoistAir.specificHeatCapacityCp(cold_side_condensing.state_in);
    heatExchange_condensing.Cp_hot = 0; // Not used by NTU method in condenser mode

  /* Subcooling */

    // Energy Balance
    hot_side_subcooling.W + cold_side_subcooling.W = 0;

    // Heat exchange
    heatExchange_subcooling.config = "monophasic_counter_current";
    heatExchange_subcooling.W = W_subc;
    heatExchange_subcooling.S = S_subc;
    heatExchange_subcooling.Kth = Kth_subc * (1-fouling/100);
    heatExchange_subcooling.T_cold_in = cold_side_subcooling.T_in;
    heatExchange_subcooling.T_hot_in = hot_side_subcooling.T_in;
    heatExchange_subcooling.Cp_cold = MoistAir.specificHeatCapacityCp(cold_side_subcooling.state_in);
    heatExchange_subcooling.Cp_hot = Water.specificHeatCapacityCp(hot_side_subcooling.state_in);

  connect(C_cold_out, C_cold_out)
    annotation (Line(points={{88,0},{88,0}},     color={28,108,200}));
  connect(C_hot_out, C_hot_out)
    annotation (Line(points={{0,-90},{0,-90}},
                                             color={28,108,200}));
  connect(C_cold_in, C_cold_in) annotation (Line(points={{-90,0},{-90,0}},
                                  color={85,170,255}));
  connect(C_cold_in, cold_side_condensing.C_in) annotation (Line(points={{-90,0},
          {-34,0},{-34,-9},{-30,-9}}, color={85,170,255}));
  connect(C_hot_in, hot_side_pipe.C_in)
    annotation (Line(points={{2,90},{-54,90},{-54,86}}, color={28,108,200}));
  connect(hot_side_pipe.C_out, incondensables_in.C_in)
    annotation (Line(points={{-54,66},{-54,60}}, color={28,108,200}));
  connect(incondensables_in.C_out, hot_side_condensing.C_in)
    annotation (Line(points={{-54,40},{-54,29},{-34,29}}, color={28,108,200}));
  connect(incondensables_out.C_out, C_hot_out)
    annotation (Line(points={{0,-60},{0,-90}}, color={28,108,200}));
  connect(hot_side_condensing.C_out, hot_side_subcooling.C_in) annotation (Line(
        points={{3.55271e-15,29},{22,29},{22,30}},         color={28,108,200}));
  connect(hot_side_subcooling.C_out, incondensables_out.C_in) annotation (Line(
        points={{46,30},{68,30},{68,-30},{1.77636e-15,-30},{1.77636e-15,-40}},
        color={28,108,200}));
  connect(cold_side_condensing.C_out, cold_side_subcooling.C_in) annotation (
      Line(points={{0,-9},{8,-9},{8,-8},{20,-8}},   color={85,170,255}));
  connect(cold_side_subcooling.C_out, C_cold_out) annotation (Line(points={{44,-8},
          {62,-8},{62,0},{88,0}},  color={85,170,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},
            {100,100}}),                                        graphics={
        Ellipse(
          extent={{-67,-81},{67,81}},
          lineColor={175,175,175},
          startAngle=0,
          endAngle=70,
          closure=EllipseClosure.None,
          origin={-107,-41},
          rotation=360,
          lineThickness=0.5),
        Ellipse(
          extent={{67,-81},{-67,81}},
          lineColor={175,175,175},
          startAngle=0,
          endAngle=70,
          closure=EllipseClosure.None,
          origin={107,-41},
          rotation=360,
          lineThickness=0.5),
        Ellipse(
          extent={{98,-130},{-98,130}},
          lineColor={175,175,175},
          startAngle=0,
          endAngle=70,
          closure=EllipseClosure.None,
          origin={98,-42},
          rotation=360,
          lineThickness=0.5),
        Ellipse(
          extent={{-98,-130},{98,130}},
          lineColor={175,175,175},
          startAngle=0,
          endAngle=70,
          closure=EllipseClosure.None,
          origin={-98,-42},
          rotation=360,
          lineThickness=0.5),
        Ellipse(
          extent={{-40,16},{-36,10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,28},{-36,22}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-16,62},{-12,56}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-28,42},{-24,36}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{28,30},{32,26}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{18,50},{16,46}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-12,52},{-8,46}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-66,-16},{-62,-22}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-74,-10},{-70,-16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{40,24},{44,18}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{56,4},{60,-2}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{58,-8},{62,-14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{64,-16},{68,-22}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{70,-10},{74,-16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-48,8},{-44,2}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-68,-4},{-64,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{44,10},{48,4}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{72,-18},{76,-24}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-90,-20},{-70,-40}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          startAngle=0,
          endAngle=360),
        Ellipse(
          extent={{-60,-4},{-56,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-58,8},{-54,2}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{0,-42},{-66,-50}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-88,-24},{-8,74}},
          color={28,108,200},
          thickness=1),
        Ellipse(
          extent={{70,-20},{90,-40}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          startAngle=0,
          endAngle=360),
        Ellipse(
          extent={{66,-42},{0,-50}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Line(
          points={{40,-49},{-40,49}},
          color={28,108,200},
          thickness=1,
          origin={48,25},
          rotation=180),
        Line(
          points={{-72,-36},{0,54}},
          color={28,108,200},
          thickness=1),
        Line(
          points={{34,-43},{-38,47}},
          color={28,108,200},
          thickness=1,
          origin={34,11},
          rotation=180),
        Ellipse(
          extent={{-10,78},{10,58}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          startAngle=0,
          endAngle=360),
        Ellipse(
          extent={{-2,64},{0,62}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(
          points={{0,54},{0,58}},
          color={28,108,200},
          thickness=1),
        Ellipse(
          extent={{-4,72},{-2,68}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{6,68},{2,70}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})));
end AirCooledCondenser_NTU;
