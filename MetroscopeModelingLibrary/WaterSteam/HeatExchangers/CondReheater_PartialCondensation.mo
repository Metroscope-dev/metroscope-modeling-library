within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model CondReheater_PartialCondensation
  replaceable package ColdMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  replaceable package HotMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;

  connector InputReal = input Real;
  connector InputCoefficientOfHeatTransfer = input
      Modelica.Units.SI.CoefficientOfHeatTransfer;
  connector InputArea = input Modelica.Units.SI.Area;

  InputReal Kfr_cold(start=1.e3) "Pressure loss coefficient";
  InputReal Kfr_hot(start=1.e3) "Pressure loss coefficient";
  InputCoefficientOfHeatTransfer Kth(start=9000)
    "heat transfer coefficient same in all zone";
  InputArea S_tot(start=100) "Total exchange area";
  Modelica.Units.SI.Power W_CondReH(start=40e6)
    "Energy transfer in CondVap zone";
  Modelica.Units.SI.Area S_CondReH(start=90) "Exchange area in CondVap zone";
  Modelica.Units.SI.Power W_DesHReH(start=0.5e6)
    "Energy transfer in CondVap zone";
  Modelica.Units.SI.Power Wtot(start=50e6) "Total energy transfered";
  //Real QCpMIN(start = 2.5e6);
  Modelica.Units.SI.SpecificEnthalpy hvsat_hot(start=2.7e6)
    "saturated liquid water (hot side) specific enthalpy at condensing inlet";
  Modelica.Units.SI.SpecificEnthalpy hlsat_hot(start=1.2e6)
    "saturated liquid water (hot side) specific enthalpy at condensing inlet";
  Modelica.Units.SI.MassFlowRate Q_cold(start=250) "Inlet Mass flow rate";
  Modelica.Units.SI.MassFlowRate Q_hot(start=50) "Inlet Mass flow rate";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_cold "Singular pressure loss";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_hot "Singular pressure loss";
  Modelica.Units.SI.Temperature T_hot_in "hot fluid temperature in K at inlet";
  Modelica.Units.SI.Temperature T_hot_out
    "hot fluid temperature in K at outlet";
  Modelica.Units.SI.Temperature T_cold_in "hot fluid temperature in K at inlet";
  Modelica.Units.SI.Temperature T_cold_out
    "hot fluid temperature in K at outlet";
  Modelica.Units.SI.MassFraction x_hot_out
    "hot fluid steam mass fraction at outlet";
  Common.Connectors.FluidInlet C_cold_in(redeclare package Medium = ColdMedium)
    annotation (Placement(transformation(extent={{210,-10},{230,10}}),
        iconTransformation(extent={{210,-10},{230,10}})));
  Common.Connectors.FluidOutlet C_cold_out(redeclare package Medium =
        ColdMedium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
        iconTransformation(extent={{-110,-10},{-90,10}})));
  replaceable Common.Partial.BasicTransportModel deheating_hot(redeclare
      package Medium = HotMedium)
    annotation (Placement(transformation(extent={{-52,28},{2,52}})));
  replaceable Common.Partial.BasicTransportModel deheating_cold(redeclare
      package Medium = ColdMedium)
    annotation (Placement(transformation(extent={{0,-52},{-52,-28}})));
  Common.Connectors.FluidInlet C_hot_in(redeclare package Medium = HotMedium)
    annotation (Placement(transformation(extent={{60,70},{80,90}}),
        iconTransformation(extent={{60,70},{80,90}})));
  Common.Connectors.FluidOutlet C_hot_out(redeclare package Medium = HotMedium)
    annotation (Placement(transformation(extent={{50,-90},{70,-70}}),
        iconTransformation(extent={{50,-90},{70,-70}})));
  replaceable Common.Partial.BasicTransportModel condensing_hot(redeclare
      package Medium = HotMedium)
    annotation (Placement(transformation(extent={{32,28},{82,52}})));
  replaceable Common.Partial.BasicTransportModel condensing_cold(redeclare
      package Medium = ColdMedium)
    annotation (Placement(transformation(extent={{78,-52},{26,-28}})));
equation
  /*===== Pressure loss ====*/
  // For convenience, the whole pressure loss in the component is modelized as only occuring in one zone.
  // The pressure loss coefficient, is however calculated for the whole system, for both sides.

  //in CondReH on cold side
  deltaP_cold =-Kfr_cold*
    MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_cold,
    condensing_cold.eps)/condensing_cold.rho_in;
  deltaP_cold =condensing_cold.P_out - condensing_cold.P_in;
  deheating_cold.P_in = deheating_cold.P_out;

  //DesHReH on hot side.
  deltaP_hot =-Kfr_hot*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(
    Q_hot, deheating_hot.eps)/deheating_hot.rho_in;
  deltaP_hot =deheating_hot.P_out - deheating_hot.P_in;
  condensing_hot.P_in = condensing_hot.P_out;

  /*==== Mass flows ====*/
  Q_cold =condensing_cold.Q_in;
                            // Mass flow on the cold side
  Q_hot =deheating_hot.Q_in;  // Mass flow on the hot side

  //mass balance equations for each basic transport element
  deheating_hot.Q_in + deheating_hot.Q_out = 0;
  condensing_hot.Q_in + condensing_hot.Q_out = 0;
  condensing_cold.Q_in + condensing_cold.Q_out = 0;
  deheating_cold.Q_in + deheating_cold.Q_out = 0;

  /*==== Energy balance ====*/
  /*----------------*/
  /*- CondReH zone-*/
  /*----------------*/
  //Condensation on hot side, reheating on cold side
  //Energy alance equations
  Q_hot*(condensing_hot.h_out - condensing_hot.h_in) = -W_CondReH;
  Q_cold*(condensing_cold.h_out - condensing_cold.h_in) = W_CondReH;
  //QCpMIN=Q_cold*ColdMedium.specificHeatCapacityCp(condensing_cold.state_in);
  //W_CondReH=(1 - exp(-Kth*S_CondReH/QCpMIN))*QCpMIN*(condensing_hot.T_in -
  //  condensing_cold.T_in);
  hlsat_hot=HotMedium.bubbleEnthalpy(HotMedium.setSat_p(condensing_hot.P_out));
  condensing_hot.h_out = hlsat_hot*(1-x_hot_out) + hvsat_hot*x_hot_out;

  W_CondReH = MetroscopeModelingLibrary.Common.Functions.PowerHeatExchange(Q_hot,Q_cold,
          0,ColdMedium.specificHeatCapacityCp(condensing_cold.state_in),
          condensing_hot.T_in,condensing_cold.T_in,Kth,S_tot,0.5); // the hot side heat capacity is useless for a two-phase flow

  /*----------------*/
  /*- DesHReH zone-*/
  /*----------------*/
  //Desuperheating on hot side, superheating on cold side
  //Energy alance equations
  Q_hot*(deheating_hot.h_out - deheating_hot.h_in) = -W_DesHReH;
  Q_cold*(deheating_cold.h_out - deheating_cold.h_in) = W_DesHReH;
  hvsat_hot=HotMedium.dewEnthalpy(HotMedium.setSat_p(deheating_hot.P_out));

  if noEvent(deheating_hot.h_in > hvsat_hot) then
    //Desuperheating is present
    deheating_hot.h_out = hvsat_hot;
  else
    //vaporization is absent
    deheating_hot.h_out = deheating_hot.h_in;
  end if;

  //Total heat power exchange
  Wtot = W_DesHReH + W_CondReH;
  //Surface repartition
  S_tot = S_CondReH;
  //calculation of temperautre at the boundaries
  T_hot_in=deheating_hot.T_in;
  T_hot_out=condensing_hot.T_out;
  T_cold_in=condensing_cold.T_in;
  T_cold_out=deheating_cold.T_out;
  connect(deheating_hot.C_out, condensing_hot.C_in)
    annotation (Line(points={{2.54,40},{32,40}}, color={63,81,181}));
  connect(deheating_hot.C_in, C_hot_in) annotation (Line(points={{-52,40},{-68,
          40},{-68,80},{70,80}}, color={63,81,181}));
  connect(condensing_cold.C_out, deheating_cold.C_in) annotation (Line(points={
          {25.48,-40},{14,-40},{14,-40},{0,-40}}, color={63,81,181}));
  connect(deheating_cold.C_out, C_cold_out) annotation (Line(points={{-52.52,-40},
          {-82,-40},{-82,0},{-100,0}}, color={63,81,181}));
  connect(condensing_cold.C_in, C_cold_in) annotation (Line(points={{78,-40},{
          112,-40},{112,0},{220,0}}, color={63,81,181}));
  connect(condensing_hot.C_out, C_hot_out) annotation (Line(points={{82.5,40},{
          146,40},{146,-80},{60,-80}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(extent={{-100,-80},{220,80}}),
                   graphics={
        Polygon(
          points={{220,80},{220,60},{220,-62.5},{220,-80},{180,-80},{50,-80},{-100,
              -80},{-100,80},{50,80},{180,80},{220,80}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{156,60},{156,38},{156,-42},{156,-62},{136,-62},{44,-62},{-84,
              -62},{-84,60},{48,60},{136,60},{156,60}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={205,225,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{178,42},{186,44},{186,-44},{178,-44},{168,-44},{54,-44},{-54,
              -44},{-56,42},{58,42},{170,42},{178,42}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{172,48},{178,48},{178,-50},{170,-50},{158,-50},{48,-50},{-74,
              -50},{-74,48},{48,48},{158,48},{172,48}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{166,30},{172,30},{172,-32},{164,-32},{152,-32},{38,-32},{-60,
              -32},{-58,28},{40,30},{152,30},{166,30}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{174,36},{180,36},{180,-36},{172,-38},{162,-38},{50,-38},{-60,
              -40},{-64,34},{48,36},{160,36},{174,36}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Rectangle(
          extent={{154,62},{204,-44}},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{156,58},{202,22}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=10),
        Rectangle(
          extent={{-24,17},{24,-17}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0,
          origin={178,31},
          rotation=0),
        Rectangle(
          extent={{154,58},{174,36}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0),
        Rectangle(
          extent={{20,23},{-20,-23}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=10,
          origin={177,-42},
          rotation=90),
        Rectangle(
          extent={{10.5,11.5},{-10.5,-11.5}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0,
          origin={164.5,-51.5},
          rotation=90),
        Rectangle(
          extent={{14,-23},{-14,23}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0,
          origin={177,-34},
          rotation=90)}), Diagram(coordinateSystem(extent={{-100,-80},{220,80}}),
        graphics={
        Rectangle(
          extent={{-74,60},{188,20}},
          lineColor={238,46,47},
          fillColor={191,0,3},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-74,-20},{188,-60}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{124,78},{180,64}},
          lineColor={238,46,47},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="Hot side"),
        Text(
          extent={{-52,-64},{-2,-80}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="Cold side"),
        Text(
          extent={{-52,6},{-22,-2}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="W_CondVap"),
        Line(points={{-20,-2},{-20,-18},{-22,-14}},
                                                  color={0,0,0}),
        Line(points={{-20,18},{-20,-18},{-18,-14}},
                                                 color={0,0,0}),
        Text(
          extent={{20,8},{52,-2}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="W_CondSupH"),
        Line(points={{56,-2},{56,-18},{54,-14}},  color={0,0,0}),
        Line(points={{56,18},{56,-18},{58,-14}}, color={0,0,0})}));
end CondReheater_PartialCondensation;
