within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model Superheater_PartialCondensation
  package ColdMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  package HotMedium =
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
  Modelica.Units.SI.Power W_CondVap(start=0.5e6)
    "Energy transfer in CondVap zone";
  Modelica.Units.SI.Power W_CondSupH(start=40e6)
    "Energy transfer in CondVap zone";
  Modelica.Units.SI.Power W_DesHSupH(start=0.5e6)
    "Energy transfer in CondVap zone";
  Modelica.Units.SI.Power Wtot(start=50e6) "Total energy transfered";
  Modelica.Units.SI.SpecificEnthalpy hvsat_cold(start=2.7e6)
    "saturated vapor (hot side) specific enthalpy at deheating outlet";
  Modelica.Units.SI.SpecificEnthalpy hvsat_hot(start=2.7e6)
    "saturated liquid water (hot side) specific enthalpy at condensing inlet";
  Modelica.Units.SI.SpecificEnthalpy hlsat_hot(start=1.2e6)
    "saturated liquid water (hot side) specific enthalpy at condensing inlet";
  Modelica.Units.SI.MassFlowRate Q_cold(start=250) "Inlet Mass flow rate";
  Modelica.Units.SI.MassFlowRate Q_hot_in(start=50)
    "Inlet Mass flow rate, hot side";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_cold "Singular pressure loss";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_hot "Singular pressure loss";
  Modelica.Units.SI.Temperature T_hot_in "hot fluid temperature in K at inlet";
  Modelica.Units.SI.Temperature T_hot_out
    "hot fluid temperature in K at outlet";
  Modelica.Units.SI.Temperature T_cold_in "hot fluid temperature in K at inlet";
  Modelica.Units.SI.Temperature T_cold_out
    "hot fluid temperature in K at outlet";
  Modelica.Units.SI.MassFlowRate Q_vent(start=5) "Vent Mass flow rate";
  Modelica.Units.SI.MassFlowRate Q_hot_out(start=45)
    "outlet Mass flow rate, hot side";
  Modelica.Units.SI.MassFraction x_hot_out
    "outlet desired steam fraction for the hot side";
  replaceable Common.Partial.BasicTransportModel vent(redeclare package Medium = HotMedium)
    annotation (Placement(transformation(extent={{-10.5,-5.5},{10.5,5.5}},
        rotation=-90,
        origin={176.5,0.5})));
  Common.Connectors.FluidOutlet C_vent_out(redeclare package Medium = HotMedium)
    annotation (Placement(transformation(extent={{210,-90},{230,-70}}),
        iconTransformation(extent={{210,-90},{230,-70}})));
  Common.Connectors.FluidInlet C_cold_in(redeclare package Medium = ColdMedium)
    annotation (Placement(transformation(extent={{50,-90},{70,-70}}),
        iconTransformation(extent={{50,-90},{70,-70}})));
  Common.Connectors.FluidOutlet C_cold_out(redeclare package Medium =
        ColdMedium)
    annotation (Placement(transformation(extent={{50,70},{70,90}}),
        iconTransformation(extent={{50,70},{70,90}})));
  replaceable Common.Partial.BasicTransportModel DesHSupH_hot(redeclare package
      Medium = HotMedium)
    annotation (Placement(transformation(extent={{-52,28},{2,52}})));
  replaceable Common.Partial.BasicTransportModel CondVap_hot(redeclare package
      Medium = HotMedium)
    annotation (Placement(transformation(extent={{110,28},{154,52}})));
  replaceable Common.Partial.BasicTransportModel DesHSupH_cold(redeclare
      package Medium = ColdMedium)
    annotation (Placement(transformation(extent={{0,-52},{-52,-28}})));
  replaceable Common.Partial.BasicTransportModel CondVap_cold(redeclare package
      Medium = ColdMedium)
    annotation (Placement(transformation(extent={{154,-52},{110,-28}})));
  Common.Connectors.FluidInlet C_hot_in(redeclare package Medium = HotMedium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
        iconTransformation(extent={{-110,-8},{-90,12}})));
  Common.Connectors.FluidOutlet C_hot_out(redeclare package Medium = HotMedium)
    annotation (Placement(transformation(extent={{212,-10},{232,10}}),
        iconTransformation(extent={{212,-10},{232,10}})));
  replaceable Common.Partial.BasicTransportModel CondSupH_hot(redeclare package
      Medium = HotMedium)
    annotation (Placement(transformation(extent={{32,28},{82,52}})));
  replaceable Common.Partial.BasicTransportModel CondSupH_cold(redeclare
      package Medium = ColdMedium)
    annotation (Placement(transformation(extent={{78,-52},{26,-28}})));
equation
  /*==== Vent ====*/
  vent.P_in=vent.P_out;
  Q_vent = vent.Q_in;
  vent.Q_in + vent.Q_out  = 0;
  vent.Q_in*vent.h_in + vent.Q_out*vent.h_out = 0;

  /*===== Pressure loss ====*/
  // For convenience, the whole pressure loss in the component is modelized as only occuring in one zone.
  // The pressure loss coefficient, is however calculated for the whole system, for both sides.
  //in CondVap on cold side
  deltaP_cold = -Kfr_cold*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_cold, CondVap_cold.eps)/CondVap_cold.rho_in;
  deltaP_cold = CondVap_cold.P_out - CondVap_cold.P_in;
  CondSupH_cold.P_in = CondSupH_cold.P_out;
  DesHSupH_cold.P_in = DesHSupH_cold.P_out;

  //in DesHSupH on hot side.
  deltaP_hot = -Kfr_hot*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_hot_in,DesHSupH_hot.eps)/DesHSupH_hot.rho_in;
  deltaP_hot = DesHSupH_hot.P_out - DesHSupH_hot.P_in;
  CondSupH_hot.P_in = CondSupH_hot.P_out;
  CondVap_hot.P_in = CondVap_hot.P_out;

  /*==== Mass flows ====*/
  Q_cold =CondVap_cold.Q_in;// Mass flow on the cold side
  Q_hot_in = DesHSupH_hot.Q_in;  // Mass flow at the inlet of the hot side
  Q_hot_out = Q_hot_in-Q_vent; // Mass flow on hot side at the outlet, taking into account the vent extraction

  //mass balance equations for each basic transport element
  DesHSupH_hot.Q_in + DesHSupH_hot.Q_out = 0;
  CondSupH_hot.Q_in + CondSupH_hot.Q_out  = 0;
  CondVap_hot.Q_in + CondVap_hot.Q_out = 0;
  CondVap_cold.Q_in + CondVap_cold.Q_out = 0;
  CondSupH_cold.Q_in + CondSupH_cold.Q_out  = 0;
  DesHSupH_cold.Q_in + DesHSupH_cold.Q_out = 0;

  /*==== Energy balance ====*/
  /*----------------*/
  /*- CondVap zone -*/
  /*----------------*/
  //Condensation on hot side, vaporization on cold side
  //Energy alance equations
  Q_hot_out*(CondVap_hot.h_out - CondVap_hot.h_in) =  -W_CondVap;
  Q_cold*(CondVap_cold.h_out - CondVap_cold.h_in) =  W_CondVap;
  ///Energy exchange equations
  //at the outlet of this zone, the hot medium has to terminate its condensation
  hlsat_hot=HotMedium.bubbleEnthalpy(HotMedium.setSat_p(CondVap_hot.P_out));
  /*We consider that on the cold side there is first pressure loss then heat exchange;
  According the medium state after pressure loss at given enthalpy, there will be vaporization or not*/
  hvsat_cold=ColdMedium.dewEnthalpy(ColdMedium.setSat_p(CondVap_cold.P_out));
  if noEvent(CondVap_cold.h_in < hvsat_cold) then
    //vaporization is present
    CondVap_cold.h_out=hvsat_cold;
  else
    //vaporization is absent
    W_CondVap = 0;
  end if;
  CondVap_hot.h_out=hlsat_hot*(1-x_hot_out) + x_hot_out*hvsat_hot;

  /*----------------*/
  /*- CondSupH zone-*/
  /*----------------*/
  //Condensation on hot side, superheating on cold side
  //Energy alance equations
  Q_hot_out*(CondSupH_hot.h_out - CondSupH_hot.h_in) = -W_CondSupH;
  Q_cold*(CondSupH_cold.h_out - CondSupH_cold.h_in)= W_CondSupH;
  W_CondSupH = MetroscopeModelingLibrary.Common.Functions.PowerHeatExchange(Q_hot_out,Q_cold,
          0,ColdMedium.specificHeatCapacityCp(CondSupH_cold.state_in),
          CondSupH_hot.T_in,CondSupH_cold.T_in,Kth,S_tot,0.5); // the hot side heat capacity is useless for a two-phase flow

  /*----------------*/
  /*- DesHSupH zone-*/
  /*----------------*/
  //Desuperheating on hot side, superheating on cold side
  //Energy alance equations
  Q_hot_in*(DesHSupH_hot.h_out -DesHSupH_hot.h_in) = -W_DesHSupH;
  Q_cold*(DesHSupH_cold.h_out -DesHSupH_cold.h_in)= W_DesHSupH;
  hvsat_hot=HotMedium.dewEnthalpy(HotMedium.setSat_p(DesHSupH_hot.P_out));
  if noEvent(DesHSupH_hot.h_in > hvsat_hot) then
    //Desuperheating is present
    DesHSupH_hot.h_out=hvsat_hot;
  else
    //vaporization is absent
    DesHSupH_hot.h_out = DesHSupH_hot.h_in;
  end if;

  //Total heat power exchange
  Wtot = W_DesHSupH + W_CondSupH + W_CondVap;

  //calculation of temperautre at the boundaries
  T_hot_in=DesHSupH_hot.T_in;
  T_hot_out=CondVap_hot.T_out;
  T_cold_in=CondVap_cold.T_in;
  T_cold_out=DesHSupH_cold.T_out;
  connect(vent.C_out, C_vent_out) annotation (Line(points={{176.5,-10.21},{176.5,
          -18},{196,-18},{196,-80},{220,-80}}, color={63,81,181}));
  connect(DesHSupH_hot.C_out, CondSupH_hot.C_in)
    annotation (Line(points={{2.54,40},{32,40}}, color={63,81,181}));
  connect(CondSupH_hot.C_out, CondVap_hot.C_in)
    annotation (Line(points={{82.5,40},{110,40}}, color={63,81,181}));
  connect(DesHSupH_hot.C_in, C_hot_in) annotation (Line(points={{-52,40},{-68,40},
          {-68,0},{-100,0}}, color={63,81,181}));
  connect(CondVap_hot.C_out, C_hot_out) annotation (Line(points={{154.44,40},{192,
          40},{192,0},{222,0}}, color={63,81,181}));
  connect(CondVap_cold.C_in, C_cold_in) annotation (Line(points={{154,-40},{172,
          -40},{172,-80},{60,-80}}, color={63,81,181}));
  connect(CondVap_cold.C_out, CondSupH_cold.C_in)
    annotation (Line(points={{109.56,-40},{78,-40}}, color={63,81,181}));
  connect(CondSupH_cold.C_out, DesHSupH_cold.C_in) annotation (Line(points={{25.48,
          -40},{14,-40},{14,-40},{0,-40}}, color={63,81,181}));
  connect(DesHSupH_cold.C_out, C_cold_out) annotation (Line(points={{-52.52,-40},
          {-82,-40},{-82,80},{60,80}}, color={63,81,181}));
  connect(DesHSupH_hot.C_out, vent.C_in) annotation (Line(points={{2.54,40},{20,
          40},{20,18},{176.5,18},{176.5,11}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(extent={{-100,-80},{220,80}}),
                   graphics={
        Polygon(
          points={{-100,80},{-100,60},{-100,-62.5},{-100,-80},{-60,-80},{70,-80},{220,-80},{220,80},{70,80},{-60,80},{-100,80}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-38,60},{-38,38},{-38,-42},{-38,-62},{-18,-62},{74,-62},{202,-62},{202,60},{70,60},{-18,60},{-38,60}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={205,225,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{-48,42},{-56,44},{-56,-44},{-48,-44},{-38,-44},{76,-44},{184,-44},{186,42},{72,42},{-40,42},{-48,42}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-54,48},{-60,48},{-60,-50},{-52,-50},{-40,-50},{70,-50},{192,-50},{192,48},{70,48},{-40,48},{-54,48}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-54,30},{-60,30},{-60,-32},{-52,-32},{-40,-32},{74,-32},{172,-32},{170,28},{72,30},{-40,30},{-54,30}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{-58,36},{-64,36},{-64,-36},{-56,-38},{-46,-38},{66,-38},{176,-40},{180,34},{68,36},{-44,36},{-58,36}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1),
        Rectangle(
          extent={{-88,52},{-38,-54}},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-88,60},{-42,24}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=10),
        Rectangle(
          extent={{-88,52},{-40,18}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0),
        Rectangle(
          extent={{-60,60},{-40,38}},
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
          origin={-63,-40},
          rotation=90),
        Rectangle(
          extent={{10.5,11.5},{-10.5,-11.5}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0,
          origin={-51.5,-49.5},
          rotation=90),
        Rectangle(
          extent={{14,23},{-14,-23}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0,
          origin={-63,-34},
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
        Line(points={{56,18},{56,-18},{58,-14}}, color={0,0,0}),
        Text(
          extent={{96,8},{128,-2}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="W_DesHSupH"),
        Line(points={{132,-2},{132,-18},{130,-14}},
                                                  color={0,0,0}),
        Line(points={{132,18},{132,-18},{134,-14}},
                                                 color={0,0,0})}));
end Superheater_PartialCondensation;
