within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model Reheater
  replaceable package ColdMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  replaceable package HotMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  Modelica.SIunits.Area S_tot( start=1000) "Total exchange area";
  Real Level( start=0.2) "in [0;1], is an image of the set water level";
  Real Level_0( start=0.2) "same as Level but >0 (min = 1e-3)";
  Real Kfr_cold(start=1.e3) "Pressure loss coefficient";
  Real Kfr_hot(start=1.e3) "Pressure loss coefficient";
  Modelica.SIunits.CoefficientOfHeatTransfer Kth_cond(start = 2000) "heat transfer coefficient in the condensation zone";
  Modelica.SIunits.CoefficientOfHeatTransfer Kth_purge(start = 1000) "heat transfer coefficient in the purge zone";
  Modelica.SIunits.Power Wcond(start=50e6) "Energy transfer during condensation";
  Modelica.SIunits.Area S_cond(start = 500) "Exchange area in the condensation zone";
  Modelica.SIunits.Power Wdeh(start=10e6) "Energy transfer during deheating";
  Modelica.SIunits.Area S_deh(start = 250) "Exchange area in the deheating zone";
  Modelica.SIunits.CoefficientOfHeatTransfer Kth_deh(start = 1000) "heat transfer coefficient in the deheating zone";
  Modelica.SIunits.Power Wpurge(start=10e6) "Energy transfer during drain cooling";
  Modelica.SIunits.Area S_purge(start = 250) "Exchange area in the purge zone";
  Modelica.SIunits.Power Wtot(start = 70e6) "Total energy transfered";
  Modelica.SIunits.SpecificEnthalpy hvsat_hot_deh_out( start = 1e6) "saturated vapor (hot side) specific enthalpy at deheating outlet";
  Modelica.SIunits.SpecificEnthalpy hlsat_hot_cond_in( start = 1e6) "saturated liquid water (hot side) specific enthalpy at condensing inlet";
  Modelica.SIunits.MassFlowRate Q_cold(start=500) "Inlet Mass flow rate";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_cold "Singular pressure loss";
  Modelica.SIunits.MassFlowRate Q_hot(start=50) "Inlet Mass flow rate";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_hot "Singular pressure loss";
  Modelica.SIunits.Density coldSide_rhom;
  Modelica.SIunits.Density hotSide_rhom;
  Modelica.SIunits.Temperature T_hot_in "hot fluid temperature in K at inlet";
  Modelica.SIunits.Temperature T_hot_out "hot fluid temperature in deg_C at outlet";
  Modelica.SIunits.Temperature T_cold_in "hot fluid temperature in deg_C at inlet";
  Modelica.SIunits.Temperature T_cold_out "hot fluid temperature in deg_C at outlet";
  replaceable Common.Partial.BasicTransportModel deheating_hot(C_out(
                                                               h_vol(      start=2500e3)),
                                                                                         redeclare
      package Medium =                                                                                              HotMedium)
    annotation (Placement(transformation(extent={{150,40},{104,64}})));
  Common.Connectors.FluidInlet C_cold_in( redeclare package Medium =
        ColdMedium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
        iconTransformation(extent={{-110,-10},{-90,10}})));
  Common.Connectors.FluidOutlet C_cold_out(redeclare package Medium =
        ColdMedium)
    annotation (Placement(transformation(extent={{210,-10},{230,10}}),
        iconTransformation(extent={{210,-10},{230,10}})));
  replaceable Common.Partial.BasicTransportModel condensing_hot(redeclare
      package Medium =                                                                     HotMedium)
    annotation (Placement(transformation(extent={{70,40},{16,64}})));
  replaceable Common.Partial.BasicTransportModel purge_hot(redeclare package
      Medium =                                                                        HotMedium)
    annotation (Placement(transformation(extent={{-10,40},{-52,64}})));
  replaceable Common.Partial.BasicTransportModel deheating_cold(h_in(start=500e3),redeclare
      package Medium =                                                                                       ColdMedium)
    annotation (Placement(transformation(extent={{100,-58},{150,-34}})));
  replaceable Common.Partial.BasicTransportModel condensing_cold(redeclare
      package Medium =                                                                      ColdMedium)
    annotation (Placement(transformation(extent={{24,-58},{76,-34}})));
  replaceable Common.Partial.BasicTransportModel purge_cold(redeclare package
      Medium =                                                                         ColdMedium)
    annotation (Placement(transformation(extent={{-46,-58},{2,-34}})));
  Common.Connectors.FluidInlet C_hot_in(redeclare package Medium = HotMedium)
    annotation (Placement(transformation(extent={{58,74},{78,94}}),
        iconTransformation(extent={{50,68},{70,88}})));
  Common.Connectors.FluidOutlet C_hot_out(redeclare package Medium = HotMedium)
    annotation (Placement(transformation(extent={{50,-90},{70,-70}}),
        iconTransformation(extent={{50,-90},{70,-70}})));
equation
  // Pressure loss
  // For convenience, the whole pressure loss in the component is modelized as only occuring in the deheating part.
  // The pressure loss coefficient, is however calculated for the whole system, for both sides.
  coldSide_rhom = (purge_cold.rho_in + deheating_cold.rho_out)/2;
  hotSide_rhom = (deheating_hot.rho_in + purge_hot.rho_out)/2;
  deltaP_cold = Kfr_cold*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_cold, deheating_cold.eps)/coldSide_rhom;
  deltaP_hot = Kfr_hot*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_hot,deheating_hot.eps)/hotSide_rhom;
  deltaP_cold = deheating_cold.P_in - deheating_cold.P_out;
  deltaP_hot = deheating_hot.P_in - deheating_hot.P_out;
  condensing_hot.P_in = condensing_hot.P_out;
  condensing_cold.P_in = condensing_cold.P_out;
  purge_hot.P_in = purge_hot.P_out;
  purge_cold.P_in = purge_cold.P_out;

  //Mass flows
  Q_cold =purge_cold.Q_in;  // Mass flow on the cold side
  Q_hot =deheating_hot.Q_in;  // Mass flow on the hot side

  //mass balance equations for each basic transport element
  deheating_cold.Q_in +deheating_cold.Q_out  = 0;
  condensing_cold.Q_in +condensing_cold.Q_out  = 0;
  purge_cold.Q_in +purge_cold.Q_out  = 0;
  deheating_hot.Q_in +deheating_hot.Q_out  = 0;
  condensing_hot.Q_in +condensing_hot.Q_out  = 0;
  purge_hot.Q_in +purge_hot.Q_out  = 0;


  //Energy balance
  //Verification of level>0 to avoid impossible calculation of Kth_purge, in case of inverse model
  if Level>0 then
    Level_0=Level;
  else
    Level_0=1e-3;
  end if;
  /*Exchange area calculation*/
  S_tot=S_deh+S_cond+S_purge;
  Level_0=S_purge/S_tot;

  /*----------------*/
  /*-Deheating zone-*/
  /*----------------*/
  deheating_hot.Q_in*deheating_hot.h_in + deheating_hot.Q_out*deheating_hot.h_out = Wdeh;
  deheating_cold.Q_in*deheating_cold.h_in + deheating_cold.Q_out*deheating_cold.h_out = -Wdeh;
  /*As it is done in TSP, K_deh and K_cond are supposed be associated such as*/
  Kth_deh=Kth_cond/2;

  /*We consider that on the hot side there is first pressure loss then heat exchange;
  following the enthalpy at the outlet (after pressure loss), there will be deheating or not*/
  hvsat_hot_deh_out=HotMedium.dewEnthalpy(HotMedium.setSat_p(deheating_hot.P_out));
  if noEvent(deheating_hot.h_in > hvsat_hot_deh_out) then
    //deheating is present
    Wdeh = MetroscopeModelingLibrary.Common.Functions.PowerHeatExchange(Q_hot,Q_cold,
          HotMedium.specificHeatCapacityCp(deheating_hot.state_in),ColdMedium.specificHeatCapacityCp(deheating_cold.state_in),
          deheating_hot.T_in,deheating_cold.T_in,Kth_deh,S_deh,1);
    deheating_hot.h_out=hvsat_hot_deh_out;
  else
    //deheating is absent
    deheating_hot.h_out=deheating_hot.h_in;
    S_deh=0;
  end if;

  /*-------------------*/
  /*-Condensation zone-*/
  /*-------------------*/
  condensing_hot.Q_in*condensing_hot.h_in + condensing_hot.Q_out*condensing_hot.h_out = Wcond;
  condensing_cold.Q_in*condensing_cold.h_in + condensing_cold.Q_out*condensing_cold.h_out = -Wcond;
  /*following the enthalpy at the inlet, there will be condensation or not*/
  hlsat_hot_cond_in=HotMedium.bubbleEnthalpy(HotMedium.setSat_p(condensing_hot.P_in));
  if noEvent(condensing_hot.h_in > hlsat_hot_cond_in) then
    //condensation is present
    Wcond = MetroscopeModelingLibrary.Common.Functions.PowerHeatExchange(Q_hot,Q_cold,
          HotMedium.specificHeatCapacityCp(condensing_hot.state_in),ColdMedium.specificHeatCapacityCp(condensing_cold.state_in),
          condensing_hot.T_in,condensing_cold.T_in,Kth_cond,S_cond,0.5);
    condensing_hot.h_out=hlsat_hot_cond_in;
  else
    //condensation is absent
    condensing_hot.h_out=condensing_hot.h_in;
    S_cond=0;
  end if;

  /*----------------*/
  /*-subcooled zone-*/
  /*----------------*/
  purge_hot.Q_in*purge_hot.h_in + purge_hot.Q_out*purge_hot.h_out = Wpurge;
  purge_cold.Q_in*purge_cold.h_in + purge_cold.Q_out*purge_cold.h_out = -Wpurge;
  Wpurge = MetroscopeModelingLibrary.Common.Functions.PowerHeatExchange(Q_hot,Q_cold,
          HotMedium.specificHeatCapacityCp(purge_hot.state_in),ColdMedium.specificHeatCapacityCp(purge_cold.state_in),
          purge_hot.T_in,purge_cold.T_in,Kth_purge,S_purge,0);


  //Total heat power exchange
  Wtot = Wdeh + Wcond + Wpurge;
  //calculation of temperautre at the boundaries
  T_hot_in=deheating_hot.T_in;
  T_hot_out=purge_hot.T_out;
  T_cold_in=purge_cold.T_in;
  T_cold_out=deheating_cold.T_out;


  connect(condensing_hot.C_out,purge_hot. C_in)
    annotation (Line(points={{15.46,52},{-10,52}},                  color={238,46,47}));
  connect(deheating_hot.C_out,condensing_hot. C_in)
    annotation (Line(points={{103.54,52},{70,52}},                      color={238,46,47}));
  connect(C_cold_in, purge_cold.C_in)
    annotation (Line(points={{-100,0},{-86,0},{-86,-46},{-46,-46}},
                                                                  color={0,0,255}));
  connect(condensing_cold.C_in, purge_cold.C_out)
    annotation (Line(points={{24,-46},{2.48,-46}},                    color={0,0,255}));
  connect(deheating_cold.C_in, condensing_cold.C_out)
    annotation (Line(points={{100,-46},{76.52,-46}},                      color={0,0,255}));
  connect(deheating_cold.C_out, C_cold_out) annotation (Line(points={{150.5,-46},
          {188,-46},{188,0},{220,0}},
                           color={238,46,47}));
  connect(purge_hot.C_out, C_hot_out) annotation (Line(points={{-52.42,52},{-78,
          52},{-78,-80},{60,-80}},  color={238,46,47}));
  connect(deheating_hot.C_in, C_hot_in) annotation (Line(points={{150,52},{190,52},{190,84},{68,84}},
                                  color={0,0,255}));
  connect(C_hot_in, C_hot_in) annotation (Line(points={{68,84},{68,84}}, color={0,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{220,80}}), graphics={
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
          points={{-66,-18},{-78,-18},{-78,-48},{-76,-60},{-20,-62},{72,-62},{182,-62},{204,-18},{186,-18},{70,-20},{-52,-18},{-66,-18}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={35,138,255},
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
          extent={{-90,50},{-40,-66}},
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
          rotation=90)}),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{220,80}}),
                                                     graphics={
        Rectangle(
          extent={{-78,72},{194,28}},
          lineColor={238,46,47},
          fillColor={191,0,3},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-74,-24},{188,-68}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-36,86},{20,72}},
          lineColor={238,46,47},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="Hot side"),
        Text(
          extent={{126,-66},{176,-82}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="Cold side"),
        Text(
          extent={{102,4},{136,-2}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="Wdeh"),
        Text(
          extent={{18,6},{52,0}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="Wcond"),
        Text(
          extent={{-54,8},{-20,2}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="Wpurge"),
        Line(points={{128,-14},{128,-30},{130,-26}}, color={0,0,0}),
        Line(points={{128,36},{128,-30},{126,-26}}, color={0,0,0}),
        Line(points={{46,-14},{46,-30},{44,-26}}, color={0,0,0}),
        Line(points={{46,38},{46,-30},{48,-26}}, color={0,0,0}),
        Line(points={{-26,-12},{-26,-28},{-28,-24}}, color={0,0,0}),
        Line(points={{-26,36},{-26,-28},{-24,-24}}, color={0,0,0})}));
end Reheater;
