within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model LiqLiqHX
  package ColdMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  package HotMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;

  connector InputReal = input Real;
  connector InputArea = input Modelica.Units.SI.Area;
  connector InputCoefficientOfHeatTransfer = input
      Modelica.Units.SI.CoefficientOfHeatTransfer;

  InputArea S_tot(start=100) "Total exchange area";
  InputReal Kfr_cold(start=1.e3) "Pressure loss coefficient";
  InputReal Kfr_hot(start=1.e3) "Pressure loss coefficient";
  InputCoefficientOfHeatTransfer Kth(start=1000)
    "heat transfer coefficient";
  Modelica.Units.SI.Power W(start=100) "Energy transfer during condensation";
  Modelica.Units.SI.MassFlowRate Q_cold(start=500) "Inlet Mass flow rate";
  Modelica.Units.SI.MassFlowRate Q_hot(start=50) "Inlet Mass flow rate";
  Modelica.Units.SI.Temperature T_hot_in "hot fluid temperature in K at inlet";
  Modelica.Units.SI.Temperature T_hot_out
    "hot fluid temperature in deg_C at outlet";
  Modelica.Units.SI.Temperature T_cold_in
    "hot fluid temperature in deg_C at inlet";
  Modelica.Units.SI.Temperature T_cold_out
    "hot fluid temperature in deg_C at outlet";
  Common.Connectors.FluidInlet C_cold_in( redeclare package Medium =
        ColdMedium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}}),
        iconTransformation(extent={{-110,-10},{-90,10}})));
  Common.Connectors.FluidOutlet C_cold_out(redeclare package Medium =
        ColdMedium)
    annotation (Placement(transformation(extent={{210,-10},{230,10}}),
        iconTransformation(extent={{210,-10},{230,10}})));
  replaceable Common.Partial.IsoPFlowModel hotSide(redeclare package
      Medium = HotMedium)
    annotation (Placement(transformation(extent={{80,38},{38,62}})));
  replaceable Common.Partial.IsoPFlowModel coldSide(redeclare package
      Medium = ColdMedium)
    annotation (Placement(transformation(extent={{40,-56},{88,-32}})));
  Common.Connectors.FluidInlet C_hot_in(redeclare package Medium = HotMedium)
    annotation (Placement(transformation(extent={{50,70},{70,90}}),
        iconTransformation(extent={{50,70},{70,90}})));
  Common.Connectors.FluidOutlet C_hot_out(redeclare package Medium = HotMedium)
    annotation (Placement(transformation(extent={{50,-90},{70,-70}}),
        iconTransformation(extent={{50,-90},{70,-70}})));
  PressureLosses.SingularPressureLoss cold_PL
    annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  PressureLosses.SingularPressureLoss hot_PL
    annotation (Placement(transformation(extent={{90,70},{110,90}})));
equation

  // Pressure loss
  cold_PL.Kfr = Kfr_cold;
  hot_PL.Kfr = Kfr_hot;

  //deltaP_cold = cold_PL.P_out - cold_PL.P_in;
  //deltaP_hot = hot_PL.P_out - hot_PL.P_in;

  //Mass flows
  Q_cold =coldSide.Q_in;    // Mass flow on the cold side
  Q_hot =hotSide.Q_in;  // Mass flow on the hot side

  //Energy balance
  hotSide.W + coldSide.W = 0;

  // Heat exchange computation
  hotSide.W = W;
  W = MetroscopeModelingLibrary.Common.Functions.PowerHeatExchange(
    Q_hot,
    Q_cold,
    HotMedium.specificHeatCapacityCp(hotSide.state_in),
    ColdMedium.specificHeatCapacityCp(coldSide.state_in),
    hotSide.T_in,
    coldSide.T_in,
    Kth,
    S_tot,
    0);

  //calculation of temperature at the boundaries
  T_hot_in=hotSide.T_in;
  T_hot_out=hotSide.T_out;
  T_cold_in=coldSide.T_in;
  T_cold_out=coldSide.T_out;

  connect(hotSide.C_out, C_hot_out) annotation (Line(points={{37.58,50},{-6,50},
          {-6,-80},{60,-80}}, color={238,46,47}));
  connect(coldSide.C_out, C_cold_out) annotation (Line(points={{88.48,-44},{160,
          -44},{160,0},{220,0}}, color={63,81,181}));
  connect(cold_PL.C_in, C_cold_in)
    annotation (Line(points={{-70,0},{-100,0}}, color={63,81,181}));
  connect(cold_PL.C_out, coldSide.C_in) annotation (Line(points={{-49.8,0},{-24,
          0},{-24,-44},{40,-44}}, color={63,81,181}));
  connect(C_hot_in, hot_PL.C_in) annotation (Line(points={{60,80},{60,66},{84,66},
          {84,80},{90,80}}, color={63,81,181}));
  connect(hot_PL.C_out, hotSide.C_in) annotation (Line(points={{110.2,80},{124,80},
          {124,50},{80,50}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{220,80}}), graphics={
        Polygon(
          points={{-100,80},{-100,0},{-100,-80},{0,-80},{120,-80},{220,-80},{220,
              0},{220,80},{120,80},{0,80},{-100,80}},
          lineColor={28,108,200},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-64,62},{-64,40},{-64,-40},{-64,-60},{-44,-60},{48,-60},{154,
              -60},{176,-60},{176,-38},{176,43.0566},{176,62},{156,62},{44,62},{
              -44,62},{-64,62}},
          smooth=Smooth.Bezier,
          lineThickness=1,
          fillColor={35,138,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Rectangle(
          extent={{-70,46},{186,-46}},
          lineColor={28,108,200},
          lineThickness=1),
        Rectangle(
          extent={{-66,38},{184,-38}},
          lineColor={28,108,200},
          lineThickness=1),
        Rectangle(
          extent={{-70,30},{186,-30}},
          lineColor={28,108,200},
          lineThickness=1),
        Rectangle(
          extent={{176,52},{202,-54}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0),
        Rectangle(
          extent={{-88,54},{-64,-52}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={157,166,218},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0},
          radius=0)}),                                           Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{220,80}}),
                                                     graphics={
        Rectangle(
          extent={{-82,72},{190,28}},
          lineColor={238,46,47},
          fillColor={191,0,3},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-70,-24},{192,-68}},
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
          extent={{32,10},{66,4}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          textString="Wth"),
        Line(points={{60,-10},{60,-26},{58,-22}},    color={0,0,0}),
        Line(points={{60,38},{60,-26},{62,-22}},    color={0,0,0})}));
end LiqLiqHX;
