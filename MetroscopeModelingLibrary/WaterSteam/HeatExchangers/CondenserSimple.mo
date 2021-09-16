within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model CondenserSimple
  replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  Modelica.Units.SI.CoefficientOfHeatTransfer Kth(start=100)
    "Global heat transfer coefficient (active if exchanger_type=3)";
  Modelica.Units.SI.Area S(start=10);
  Modelica.Units.SI.Power W(start=1e8);
  Modelica.Units.SI.MassFlowRate Q_top_up(start=0) "Inlet Mass flow rate";
  Modelica.Units.SI.Temperature Tsat(start=10 + 273.15);
  Modelica.Units.SI.AbsolutePressure P_hot_in(start=0.051e5) "Steam pressure at the condenser inlet";
  Modelica.Units.SI.AbsolutePressure Psat(start=0.05e5);
  Modelica.Units.SI.Height WaterHeight(start=2);
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_cold(start=1e5) "Singular pressure loss";
  Real Kfr_cold(start=10) "Friction pressure loss coefficient";
  Modelica.Units.SI.AbsolutePressure P_incond(start=0.001e5);
  Real C_incond "Incondensable molar concentration";
  Modelica.Units.SI.AbsolutePressure P_offset "Offset correction for ideal gaz law";
  Modelica.Units.SI.MassFlowRate Q_cold(start=4500) "Cold water massflow rate";
  Modelica.Units.SI.VolumeFlowRate Qv_cold_in(start=4.5) "cold water volumic flow rate at the inlet";
  Modelica.Units.SI.Temperature T_cold_in "cold water temperature at the condenser inlet";
  Modelica.Units.SI.Temperature T_cold_out "cold water temperature at the condenser outlet";
  constant Real R=Modelica.Constants.R "ideal gas constant";

  parameter Boolean mass_balance = true;
  Common.Connectors.FluidInlet C_hot_in( redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{-10,92},{10,112}})));
  Common.Connectors.FluidOutlet C_hot_out(redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{-10,-96},{10,-76}}),
        iconTransformation(extent={{-10,-96},{10,-76}})));
  replaceable Common.Partial.BasicTransportModel coldSide(redeclare package
      Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(
        extent={{-25,-18},{25,18}},
        rotation=0,
        origin={45,0})));
  Common.Connectors.FluidInlet C_cold_in(redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{-112,42},{-92,62}}),
        iconTransformation(extent={{-112,42},{-92,62}})));
  Common.Connectors.FluidOutlet C_cold_out(redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{90,-16},{110,4}}),
        iconTransformation(extent={{90,-16},{110,4}})));
  replaceable Common.Partial.BasicTransportModel hotSide(redeclare package
      Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(
        extent={{-31.5,-17.5},{31.5,17.5}},
        rotation=-90,
        origin={-30.5,6.5})));
  PressureLosses.PipePressureLoss waterHeightPressureLoss annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-32,-86})));
  replaceable Common.Partial.BasicTransportModel incond_in(redeclare package
      Medium =
        WaterSteamMedium) annotation (Placement(
        transformation(
        extent={{-13,-5},{13,5}},
        rotation=270,
        origin={-31,67})));
  replaceable Common.Partial.BasicTransportModel incond_out(redeclare package
      Medium = WaterSteamMedium)
                          annotation (Placement(
        transformation(
        extent={{-13,-6},{13,6}},
        rotation=270,
        origin={-32,-51})));
equation

  /***** INCONDENSABLES *****/

  P_incond = P_offset + R * C_incond * Tsat;  // Ideal gaz law

  /* According to Dalton law, incondensable pressure is substracted first, 
  then water is condensed, then incondensable pressure is added again. */

  incond_in.Q_in + incond_in.Q_out = 0;
  incond_in.Q_in*incond_in.h_in + incond_in.Q_out*incond_in.h_out = 0;
  P_incond = incond_in.P_in - incond_in.P_out; // Dalton law

  incond_out.Q_in + incond_out.Q_out = 0;
  incond_out.Q_in*incond_out.h_in + incond_out.Q_out*incond_out.h_out = 0;
  P_incond = incond_out.P_out - incond_out.P_in; // Dalton law
  P_hot_in=incond_in.P_in;


  /***** CONDENSATION *****/

  // HOT SIDE
  hotSide.Q_in + hotSide.Q_out + Q_top_up = 0; // Mass balance
  if mass_balance then
    hotSide.Q_in+hotSide.Q_out = 0;
  end if;
  hotSide.Q_in*hotSide.h_in + hotSide.Q_out*hotSide.h_out = W; // Energy balance
  Psat = hotSide.P_in;
  Tsat = hotSide.T_in;
  hotSide.h_out = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_T(Tsat)); // Saturation at end of condensation (use of Tsat instead of Psat for stability)
  hotSide.P_out = hotSide.P_in; // Saturation at both ends

  // COLD SIDE
  Q_cold=coldSide.Q_in;
  Qv_cold_in=coldSide.Qv_in;
  T_cold_in=coldSide.T_in;
  T_cold_out=coldSide.T_out;

  coldSide.Q_in + coldSide.Q_out = 0; // Mass balance
  coldSide.Q_in*coldSide.h_in + coldSide.Q_out*coldSide.h_out = -W; // Energy balance
  deltaP_cold = coldSide.P_in - coldSide.P_out; // Pressure loss
  deltaP_cold = Kfr_cold*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_cold, coldSide.eps)/coldSide.rho_in; // Pressure Loss

  // HEAT EXCHANGE
  0 = Tsat - T_cold_out - (Tsat - T_cold_in)*exp(Kth*S*((T_cold_in - T_cold_out)/W));


  /***** WaterHeight *****/
  waterHeightPressureLoss.Kfr = 0;
  waterHeightPressureLoss.z1 = WaterHeight;
  waterHeightPressureLoss.z2  = 0;


  connect(coldSide.C_in, C_cold_in) annotation (Line(points={{20,0},{-40,0},{
          -40,52},{-102,52}},                                                   color={0,0,255}));
  connect(coldSide.C_out, C_cold_out)
    annotation (Line(points={{70.5,0},{86,0},{86,-6},{100,-6}},
                                                color={238,46,47}));
  connect(waterHeightPressureLoss.C_out, C_hot_out) annotation (Line(points={{-32,
          -96.2},{-32,-106},{0,-106},{0,-86}},   color={63,81,181}));
  connect(incond_in.C_in, C_hot_in) annotation (Line(points={{-31,80},{-31,88},{
          0,88},{0,102}}, color={63,81,181}));
  connect(incond_in.C_out, hotSide.C_in) annotation (Line(points={{-31,53.74},{-30.5,
          38}},                    color={63,81,181}));
  connect(incond_out.C_out, waterHeightPressureLoss.C_in)
    annotation (Line(points={{-32,-64.26},{-32,-76}}, color={63,81,181}));
  connect(incond_out.C_in, hotSide.C_out) annotation (Line(points={{-32,-38},{-30.5,
          -38},{-30.5,-25.63}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -80},{100,100}}),                                   graphics={
        Rectangle(
          extent={{-100,100},{100,-20}},
          pattern=LinePattern.None,
          lineThickness=1,
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,0}),
        Ellipse(
          extent={{-40,34},{-36,28}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-86},{100,-20}},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Polygon(
          points={{100,-86},{100,100},{-100,100},{-100,-86},{100,-86}},
          lineColor={0,0,255}),
        Ellipse(
          extent={{-62,44},{-58,38}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,58},{-36,52}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-20,46},{-16,40}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-8,62},{-4,56}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-20,58},{-16,52}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,76},{-36,70}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{10,46},{14,40}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{20,64},{16,56}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{32,54},{36,48}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{40,68},{44,62}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{48,48},{52,42}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-16,76},{-12,70}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{4,78},{8,72}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{24,84},{28,78}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{50,78},{54,72}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-56,64},{-52,58}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-60,80},{-56,74}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-60,-4},{-56,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-42,-10},{-38,-16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-24,-2},{-20,-8}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-4,-8},{0,-14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{14,-4},{18,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{32,-6},{36,-12}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{50,-4},{54,-10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{64,-6},{68,-12}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{32,36},{36,30}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{64,48},{68,42}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-4,38},{0,32}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-18,26},{-14,20}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{20,22},{24,16}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-2,16},{2,10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-38,12},{-34,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-58,20},{-54,14}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{42,16},{46,10}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{54,24},{58,18}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,12},{64,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{10,12},{14,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-78,64},{-74,58}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-74,40},{-70,32}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-76,12},{-72,6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-70,0},{-66,-6}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,62},{64,56}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{70,74},{74,68}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-74,76},{-70,70}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-100,50},{82,50},{82,36},{-88,36},{-88,22},{82,22},{82,8},{
              -88,8},{-88,-8},{100,-8}},
          color={0,0,255},
          thickness=1)}),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{100,100}})));
end CondenserSimple;
