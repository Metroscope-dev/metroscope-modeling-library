within MetroscopeModelingLibrary.WaterSteam.HeatExchangers;
model CondenserSimple_wIncondensable
  replaceable package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  Modelica.Units.SI.CoefficientOfHeatTransfer Kth(start=100)
    "Global heat transfer coefficient (active if exchanger_type=3)";
  Modelica.Units.SI.Area S(start=10);
  Modelica.Units.SI.Power W(start=1e8);
  Modelica.Units.SI.MassFlowRate Q_top_up(start=0) "Inlet Mass flow rate";
  Modelica.Units.SI.Temperature Tsat(start=10 + 273.15);
  Modelica.Units.SI.AbsolutePressure Psat(start=0.05e5);
  Modelica.Units.SI.Height WaterHeight(start=2);
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_cold(start=1e5) "Singular pressure loss";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_hot(start=1e5) "Singular pressure loss";
  Modelica.Units.SI.Density rho_liquid_hot(start=1000);
  Real Kfr_cold(start=10) "Friction pressure loss coefficient";
  Modelica.Units.SI.AbsolutePressure P_incond(start=0) "partial pressure of incondensable gas";
  Modelica.Units.SI.AbsolutePressure Ptot(start=0.05e5) "Total pressure inside the condenser, as it is measured";
  Modelica.Units.SI.Temperature Tcold_in(start=15 + 273.15) "Water temperature at the cold side inlet";
  Modelica.Units.SI.Temperature Tcold_out(start=25 + 273.15) "Water temperature at the cold side outlet";


  constant Modelica.Units.SI.Acceleration g=Modelica.Constants.g_n
    "Gravity constant";
  parameter Boolean mass_balance = true;
 Modelica.Units.SI.AbsolutePressure P_offset(start=0e5) "possible necessary constant to make compatible P_incond with the ideal gas law";
  Real N_incond(start=0) "can be considered as an image of the incondensable quantity present in the condenser";

  Common.Connectors.FluidInlet C_hot_in( redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{-10,92},{10,112}})));
  Common.Connectors.FluidOutlet C_hot_out(redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{-10,-112},{10,-92}})));
  replaceable Common.Partial.BasicTransportModel coldSide(redeclare package
      Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(
        extent={{-25,-18},{25,18}},
        rotation=0,
        origin={45,0})));
  Common.Connectors.FluidInlet C_cold_in(redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Common.Connectors.FluidOutlet C_cold_out(redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  replaceable Common.Partial.BasicTransportModel hotSide(redeclare package
      Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(
        extent={{-25,-18},{25,18}},
        rotation=-90,
        origin={-27,-36})));
equation
  /***** HOT SIDE ********/
  hotSide.Q_in + hotSide.Q_out + Q_top_up = 0;
  //Energy balance
  hotSide.Q_in*hotSide.h_in + hotSide.Q_out*hotSide.h_out = W;

  //introduction of a decoupling between Psat and the pressure in condenser
  //We can assume that in real life there's not only water in condenser, with some incindensable present also
  Ptot=hotSide.P_in;
  hotSide.P_in=P_incond+Psat;//In reference to the Dalton's law
  P_incond=P_offset+N_incond*Tsat;//in reference to the ideal gas law

  hotSide.h_out = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_T(Tsat)); //use of Tsat instead of Psat to improve model stability
  rho_liquid_hot=WaterSteamMedium.bubbleDensity(WaterSteamMedium.setSat_T(Tsat));
  deltaP_hot = hotSide.P_out - hotSide.P_in;
  deltaP_hot = rho_liquid_hot*g*WaterHeight;
  Tsat = hotSide.T_out;
  /***** COLD SIDE ********/
   Tcold_in=coldSide.T_in;
   Tcold_out=coldSide.T_out;
  coldSide.Q_in + coldSide.Q_out = 0;
  coldSide.Q_in*coldSide.h_in + coldSide.Q_out*coldSide.h_out = -W;
  deltaP_cold = coldSide.P_in - coldSide.P_out;
  deltaP_cold = Kfr_cold*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(coldSide.Q_in, coldSide.eps)/coldSide.rho_in;
  0 = Tsat - coldSide.T_out - (Tsat - coldSide.T_in)*exp(Kth*S*((coldSide.T_in - coldSide.T_out)/W));
  if mass_balance then
    hotSide.Q_in+hotSide.Q_out = 0;
  end if;
  connect(coldSide.C_in, C_cold_in) annotation (Line(points={{20,0},{-100,0}},  color={0,0,255}));
  connect(coldSide.C_out, C_cold_out)
    annotation (Line(points={{70.5,0},{100,0}}, color={238,46,47}));
  connect(hotSide.C_in, C_hot_in) annotation (Line(points={{-27,-11},{-27,34},{0,34},{0,102}}, color={0,0,255}));
  connect(hotSide.C_out, C_hot_out)
    annotation (Line(points={{-27,-61.5},{-27,-72},{0,-72},{0,-102}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Polygon(
          points={{100,-86},{100,94},{-100,94},{-100,-86},{100,-86}},
          lineColor={0,0,255},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-100,-14},{80,-14},{80,-20},{-90,-20},{-90,-26},{80,-26},{80,
              -32},{-90,-32},{-90,-38},{100,-38}},
          color={0,0,255},
          thickness=0.5),
        Polygon(
          points={{0,-90},{-11,-70},{11,-70},{0,-90}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillPattern=FillPattern.Sphere,
          fillColor={191,0,0}),
        Line(
          points={{0,8},{0,-70}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{-100,8},{100,8}},
          color={0,0,255},
          thickness=0.5)}),                                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end CondenserSimple_wIncondensable;
