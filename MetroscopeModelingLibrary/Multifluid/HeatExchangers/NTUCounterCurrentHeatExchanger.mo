within MetroscopeModelingLibrary.Multifluid.HeatExchangers;
model NTUCounterCurrentHeatExchanger
  extends MetroscopeModelingLibrary.Multifluid.HeatExchangers.PartialHeatExchanger;
 Modelica.SIunits.CoefficientOfHeatTransfer K(start = 100)
   "Global heat transfer coefficient (active if exchanger_type=3)";
 Modelica.SIunits.Area S(start = 10);
 Modelica.SIunits.Power Wmax(start=1e8);
 Modelica.SIunits.SpecificHeatCapacity Cp_hot(start=1000)
   "Flue gas specific heat capacity";
 Modelica.SIunits.SpecificHeatCapacity Cp_cold(start=4200)
   "Water specific heat capacity";
 Real NTU(start=0.3);
 Real epsilon(start=0.9);
 Real Cmin(start=10000);
 Real Cmax(start=42000);
 Real Cr(start=0.2);
 ColdMedium.ThermodynamicState state_cold_median;
  HotMedium.ThermodynamicState state_hot_median;
 replaceable Common.Partial.BasicTransportModel coldSide( redeclare package
     Medium =
       ColdMedium)
   annotation (Placement(transformation(extent={{-28,-52},{22,-16}})));
 replaceable Common.Partial.BasicTransportModel hotSide( redeclare package
     Medium =
       HotMedium)
   annotation (Placement(transformation(extent={{-28,4},{22,40}})));
 Common.Connectors.FluidInlet C_hot_in( redeclare package Medium =
       HotMedium)
   annotation (Placement(transformation(extent={{-108,-8},{-88,12}}),
       iconTransformation(extent={{-108,-8},{-88,12}})));
 Common.Connectors.FluidOutlet C_hot_out(redeclare package Medium = HotMedium)
   annotation (Placement(transformation(extent={{88,-10},{108,10}}),
       iconTransformation(extent={{88,-10},{108,10}})));
 Common.Connectors.FluidInlet C_cold_in( redeclare package Medium =
       ColdMedium)
   annotation (Placement(transformation(extent={{-10,90},{10,110}})));
 Common.Connectors.FluidOutlet C_cold_out(redeclare package Medium =
       ColdMedium)
   annotation (Placement(transformation(extent={{-10,-108},{10,-88}})));
equation
 state_cold_median = ColdMedium.setState_pTX(coldSide.P_in, (coldSide.T_in + hotSide.T_in)/2,coldSide.Xi_in);
  state_hot_median = HotMedium.setState_pTX(hotSide.P_in, (coldSide.T_in + hotSide.T_in)/2,hotSide.Xi_in);
  Cp_cold = ColdMedium.specificHeatCapacityCp(state_cold_median);
 Cp_hot = HotMedium.specificHeatCapacityCp(state_hot_median);
 Cmin = min(hotSide.Q_in*Cp_hot,coldSide.Q_in*Cp_cold);
 Cmax = max(hotSide.Q_in*Cp_hot,coldSide.Q_in*Cp_cold);
 Cr = Cmin/Cmax;
 Wmax = Cmin*(hotSide.T_in-coldSide.T_in);
 W = epsilon*Wmax;
 epsilon = (1-exp(-NTU*(1-Cr)))/(1-Cr*exp(-NTU*(1-Cr)));
 NTU = K*S/Cmin;
 connect(C_hot_in, hotSide.C_in) annotation (Line(points={{-98,2},{-52,2},{-52,
         22},{-28,22}},      color={0,0,255}));
 connect(hotSide.C_out, C_hot_out) annotation (Line(points={{22.5,22},{62,22},{
         62,0},{98,0}},       color={238,46,47}));
 connect(coldSide.C_in, C_cold_in) annotation (Line(points={{-28,-34},{-68,-34},
         {-68,68},{0,68},{0,100}},color={0,0,255}));
 connect(coldSide.C_out, C_cold_out) annotation (Line(points={{22.5,-34},{48,
         -34},{48,-66},{0,-66},{0,-98}},
                                    color={238,46,47}));
 annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
       Rectangle(
         extent={{-100.5,15.5},{100.5,-15.5}},
         lineColor={0,0,255},
         fillColor={255,255,0},
         fillPattern=FillPattern.Backward,
         origin={73.5,0.5},
         rotation=90),
       Rectangle(
         extent={{-100,59},{100,-59}},
         lineColor={0,0,255},
         fillColor={255,255,0},
         fillPattern=FillPattern.Solid,
         origin={0,1},
         rotation=90),
       Line(
         points={{-92,-1},{-42,-1},{-22,47},{18,-47},{38,-1},{92,-1}},
         color={0,0,0},
         thickness=0.5,
         origin={-2,-1},
         rotation=-90),
       Text(
         extent={{-38,10},{38,-4}},
         lineColor={0,0,0},
         lineThickness=0.5,
         fillPattern=FillPattern.HorizontalCylinder,
         fillColor={175,175,175},
         textString="Cold Side"),
       Rectangle(
         extent={{-100,15},{100,-15}},
         lineColor={0,0,255},
         fillColor={255,255,0},
         fillPattern=FillPattern.Backward,
         origin={-74,1},
         rotation=-90),
       Text(
         extent={{-31,8},{31,-8}},
         lineColor={0,0,0},
         lineThickness=0.5,
         fillPattern=FillPattern.HorizontalCylinder,
         fillColor={175,175,175},
         textString="Hot Side",
         origin={75,-2},
         rotation=90),
       Text(
         extent={{-31,8},{31,-8}},
         lineColor={0,0,0},
         lineThickness=0.5,
         fillPattern=FillPattern.HorizontalCylinder,
         fillColor={175,175,175},
         textString="Hot Side",
         origin={-75,0},
         rotation=90)}),                                        Diagram(
       coordinateSystem(preserveAspectRatio=false)));
end NTUCounterCurrentHeatExchanger;
