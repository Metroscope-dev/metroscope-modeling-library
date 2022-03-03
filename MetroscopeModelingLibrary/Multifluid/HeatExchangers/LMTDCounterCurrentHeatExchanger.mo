within MetroscopeModelingLibrary.Multifluid.HeatExchangers;
model LMTDCounterCurrentHeatExchanger
   package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  package MoistAirMedium =
      MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;

  connector InputReal = input Real;

  // From partialHeatExchanger
  Real K_friction_cold(start=1.e-3) "Pressure loss coefficient";
  Real K_friction_hot(start=1.e-3) "Pressure loss coefficient";
  Modelica.Units.SI.Power W(start=1e8);//57824);//
  Modelica.Units.SI.MassFlowRate Q_cold(start= 100) "Inlet Mass flow rate"; //start=572);//
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_cold(start = 0) "Singular pressure loss";
  Modelica.Units.SI.MassFlowRate Q_hot(start=100) "Inlet Mass flow rate";//0.02);//
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_hot(start= 0) "Singular pressure loss";

  // For LMTD method
  Modelica.Units.SI.CoefficientOfHeatTransfer K(start=100);// "Global heat transfer coefficient (active if exchanger_type=3)"//28.3);
  Modelica.Units.SI.Area S(start=10); //Modelica.Units.SI.Power Wmax(start=1e8);
  Real T_hot_out(start = 7+273.15);//31 + 273.15);
  Real T_hot_in(start= 20 + 273.15);//279 + 273.15);
  Real T_cold_out(start = 6+273.15);//14.9 + 273.15);
  Real T_cold_in(start = 5+273.15);//15.1+273.15);
  Real dT_a(start = 15);
  Real dT_b(start=1);
  //ColdMedium.ThermodynamicState state_cold_median;
  //HotMedium.ThermodynamicState state_hot_median;

 replaceable Common.Partial.BasicTransportModel coldSide( redeclare package
      Medium =
       WaterSteamMedium)
   annotation (Placement(transformation(extent={{-28,-52},{22,-16}})));
 replaceable Common.Partial.BasicTransportModel hotSide( redeclare package
      Medium =
       MoistAirMedium)
   annotation (Placement(transformation(extent={{-28,4},{22,40}})));
 Common.Connectors.FluidInlet C_hot_in( redeclare package Medium =
       MoistAirMedium)
   annotation (Placement(transformation(extent={{-108,-8},{-88,12}}),
       iconTransformation(extent={{-108,-8},{-88,12}})));
 Common.Connectors.FluidOutlet C_hot_out(redeclare package Medium =
        MoistAirMedium)
   annotation (Placement(transformation(extent={{88,-10},{108,10}}),
       iconTransformation(extent={{88,-10},{108,10}})));
 Common.Connectors.FluidInlet C_cold_in( redeclare package Medium =
       WaterSteamMedium)
   annotation (Placement(transformation(extent={{-10,90},{10,110}})));
 Common.Connectors.FluidOutlet C_cold_out(redeclare package Medium =
       WaterSteamMedium)
   annotation (Placement(transformation(extent={{-10,-108},{10,-88}})));
equation

  // *** Common equations *** (from partialHeatExchanger)
  // Mechanical balance
  deltaP_cold = coldSide.P_in - coldSide.P_out;
  deltaP_hot = hotSide.P_in - hotSide.P_out;
  deltaP_cold = K_friction_cold*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_cold, coldSide.eps)/coldSide.rhom;
  deltaP_hot = K_friction_hot*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_hot, hotSide.eps)/hotSide.rhom;
  // Mass balance
  Q_cold = coldSide.Q_in;
  Q_hot = hotSide.Q_in;
  coldSide.Q_in + coldSide.Q_out = 0;
  hotSide.Q_in + hotSide.Q_out = 0;
  coldSide.Q_in*coldSide.Xi_in = -coldSide.Q_out*coldSide.Xi_out;
  hotSide.Q_in*hotSide.Xi_in = -hotSide.Q_out*hotSide.Xi_out;
  // Energy balance
  coldSide.Q_in*coldSide.h_in + coldSide.Q_out*coldSide.h_out = -W;
  hotSide.Q_in*hotSide.h_in + hotSide.Q_out*hotSide.h_out = W;

  //state_cold_median = WaterSteamMedium.setState_pTX(coldSide.P_in, (coldSide.T_in + hotSide.T_in)/2,coldSide.Xi_in);
  //state_hot_median = MoistAirMedium.setState_pTX(hotSide.P_in, (coldSide.T_in + hotSide.T_in)/2,hotSide.Xi_in);

  // *** LMTD method ***
 T_hot_out = hotSide.T_out;
 T_hot_in = hotSide.T_in;
 T_cold_out = coldSide.T_out;
 T_cold_in = coldSide.T_in;

 dT_a = T_hot_in - T_cold_out;
 dT_b = T_hot_out - T_cold_in;

 0 = - dT_a + dT_b * exp(K*S*(dT_a - dT_b)/W);

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
end LMTDCounterCurrentHeatExchanger;
