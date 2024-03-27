within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model CoolingTower2
  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  package MoistAir = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  import MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium.specificEnthalpy;

  Inputs.InputArea Afr;
  Inputs.InputReal hd;
  Inputs.InputReal Lfi;
  Inputs.InputReal afi;
  Inputs.InputReal Vd;
  Inputs.InputFrictionCoefficient Cf;

  Units.MassFlowRate Q_cold;             //REMOVED THE INITIALIZATION VALUES
  Units.MassFlowRate Q_hot;

  Units.MassFlowRate Q_makeup;

  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  Units.Temperature T1(start=T1_0);
  Units.Temperature T2(start=T2_0);
  Units.Temperature T3(start=T3_0);
  Units.Temperature T4(start=T4_0);

  Units.Power W;

  Units.SpecificEnthalpy i_initial(start=i_initial_0);
  Units.SpecificEnthalpy i_final(start=i_final_0);
  Units.SpecificEnthalpy i1(start=i1_0);
  Units.SpecificEnthalpy i2(start=i2_0);
  Units.SpecificEnthalpy i3(start=i3_0);
  Units.SpecificEnthalpy i4(start=i4_0);
  Units.SpecificEnthalpy iTot(start=iTot_0);

  Units.Density d_air_initial(start=d_air_initial_0);
  Units.Density d_air_final(start=d_air_final_0);

  Units.HeatCapacity cp;
  Units.Pressure P_in;
  Units.Pressure P_out;

  constant Real R(unit="J/(mol.K)") = Modelica.Constants.R "ideal gas constant";
  constant Real g(unit="m/s2") = Modelica.Constants.g_n;

   // Initialization Parameters

  parameter Units.Temperature T_cold_in_0 = 15 + 273.15;
  parameter Units.Temperature T_cold_out_0 = 25 + 273.15;
  parameter Units.Temperature T_hot_in_0 = 40 + 273.15;
  parameter Units.Temperature T_hot_out_0 = 20 + 273.15;

  parameter Units.Temperature T1_0 = 15 + 273.15;
  parameter Units.Temperature T2_0 = 18 + 273.15;
  parameter Units.Temperature T3_0 = 22 + 273.15;
  parameter Units.Temperature T4_0 = 25 + 273.15;

  parameter Units.SpecificEnthalpy i_initial_0 = 0.5e5;
  parameter Units.SpecificEnthalpy i_final_0 = 1.05e5;
  parameter Units.SpecificEnthalpy i1_0 = 0.65e5;
  parameter Units.SpecificEnthalpy i2_0 = 0.8e5;
  parameter Units.SpecificEnthalpy i3_0 = 0.9e5;
  parameter Units.SpecificEnthalpy i4_0 = 1e5;
  parameter Units.SpecificEnthalpy iTot_0 = (1 / (2e5));

  parameter Units.Density d_air_initial_0 = 1.2754;
  parameter Units.Density d_air_final_0 = 1.2460;

  MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_hot_in annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_hot_out annotation (Placement(transformation(extent={{80,-10},{100,10}})));
  MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_cold_in annotation (Placement(transformation(extent={{-10,80},{10,100}})));
  MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_cold_out annotation (Placement(transformation(extent={{-10,-100},{10,-80}})));
  MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPFlowModel hot_side_cooling annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink Air_inlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,24})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source Air_outlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-38})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel inputflowmodel annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,52})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel outputflowmodel annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,-60})));
equation
  // Definition
  //Q_cold = Air_inlet.Q_in;                          //REMOVED THIS
  //Q_hot = hot_side_cooling.Q;

  T_hot_in = hot_side_cooling.T_in;
  T_hot_out = hot_side_cooling.T_out;
  P_in = Air_inlet.P_in;
  P_out = Air_outlet.P_out;
  T_cold_in = Air_inlet.T_in;
  T_cold_out = Air_outlet.T_out;

  cp = WaterSteamMedium.specificHeatCapacityCp(hot_side_cooling.state_in);
  W = Q_hot * cp * (T_hot_in - T_hot_out);
  Q_makeup = -(Air_outlet.Q_out + Air_inlet.Q_in);

  // Energy Balance - Supplementary Equation
  Q_hot * cp * (T_hot_in - T_hot_out) + Q_cold * (i_initial - i_final) = 0;

  // Tchebyshev Integral
  T1 = T_hot_out + 0.1 * (T_hot_in - T_hot_out);
  T2 = T_hot_out + 0.4 * (T_hot_in - T_hot_out);
  T3 = T_hot_out + 0.6 * (T_hot_in - T_hot_out);
  T4 = T_hot_out + 0.9 * (T_hot_in - T_hot_out);

  i_initial = Air_inlet.h_in;
  i_final = Air_outlet.h_out;

  Air_outlet.relative_humidity = 1;
  Air_outlet.Q_out * (1 - Air_outlet.Xi_out[1]) = - Air_inlet.Q_in *(1 - Air_inlet.Xi_in[1]);
  Air_outlet.P_out = Air_inlet.P_in;

  i1 = MoistAir.h_pTX(P_in, T1, {MoistAir.massFraction_pTphi(P_in, T1, 1)}) - ((i_initial + 0.1 * (i_final - i_initial)));                                                                                                                                                                                                        //First integral section
  i2 = MoistAir.h_pTX(P_in, T2, {MoistAir.massFraction_pTphi(P_in, T2, 1)}) - ((i_initial + 0.4 * (i_final - i_initial)));
  i3 = MoistAir.h_pTX(P_in, T3, {MoistAir.massFraction_pTphi(P_in, T3, 1)}) - ((i_initial + 0.6 * (i_final - i_initial)));
  i4 = MoistAir.h_pTX(P_in, T4, {MoistAir.massFraction_pTphi(P_in, T4, 1)}) - ((i_initial + 0.9 * (i_final - i_initial)));
  iTot = 1 / i1 + 1 / i2 + 1 / i3 + 1 / i4;

  // Heat Exchange - Merkel
  (Afr * hd * afi * Lfi) / Q_hot = cp * iTot * ((T_hot_in - T_hot_out) / 4);

  // Drift Equation
  d_air_initial = inputflowmodel.rho_in;
  d_air_final = outputflowmodel.rho_in;

  0.25 * (d_air_initial - d_air_final) * Vd * Vd * Cf - ((d_air_initial - d_air_final) * g * Lfi) = 0;
  Q_cold - (Vd * Afr * d_air_initial * (1 - Air_outlet.Xi_out[1])) = 0;
  connect(C_hot_in, hot_side_cooling.C_in) annotation (Line(points={{-90,0},{-50,0}}, color={28,108,200}));
  connect(hot_side_cooling.C_out, C_hot_out) annotation (Line(points={{-30,0},{90,0}}, color={28,108,200}));
  connect(inputflowmodel.C_out, Air_inlet.C_in) annotation (Line(points={{-1.77636e-15,42},{-1.77636e-15,35.5},{8.88178e-16,35.5},{8.88178e-16,29}}, color={85,170,255}));
  connect(inputflowmodel.C_in, C_cold_in) annotation (Line(points={{1.77636e-15,62},{1.77636e-15,76},{0,76},{0,90}}, color={85,170,255}));
  connect(Air_outlet.C_out, outputflowmodel.C_in) annotation (Line(points={{-8.88178e-16,-43},{-8.88178e-16,-46.5},{1.77636e-15,-46.5},{1.77636e-15,-50}}, color={85,170,255}));
  connect(outputflowmodel.C_out, C_cold_out) annotation (Line(points={{-1.77636e-15,-70},{-1.77636e-15,-80},{0,-80},{0,-90}}, color={85,170,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end CoolingTower2;
