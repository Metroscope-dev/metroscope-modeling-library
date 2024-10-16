within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model CoolingTowerMerkel
                            //Reference Paper: Abdo, Rodrigo & Rodrigues, Yuri & Silva, Víctor & Cabezas-Gómez, Luben & Hanriot, Sérgio,  The difference between Merkel and Poppe models and its influence on the prediction of wet-cooling towers, 2013

  MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_cold_in annotation (Placement(transformation(extent={{-10,-116},{10,-96}})));
  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  package MoistAir = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  import MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium.specificEnthalpy;

  Units.Velocity V_inlet;                  //Air velocity at the bottom of the cooling tower
  Inputs.InputReal hd;                     //Mass transfer coefficient
  Inputs.InputArea Afr;                    //Tower cross-sectional area
  Inputs.InputReal Lfi;                    //Height of filling/packing
  Inputs.InputFrictionCoefficient Cf;      //Friction coefficient of air
  Inputs.InputReal afi;                    //Fill material surface area per unit volume
  Inputs.InputReal Ratio;                  //Ratio of Q_evaporation to P_thermal used to see if results align with an EDF reference paper
  Inputs.InputReal eta_fan;                //Fan effiency
  Units.Power W_fan;                       //Fan power

  Units.Power W;                           //Heat transfer-rate or Power of the Cooling Tower

  parameter String configuration = "mechanical";

  Units.MassFlowRate Q_cold_in;
  Units.MassFlowRate Q_cold_out;
  Units.MassFlowRate Q_hot_in;
  Units.MassFlowRate Q_hot_out;
  Units.MassFlowRate Q_evap;
  Units.VolumeFlowRate Qv_evap;

  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  Units.Temperature T1(start=T1_0);
  Units.Temperature T2(start=T2_0);
  Units.Temperature T3(start=T3_0);
  Units.Temperature T4(start=T4_0);

  Units.SpecificEnthalpy i_initial(start=i_initial_0);
  Units.SpecificEnthalpy i_final(start=i_final_0);
  Units.SpecificEnthalpy i1(start=i1_0);
  Units.SpecificEnthalpy i2(start=i2_0);
  Units.SpecificEnthalpy i3(start=i3_0);
  Units.SpecificEnthalpy i4(start=i4_0);
  Units.SpecificEnthalpy iTot(start=iTot_0);

  Units.Density rho_air_inlet(start=rho_air_inlet_0);
  Units.Density rho_air_outlet(start=rho_air_outlet_0);

  Units.HeatCapacity cp;
  Units.Pressure P_in;
  Units.Pressure P_out;
  Units.Pressure deltaP_fan;                             //Pressure Change across Fan

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

  parameter Units.Density rho_air_inlet_0 = 1.2754;
  parameter Units.Density rho_air_outlet_0 = 1.2460;

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling(min = 0, max=100, start=0, nominal=10); // Fouling percentage

  MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_hot_in annotation (Placement(transformation(extent={{-100,-10},{-80,10}}), iconTransformation(extent={{-100,-10},{-80,10}})));
  MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_hot_out annotation (Placement(transformation(extent={{80,-10},{100,10}})));
  MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_cold_out annotation (Placement(transformation(extent={{-10,94},{10,114}})));
  WaterSteam.BaseClasses.IsoPHFlowModel                          hot_side_cooling annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink Air_inlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-16})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source Air_outlet annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,26})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel inputflowmodel annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-48})));
  MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel outputflowmodel annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,64})));
  MetroscopeModelingLibrary.MoistAir.Pipes.Pipe pipe annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-78})));
  WaterSteam.BoundaryConditions.Sink Water_inlet annotation (Placement(transformation(extent={{-32,-10},{-12,10}})));
  WaterSteam.BoundaryConditions.Source Water_outlet annotation (Placement(transformation(extent={{10,-10},{30,10}})));
equation

  // Failure modes
  if not faulty then
    fouling = 0;
  end if;


  // Definition
  Q_cold_in = Air_inlet.Q_in;
  Q_cold_out = Air_outlet.Q_out;
  Q_hot_in = Water_inlet.Q_in;
  Q_hot_out = Water_outlet.Q_out;

  T_hot_in = Water_inlet.T_in;
  T_hot_out = Water_outlet.T_out;
  P_in = Air_inlet.P_in;
  P_out = Air_outlet.P_out;
  T_cold_in = Air_inlet.T_in;
  T_cold_out = Air_outlet.T_out;

  cp = WaterSteamMedium.specificHeatCapacityCp(hot_side_cooling.state_in);
  W = Q_hot_in * cp * (T_hot_in - T_hot_out);

  Qv_evap = Q_evap / 1000;
  Q_evap = - (Q_cold_out + Q_cold_in);

  Ratio = Q_evap / W;

  // Energy Balance
  Q_hot_in * cp * (T_hot_in - T_hot_out) + Q_cold_in * (i_initial - i_final) = 0;

  // Tchebyshev Integral
  T1 = T_hot_out + 0.1 * (T_hot_in - T_hot_out);
  T2 = T_hot_out + 0.4 * (T_hot_in - T_hot_out);
  T3 = T_hot_out + 0.6 * (T_hot_in - T_hot_out);
  T4 = T_hot_out + 0.9 * (T_hot_in - T_hot_out);

  i_initial = Air_inlet.h_in;
  i_final = Air_outlet.h_out;

  Air_outlet.relative_humidity = 1;                                                              //Most Significant Hypothesis of the Model
  Air_outlet.Q_out * (1 - Air_outlet.Xi_out[1]) = - Air_inlet.Q_in *(1 - Air_inlet.Xi_in[1]);

  Water_outlet.P_out = Water_inlet.P_in;
  Q_hot_out = -(Q_hot_in - Q_evap);

  i1 = MoistAir.h_pTX(P_in, T1, {MoistAir.massFraction_pTphi(P_in, T1, 1)}) - ((i_initial + 0.1 * (i_final - i_initial)));                                                                                                                                                                                                        //First integral section
  i2 = MoistAir.h_pTX(P_in, T2, {MoistAir.massFraction_pTphi(P_in, T2, 1)}) - ((i_initial + 0.4 * (i_final - i_initial)));
  i3 = MoistAir.h_pTX(P_in, T3, {MoistAir.massFraction_pTphi(P_in, T3, 1)}) - ((i_initial + 0.6 * (i_final - i_initial)));
  i4 = MoistAir.h_pTX(P_in, T4, {MoistAir.massFraction_pTphi(P_in, T4, 1)}) - ((i_initial + 0.9 * (i_final - i_initial)));
  iTot = 1 / i1 + 1 / i2 + 1 / i3 + 1 / i4;

  // Heat Exchange - Merkel
  (Afr * hd * (1 - fouling/100) * afi * Lfi) / Q_hot_in = cp * iTot * ((T_hot_in - T_hot_out) / 4);

  // Drift Equation
  rho_air_inlet = inputflowmodel.rho_in;
  rho_air_outlet = outputflowmodel.rho_out;

  pipe.Kfr = 0;
  pipe.delta_z = 0;

  Air_inlet.P_in = Air_outlet.P_out;

  deltaP_fan = (W_fan * eta_fan)/(abs(V_inlet) * Afr);

  if configuration == "natural" then

  0.5 * 0.5 *(rho_air_inlet + rho_air_outlet) * Cf * abs(V_inlet) * V_inlet  =  (rho_air_inlet - rho_air_outlet) * g * Lfi;
  Q_cold_in = (V_inlet * Afr * rho_air_inlet * (1 - Air_inlet.Xi_in[1]));

  elseif configuration == "mechanical" then

  0.5 * 0.5 *(rho_air_inlet + rho_air_outlet) * Cf * abs(V_inlet) * V_inlet  = (W_fan * eta_fan)/(abs(V_inlet) * Afr);
  Q_cold_in = (V_inlet * Afr * rho_air_inlet * (1 - Air_inlet.Xi_in[1]));

  end if;

  connect(C_hot_in, hot_side_cooling.C_in) annotation (Line(points={{-90,0},{-70,0}}, color={28,108,200}));
  connect(inputflowmodel.C_out, Air_inlet.C_in) annotation (Line(points={{0,-38},{0,-21}},                                                           color={85,170,255}));
  connect(Air_outlet.C_out, outputflowmodel.C_in) annotation (Line(points={{0,31},{0,54}},                                                                 color={85,170,255}));
  connect(outputflowmodel.C_out, C_cold_out) annotation (Line(points={{0,74},{0,104}},                                        color={85,170,255}));
  connect(pipe.C_in, C_cold_in) annotation (Line(points={{0,-88},{0,-106}},                                color={85,170,255}));
  connect(pipe.C_out, inputflowmodel.C_in) annotation (Line(points={{0,-68},{0,-58}},                                                       color={85,170,255}));
  connect(hot_side_cooling.C_out, Water_inlet.C_in) annotation (Line(points={{-50,0},{-27,0}},                                color={28,108,200}));
  connect(Water_outlet.C_out, C_hot_out) annotation (Line(points={{25,0},{90,0}}, color={28,108,200}));
  connect(C_cold_in, C_cold_in) annotation (Line(points={{0,-106},{0,-106}},
                                                                         color={85,170,255}));
  connect(C_hot_out, C_hot_out) annotation (Line(points={{90,0},{90,0}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}}), graphics={
        Ellipse(
          extent={{-20,110},{20,70}},
          lineColor={28,108,200},
          fillColor={95,95,95},
          fillPattern=FillPattern.Backward),
        Line(points={{-80,-80},{82,-80},{40,60},{-40,60},{-80,-80}}, color={28,108,200}),
        Ellipse(
          extent={{-48,82},{-40,74}},
          lineColor={28,108,200},
          fillColor={95,95,95},
          fillPattern=FillPattern.Backward),
        Ellipse(
          extent={{32,114},{40,106}},
          lineColor={28,108,200},
          fillColor={95,95,95},
          fillPattern=FillPattern.Backward),
        Ellipse(
          extent={{28,78},{36,70}},
          lineColor={28,108,200},
          fillColor={95,95,95},
          fillPattern=FillPattern.Backward),
        Ellipse(
          extent={{-36,110},{-28,104}},
          lineColor={28,108,200},
          fillColor={95,95,95},
          fillPattern=FillPattern.Backward),
        Ellipse(
          extent={{26,-40},{-28,26}},
          lineColor={28,108,200},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,120}})));
end CoolingTowerMerkel;
