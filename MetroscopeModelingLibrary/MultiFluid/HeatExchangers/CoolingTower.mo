within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model CoolingTower
  package Water = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  import MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

  //Inputs.InputArea S;
  //Units.HeatExchangeCoefficient Kth;                 NEED TO INPUT MY PARAMETERS AND DEFINE THEM AS CONSTANTS WITH SET KNOWN VALUES
  //Inputs.InputFrictionCoefficient Kfr_hot;

  Units.MassFlowRate Q_cold(start=Q_cold_0);
  Units.MassFlowRate Q_hot(start=Q_hot_0);

  Units.Temperature T_cold_in(start=T_cold_in_0);
  Units.Temperature T_cold_out(start=T_cold_out_0);
  Units.Temperature T_hot_in(start=T_hot_in_0);
  Units.Temperature T_hot_out(start=T_hot_out_0);

  Units.Temperature T1(start=T1_0);
  Units.Temperature T2(start=T2_0);
  Units.Temperature T3(start=T3_0);
  Units.Temperature T4(start=T4_0);

  Units.SpecificEnthalpy i_initial;
  Units.SpecificEnthalpy i1;
  Units.SpecificEnthalpy i2;
  Units.SpecificEnthalpy i3;
  Units.SpecificEnthalpy i4;
  Units.SpecificEnthalpy iTot;

  Units.HeatCapacity cp;

  Units.Pressure Psat;
  //Units.Pressure P_tot(start=Psat_0, nominal=Psat_0);
  Units.Temperature Tsat;

  constant Real R(unit="J/(mol.K)") = Modelica.Constants.R "ideal gas constant";

   // Initialization Parameters
  parameter Units.MassFlowRate Q_cold_0 = 1800*1.292;
  parameter Units.MassFlowRate Q_hot_0 = 21;

  parameter Units.Temperature T_cold_in_0 = 15 + 273.15;
  parameter Units.Temperature T_cold_out_0 = 30 + 273.15;
  parameter Units.Temperature T_hot_in_0 = 40 + 273.15;
  parameter Units.Temperature T_hot_out_0 = 20 + 273.15;

  parameter Units.Temperature T1_0 = 15 + 273.15;
  parameter Units.Temperature T2_0 = 20 + 273.15;
  parameter Units.Temperature T3_0 = 25 + 273.15;
  parameter Units.Temperature T4_0 = 30 + 273.15;

  //Need to initialize the enthalpies ?

  //parameter Units.SpecificEnthalpy h_liq_sat_0 = Water.bubbleEnthalpy(Water.setSat_p(Psat_0));

  //parameter Units.Temperature Tsat = Water.saturationTemperature(Psat);
  WaterSteam.Connectors.Inlet C_hot_in annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  WaterSteam.Connectors.Outlet C_hot_out annotation (Placement(transformation(extent={{80,-10},{100,10}})));
  MoistAir.Connectors.Inlet C_cold_in annotation (Placement(transformation(extent={{-10,80},{10,100}})));
  MoistAir.Connectors.Outlet C_cold_out annotation (Placement(transformation(extent={{-10,-100},{10,-80}})));
  WaterSteam.BaseClasses.IsoPFlowModel hot_side_condensing annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  WaterSteam.BaseClasses.IsoPFlowModel cold_side_condensing annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,38})));
equation
  // Definition
  Q_hot = hot_side_condensing.Q;
  Q_cold = cold_side_condensing.Q;

  T_hot_in = hot_side_condensing.T_in;
  T_hot_out = hot_side_condensing.T_out;
  T_cold_in = cold_side_condensing.T_in;
  T_cold_out = Tsat;

  // Energy Balance - Supplementary Equation
  Q_hot * cp * (T_hot_in - T_hot_out) - Q_cold * (h_pTX(1, T_hot_in, 1) - h_pTX(1, T_hot_out, 1)) = 0;

  // Saturation
  Tsat =  Water.saturationTemperature(Psat);

  // Tchebyshev Integral
  T1 = T_hot_out + 0.1 * (T_hot_in - T_hot_out);
  T2 = T_hot_out + 0.4 * (T_hot_in - T_hot_out);
  T3 = T_hot_out + 0.6 * (T_hot_in - T_hot_out);
  T4 = T_hot_out + 0.9 * (T_hot_in - T_hot_out);

  i_initial = h_pTX(1, 273.15 + 15, 1);                                                         //Initial Specific Enthalpy of Moist Air at Inlet
  i1 = h_pTX(1, T1, 1) - ((i_initial + 0.1 * (h_pTX(1, T_cold_out, 1) - i_initial)));           //First integral section
  i2 = h_pTX(1, T2, 1) - ((i_initial + 0.4 * (h_pTX(1, T_cold_out, 1) - i_initial)));
  i3 = h_pTX(1, T3, 1) - ((i_initial + 0.6 * (h_pTX(1, T_cold_out, 1) - i_initial)));
  i4 = h_pTX(1, T4, 1) - ((i_initial + 0.9 * (h_pTX(1, T_cold_out, 1) - i_initial)));
  iTot = 1 / i1 + 1 / i2 + 1 / i3 + 1 / i4;                                                     //Sum of the inverse enthalpy differences

  // Heat Exchange - Merkel
  // (need to input variables)/Q_hot = cp * iTot * (T_hot_in - T_hot_out)/4
  connect(C_hot_in, hot_side_condensing.C_in) annotation (Line(points={{-90,0},{-50,0}}, color={28,108,200}));
  connect(hot_side_condensing.C_out, C_hot_out) annotation (Line(points={{-30,0},{90,0}}, color={28,108,200}));
  connect(C_cold_in, cold_side_condensing.C_in) annotation (Line(points={{0,90},{0,69},{1.77636e-15,69},{1.77636e-15,48}}, color={85,170,255}));
  connect(cold_side_condensing.C_out, C_cold_out) annotation (Line(points={{-1.77636e-15,28},{-1.77636e-15,-31},{0,-31},{0,-90}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end CoolingTower;
