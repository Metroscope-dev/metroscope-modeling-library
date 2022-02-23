within MetroscopeModelingLibrary.Tests.UnitTests.Multifluid.HeatExchangers;
model TestLMTDCounterCurrentHeatExchanger
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  package MoistAirMedium =
      MetroscopeModelingLibrary.MoistAir.Medium.MoistAirMedium;
      // 1 output to activate (=uncomment) if causality is direct :
   //output Real Wth(start = 11819);
   //output Real h_hot_out(start= 29042);
   //output Real h_cold_out(start= 22123);

  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source moistAirSource
    annotation (Placement(transformation(extent={{-72,70},{-52,90}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink moistAirSink
    annotation (Placement(transformation(extent={{66,70},{86,90}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source waterSource
    annotation (Placement(transformation(extent={{-40,106},{-20,126}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink waterSink
    annotation (Placement(transformation(extent={{18,36},{38,56}})));
  MetroscopeModelingLibrary.Multifluid.HeatExchangers.LMTDCounterCurrentHeatExchanger
    lMTDCounterCurrentHeatExchanger
    annotation (Placement(transformation(extent={{0,70},{20,90}})));
equation
  // moist air = hot side
  // water = cold side

  //inlets
  moistAirSource.P_out = 0.989e5;
  moistAirSource.Q_out = -2;
  moistAirSource.T_vol = 20 + 273.15;
  moistAirSource.relative_humidity = 0.4;
  waterSource.P_out = 10e5;
  waterSource.Q_out = -100;
  waterSource.T_vol = 5+273.15;
  //outlets
  moistAirSink.T_vol = 40 + 273.15;
  moistAirSink.Xi_vol = {0.01};
  waterSink.T_vol = 10+273.15;
  //heat exchanger
  lMTDCounterCurrentHeatExchanger.S = 10;

  // *** direct causality  ***
  // Initialization of one of the next variables is very likely to be required in order to reach convergence in direct causality
  // Chose between : W, hotSide.h_out and ColdSide.h_out.
  // Be carefull : an h_out can also be chosen as a reverse model parameter. In that case, intitialize W or the other h_out.
  // 2nd be carefull - check the consistency : the output declared above must match with the chosen convergence variable.
  // --> convergence variables :
  //lMTDCounterCurrentHeatExchanger.W = Wth; // start value attribution for W --> enhance convergence
  //lMTDCounterCurrentHeatExchanger.hotSide.h_out = h_hot_out;
  //lMTDCounterCurrentHeatExchanger.coldSide.h_out = h_cold_out;

  // --> direct parameters :
  //lMTDCounterCurrentHeatExchanger.K = 100;
  //lMTDCounterCurrentHeatExchanger.K_friction_cold = 1e-3;
  //lMTDCounterCurrentHeatExchanger.K_friction_hot = 1e-3;


  // *** reverse causality ***
  // heat transfer coefficient is given by an outlet temperature or enthalpy (whether hot or cold)
  lMTDCounterCurrentHeatExchanger.coldSide.T_out = 278.18;
  //lMTDCounterCurrentHeatExchanger.hotSide.T_out = 287.3;
  // pressure loss coefficient for the cold line is given by cold outlet pressure. Same for the hot line.
  lMTDCounterCurrentHeatExchanger.hotSide.P_out = 0.989e5;
  lMTDCounterCurrentHeatExchanger.coldSide.P_out =10e5;
  connect(waterSource.C_out, lMTDCounterCurrentHeatExchanger.C_cold_in)
    annotation (Line(points={{-20,116},{10,116},{10,90}}, color={63,81,181}));
  connect(waterSink.C_in, lMTDCounterCurrentHeatExchanger.C_cold_out)
    annotation (Line(points={{18,46},{10,46},{10,70.2}}, color={63,81,181}));
  connect(lMTDCounterCurrentHeatExchanger.C_hot_out, moistAirSink.C_in)
    annotation (Line(points={{19.8,80},{66,80}}, color={63,81,181}));
  connect(moistAirSource.C_out, lMTDCounterCurrentHeatExchanger.C_hot_in)
    annotation (Line(points={{-52,80},{-26,80},{-26,80.2},{0.2,80.2}}, color={63,
          81,181}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,20},{100,140}})));
end TestLMTDCounterCurrentHeatExchanger;
