within MetroscopeModelingLibrary.Tests.UnitTests.Multifluid.HeatExchangers;
model TestWaterFlueGasesEvaporator
  extends Modelica.Icons.Example;
   MetroscopeModelingLibrary.Multifluid.HeatExchangers.WaterFlueGasesEvaporator
    waterFlueGasesEvaporator
    annotation (Placement(transformation(extent={{-10,-8},{10,12}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Source source_fluegas
    annotation (Placement(transformation(extent={{-54,-8},{-34,12}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink_flueGas
    annotation (Placement(transformation(extent={{40,-8},{60,12}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source sourceWater
    annotation (Placement(transformation(extent={{-24,24},{-4,44}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sinkWaterVapor
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-14,-30})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sinkWaterLiquid
    annotation (Placement(transformation(extent={{10,-40},{30,-20}})));
equation
  // *** inlets ***
  // water
  sourceWater.Q_out = - 108.3628;
  sourceWater.P_out =  3.6840706e5; // Psat
  sourceWater.h_out = 557016.6;
  // fg
  source_fluegas.Q_out = - 586.1577;
  source_fluegas.h_out = 493760.22;
  source_fluegas.Xi_out[1]=0.748;
  source_fluegas.Xi_out[2]=0.139;
  source_fluegas.Xi_out[3]=0.053;
  source_fluegas.Xi_out[4]=0.06;
  source_fluegas.Xi_out[5]= 1 - source_fluegas.Xi_out[1] -source_fluegas.Xi_out[2]- source_fluegas.Xi_out[3]-source_fluegas.Xi_out[4];
  source_fluegas.P_out = 1e5; //P atm

  // *** outlets ***
  // water
  sinkWaterLiquid.h_vol =592077.44;
  sinkWaterVapor.h_vol = 2734308.2;
  // fg
  sink_flueGas.Xi_vol[1]=0.748;
  sink_flueGas.Xi_vol[2]=0.139;
  sink_flueGas.Xi_vol[3]=0.053;
  sink_flueGas.Xi_vol[4]=0.06;
  sink_flueGas.Xi_vol[5]= 1 - sink_flueGas.Xi_vol[1] -sink_flueGas.Xi_vol[2]- sink_flueGas.Xi_vol[3]-sink_flueGas.Xi_vol[4];
  sink_flueGas.h_vol =445684.16;

  // *** system parameters ***
  waterFlueGasesEvaporator.S = 10; // for linearity considerations, S must be given even for reverse causality

  // FORWARD CAUSALITY
  /*
  waterFlueGasesEvaporator.K = 102000; // global heat coef
  waterFlueGasesEvaporator.K_fg = 0; //Pressure loss coeff
*/
  // REVERSE CAUSALITY
  // To compute K, specifiy Q_in or T_in for the outlet flue gases.
  sinkWaterLiquid.Q_in = 96.98174;
  //sink_flueGas.T_in = 151+273.15; --> May not converge, pay attention to the start values.
  // To compute K_fg, specify P_in for the outlet flue gas/
  sink_flueGas.P_in = 3.6840706e5;



  connect(waterFlueGasesEvaporator.C_fg_in, source_fluegas.C_out)
    annotation (Line(points={{-10,2},{-34,2}}, color={63,81,181}));
  connect(waterFlueGasesEvaporator.C_fg_out, sink_flueGas.C_in)
    annotation (Line(points={{10,2},{40,2}}, color={63,81,181}));
  connect(waterFlueGasesEvaporator.C_w_in, sourceWater.C_out)
    annotation (Line(points={{0,11},{0,34},{-4,34}}, color={63,81,181}));
  connect(waterFlueGasesEvaporator.C_w_vapor_out, sinkWaterVapor.C_in)
    annotation (Line(points={{-5,-7},{-14,-7},{-14,-20}}, color={63,81,181}));
  connect(waterFlueGasesEvaporator.C_w_liquid_out, sinkWaterLiquid.C_in)
    annotation (Line(points={{5,-7},{4,-7},{4,-30},{10,-30}}, color={63,81,181}));
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={18,-30})),
               Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end TestWaterFlueGasesEvaporator;
