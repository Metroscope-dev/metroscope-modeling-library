within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.HeatExchangers;
model TestLiqLiqHX
  import MetroscopeModelingLibrary;
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.LiqLiqHX liqLiqHX
    annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_cold
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_cold
    annotation (Placement(transformation(extent={{28,-10},{48,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source_hot
    annotation (Placement(transformation(extent={{-30,16},{-10,36}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_hot
    annotation (Placement(transformation(extent={{10,-36},{30,-16}})));
equation

  // Forward causality
  // The outputs are the temperatures and pressions of the water at both outlets.

  source_hot.P_out = 50e5;
  source_hot.T_out = 273.15 + 50;
  source_hot.Q_out = -100;

  source_cold.P_out = 20e5;
  source_cold.T_out = 273.15 + 100;
  source_cold.Q_out = -50;

  sink_hot.h_vol=1e6;
  sink_cold.h_vol=1e6;

  liqLiqHX.S_tot = 100;
  liqLiqHX.Kth = 500;
  liqLiqHX.Kfr_hot = 20;
  liqLiqHX.Kfr_cold = 50;


  // Reverse causality
  // Determine both Kfr by giving the outlet pressures
  // Determine the Kth by giving either the hot outlet temperature or the cold outlet temperature
  /*
  source_hot.P_out = 50e5;
  source_hot.T_out = 273.15 + 50;
  source_hot.Q_out = -100 ;
  
  source_cold.P_out = 20e5 ;
  source_cold.T_out = 273.15 + 100;
  source_cold.Q_out = -50;
  
  sink_hot.P_in = 46e5;
  sink_cold.P_in = 18e5;
  
  sink_hot.T_in = 273.15 + 60 ;
  
  sink_hot.h_vol=1e6;
  sink_cold.h_vol=1e6;
  
  liqLiqHX.S_tot = 100;
  */


  connect(source_cold.C_out, liqLiqHX.C_cold_in)
    annotation (Line(points={{-30,0},{-16,0}}, color={63,81,181}));
  connect(liqLiqHX.C_cold_out, sink_cold.C_in)
    annotation (Line(points={{16,0},{28,0}}, color={63,81,181}));
  connect(source_hot.C_out, liqLiqHX.C_hot_in)
    annotation (Line(points={{-10,26},{0,26},{0,8}}, color={63,81,181}));
  connect(sink_hot.C_in, liqLiqHX.C_hot_out)
    annotation (Line(points={{10,-26},{0,-26},{0,-8}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-40},
            {60,40}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-60,-40},{60,40}})));
end TestLiqLiqHX;
