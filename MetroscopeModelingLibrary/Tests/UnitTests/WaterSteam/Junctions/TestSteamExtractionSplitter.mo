within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.Junctions;
model TestSteamExtractionSplitter
  import MetroscopeModelingLibrary;
  input Modelica.SIunits.AbsolutePressure P_source(start = 2.64e6);
  input Modelica.SIunits.SpecificEnthalpy h_source(start = 2.65e6);
  input Modelica.SIunits.MassFlowRate Q(start = 1900);
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source sourcesWater1
    annotation (Placement(transformation(extent={{-80,14},{-60,34}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sinkMainflow
    annotation (Placement(visible=true, transformation(
        origin={50,24},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sinkExtraction
    annotation (Placement(visible=true, transformation(
        origin={-22,-24},
        extent={{-10,-10},{10,10}},
        rotation=-90)));
    MetroscopeModelingLibrary.WaterSteam.Junctions.SteamExtractionSplitter
                                                              steamExtractionSplitter
      annotation (Placement(transformation(extent={{-52,6},{4,26}})));
equation

  // Forward causality

  sourcesWater1.P_out = P_source;
  sourcesWater1.h_out = h_source; // value fixed to have about 80% vapor at the inlet
  sourcesWater1.Q_out = -Q;

  steamExtractionSplitter.alpha = 0.972;
  sinkMainflow.h_vol = 1e5;
  sinkExtraction.h_vol = 1e5;
  sinkExtraction.Q_in = 112;


  // Reverse causality
  // To determine alpha, give the enthalpy of the extracted steam
  /*
  sourcesWater1.P_out = P_source;
  sourcesWater1.h_out = h_source; // value fixed to have about 80% vapor at the inlet
  sourcesWater1.Q_out = -Q;

  sinkMainflow.h_vol = 1e5;
  sinkExtraction.h_vol = 1e5;
  sinkExtraction.Q_in = 112;
  sinkExtraction.h_in = 2.60e6;
  */




  connect(sourcesWater1.C_out, steamExtractionSplitter.C_in) annotation (Line(
        points={{-60,24},{-60,21},{-51.44,21}},  color={238,46,47}));
  connect(steamExtractionSplitter.C_ext_out, sinkExtraction.C_in)
    annotation (Line(points={{-24,6},{-24,-14},{-22,-14}},
                                                 color={238,46,47}));
  connect(steamExtractionSplitter.C_main_out, sinkMainflow.C_in) annotation (
      Line(points={{5.12,21},{24.68,21},{24.68,24},{40,24}},     color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>This test coresponds to data from a real Nuclear Process</p>
<p>the inlet conditions are :</p>
<p>P=9.7 bar ; h=2.5e6 J/kg ; Q=834 kg/s and the corresponding x=0.86</p>
<p>for a dryer with a maximal efficiency (x=1 at the steam outlet)</p>
<p>The expected outlet are</p>
<p>for the steam outlet : P=9.7 bar ; h=2.78e6 J/kg ; Q=721kg/s</p>
<p>for the liquid outlet : P=9.7 bar; h=7.57e5 J/kg ; Q=113</p>
</html>"));
end TestSteamExtractionSplitter;
