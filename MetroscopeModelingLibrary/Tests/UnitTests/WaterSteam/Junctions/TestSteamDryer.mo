within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.Junctions;
model TestSteamDryer
  extends Modelica.Icons.Example;
  input Modelica.Units.SI.AbsolutePressure P_source(start=71.2e5);
  input Modelica.Units.SI.SpecificEnthalpy h_source(start=2.1e6);
  input Modelica.Units.SI.MassFlowRate Q(start=4000);

  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source sourcesWater1
    annotation (Placement(transformation(extent={{-94,14},{-74,34}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_steam
    annotation (Placement(visible=true, transformation(
        origin={50,24},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_water
    annotation (Placement(visible=true, transformation(
        origin={50,8},
        extent={{-10,-10},{10,10}},
        rotation=0)));
    MetroscopeModelingLibrary.WaterSteam.Junctions.SteamDryer steamDryer
      annotation (Placement(transformation(extent={{-32,12},{4,26}})));
equation

  // Forward Causality

  sourcesWater1.P_out = P_source;
  sourcesWater1.h_out = h_source; // value fixed to have about 80% vapor at the inlet
  sourcesWater1.Q_out = -Q;

  steamDryer.x = 0.995;

  sink_steam.h_vol = 1e6;
                     //steam outlet
  sink_water.h_vol = 1e6;
                     //water outlet


  // Reverse Causality
  // To determine x, give the outlet steam enthalpy
  /*
  sourcesWater1.P_out = P_source;
  sourcesWater1.h_out = h_source; // value fixed to have about 80% vapor at the inlet
  sourcesWater1.Q_out = -Q;
  
  sink1.h_in = 2.76351e+6;
  
  sink1.h_vol = 1e6; //steam outlet
  sink2.h_vol = 1e6; //water outlet
  */

  connect(sourcesWater1.C_out, steamDryer.C_in) annotation (Line(points={{-74,24},
          {-54,24},{-54,25.65},{-32,25.65}},     color={63,81,181}));
  connect(steamDryer.C_vapor_out, sink_steam.C_in) annotation (Line(points={{
          5.8,26},{23.9,26},{23.9,24},{40,24}}, color={63,81,181}));
  connect(steamDryer.C_liquid_out, sink_water.C_in) annotation (Line(points={{
          5.8,12.35},{23.9,12.35},{23.9,8},{40,8}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end TestSteamDryer;
