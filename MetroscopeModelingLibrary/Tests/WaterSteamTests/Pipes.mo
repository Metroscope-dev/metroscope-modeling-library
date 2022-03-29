within MetroscopeModelingLibrary.Tests.WaterSteamTests;
package Pipes

  model SteamExtractionSplitterTest
    WaterSteam.Pipes.SteamExtractionSplitter steamExtractionSplitter annotation (Placement(transformation(extent={{-27,-26.6667},{27,21.3333}})));
    WaterSteam.BoundaryConditions.WaterSource source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
    WaterSteam.BoundaryConditions.WaterSink extraction_sink annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={0,-67.3333})));
    Sensors.WaterSteam.WaterPressureSensor waterPressureSensor annotation (Placement(transformation(extent={{-76,-6},{-64,6}})));
    Sensors.WaterSteam.WaterFlowSensor waterFlowSensor annotation (Placement(transformation(extent={{-56,-6},{-44,6}})));
    WaterSteam.BoundaryConditions.WaterSink main_sink annotation (Placement(transformation(extent={{80,-10},{100,10}})));
  equation
    source.h_out = 2.65e6;
    source.P_out = 2.64e6;
    source.Q_out = -1900;

    steamExtractionSplitter.alpha = 1;
    steamExtractionSplitter.x_in = 0.8;

    extraction_sink.Q_in = 122;
    connect(steamExtractionSplitter.C_main_out, main_sink.C_in) annotation (Line(points={{28.62,-3.33333e-05},{28,-3.33333e-05},{28,0},{86,0},{86,9e-06},{85,9e-06}},
                                                                                                                                                            color={28,108,200}));
    connect(steamExtractionSplitter.C_ext_out, extraction_sink.C_in) annotation (Line(points={{0,-18.1334},{8.88178e-16,-18.1334},{8.88178e-16,-62.3333}}, color={28,108,200}));
    connect(waterPressureSensor.C_in, source.C_out) annotation (Line(points={{-76,-0.06},{-80,-0.06},{-80,0},{-85.6,0}}, color={28,108,200}));
    connect(steamExtractionSplitter.C_in, waterFlowSensor.C_out) annotation (Line(points={{-28.62,-3.33333e-05},{-44.8,-3.33333e-05},{-44.8,-0.06},{-44,-0.06}}, color={28,108,200}));
    connect(waterFlowSensor.C_in, waterPressureSensor.C_out) annotation (Line(points={{-56,-0.06},{-58,-0.06},{-58,-0.06},{-64,-0.06}}, color={28,108,200}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(lineColor = {75,138,73},
                  fillColor={255,255,255},
                  fillPattern = FillPattern.Solid,
                  extent={{-100,-100},{100,100}}),
          Polygon(lineColor = {0,0,255},
                  fillColor = {75,138,73},
                  pattern = LinePattern.None,
                  fillPattern = FillPattern.Solid,
                  points={{-36,60},{64,0},{-36,-60},{-36,60}}),
                                     Polygon(
            points={{10,-22},{10,-62},{28,-62},{48,-80},{64,-80},{58,-62},{88,-62},{88,-22},{10,-22}},
            lineColor={64,82,185},
            fillColor={236,238,248},
            fillPattern=FillPattern.Solid,
            lineThickness=0.5),
          Rectangle(
            extent={{80,-34},{96,-50}},
            lineColor={28,108,200},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{46,-76},{62,-92}},
            lineColor={28,108,200},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{0,-32},{20,-52}},
            lineColor={28,108,200},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid)}),
                                  Diagram(coordinateSystem(preserveAspectRatio=false)));
  end SteamExtractionSplitterTest;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Polygon(
          origin={8,14},
          lineColor={78,138,73},
          fillColor={78,138,73},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}}),
                               Rectangle(
          extent={{24,-22},{82,-93}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{10,-45},{36,-71}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{70,-46},{94,-70}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end Pipes;
