within MetroscopeModelingLibrary.Utilities;
package Icons
  partial class KeepingScaleIcon
    annotation (Icon(coordinateSystem(preserveAspectRatio=true)),
          Diagram(coordinateSystem(preserveAspectRatio=true)));
  end KeepingScaleIcon;

  partial package PackageIcon

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
            radius=25.0)}));
  end PackageIcon;

  package BaseClasses
    extends Icons.PackageIcon;

    partial record BaseClassIcon "should be extended in partial base classes"
      annotation (Icon(graphics={   Rectangle(
              extent={{-100,40},{100,-40}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineThickness=1), Text(
              extent={{-66,28},{66,-20}},
              textColor={0,0,0},
              textString="%name")}));
    end BaseClassIcon;

    partial record WaterSteamBaseClassIcon "should be extended in water steam base classes"
      annotation (Icon(graphics={   Rectangle(
              extent={{-100,40},{100,-40}},
              lineColor={28,108,200},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineThickness=1), Text(
              extent={{-64,28},{68,-20}},
              textColor={28,108,200},
              textString="%name")}));
    end WaterSteamBaseClassIcon;

    partial record MoistAirBaseClassIcon "should be extended in moist air base classes"
      annotation (Diagram(graphics={Rectangle(
              extent={{-100,40},{100,-40}},
              lineColor={85,170,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineThickness=1), Text(
              extent={{-66,24},{66,-24}},
              textColor={85,170,255},
              textString="%name")}),
                                  Icon(graphics={
                                    Rectangle(
              extent={{-100,40},{100,-40}},
              lineColor={85,170,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineThickness=1)}));
    end MoistAirBaseClassIcon;

    partial record FlueGasesBaseClassIcon "should be extended in flue gases base classes"
      annotation (Icon(graphics={   Rectangle(
              extent={{-100,40},{100,-40}},
              lineColor={95,95,95},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineThickness=1), Text(
              extent={{-66,24},{66,-24}},
              textColor={95,95,95},
              textString="%name")}));
    end FlueGasesBaseClassIcon;

    partial record FuelBaseClassIcon "should be extended in flue gases base classes"
      annotation (Icon(graphics={   Rectangle(
              extent={{-100,40},{100,-40}},
              lineColor={213,213,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineThickness=1), Text(
              extent={{-66,24},{66,-24}},
              textColor={213,213,0},
              textString="%name")}));
    end FuelBaseClassIcon;
    annotation (Icon(graphics={
                  Rectangle(
                    extent={{-48,27},{48,-27}},
                    lineColor={0,0,0},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid,
                    lineThickness=1)}));
  end BaseClasses;

  package Machines
    extends Icons.PackageIcon;

    partial record PumpIcon
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      annotation (
        Diagram(coordinateSystem(
            extent={{-100,-100},{100,100}},
            grid={2,2}), graphics={
            Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={127,255,0},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,0},{80,0}}),
            Line(points={{80,0},{2,60}}),
            Line(points={{80,0},{0,-60}})}),
        Icon(coordinateSystem(
            extent={{-100,-100},{100,100}},
            grid={2,2}), graphics={
            Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={127,255,0},
              fillPattern=FillPattern.Solid),
            Line(points={{-80,0},{80,0}}),
            Line(points={{80,0},{2,60}}),
            Line(points={{80,0},{0,-60}})}));
    end PumpIcon;
    annotation (Icon(graphics={
          Ellipse(
            extent={{-60,60},{60,-60}},
            lineColor={0,0,0},
            fillColor={127,255,0},
            fillPattern=FillPattern.Solid),
          Line(points={{-32,0},{30,0}},
          color={0,0,0},
          thickness=1),
          Line(points={{30,0},{6,20}},
          color={0,0,0},
          thickness=1),
          Line(points={{30,0},{6,-20}},
          color={0,0,0},
          thickness=1)}));
  end Machines;

  partial package HeatExchangePackage
    extends Icons.PackageIcon;
     annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Polygon(
            points={{-66,-70},{-48,-70},{-48,-64},{-50,-52},{-54,-42},{-58,-32},{-60,
                -26},{-60,-14},{-60,-6},{-54,10},{-52,14},{-50,22},{-48,32},{-48,38},
                {-32,38},{-56,70},{-56,70},{-80,38},{-64,38},{-64,32},{-66,26},{-68,
                18},{-72,10},{-74,4},{-76,-4},{-76,-14},{-76,-26},{-74,-34},{-70,-46},
                {-66,-56},{-66,-70}},
            lineColor={238,46,47},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-10,-70},{8,-70},{8,-64},{6,-52},{2,-42},{-2,-32},{-4,-26},{-4,
                -14},{-4,-6},{2,10},{4,14},{6,22},{8,32},{8,38},{24,38},{0,70},{0,
                70},{-24,38},{-8,38},{-8,32},{-10,26},{-12,18},{-16,10},{-18,4},{-20,
                -4},{-20,-14},{-20,-26},{-18,-34},{-14,-46},{-10,-56},{-10,-70}},
            lineColor={238,46,47},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{48,-70},{66,-70},{66,-64},{64,-52},{60,-42},{56,-32},{54,-26},
                {54,-14},{54,-6},{60,10},{62,14},{64,22},{66,32},{66,38},{82,38},{
                58,70},{58,70},{34,38},{50,38},{50,32},{48,26},{46,18},{42,10},{40,
                4},{38,-4},{38,-14},{38,-26},{40,-34},{44,-46},{48,-56},{48,-70}},
            lineColor={238,46,47},
            fillColor={238,46,47},
            fillPattern=FillPattern.Solid)}),                      Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end HeatExchangePackage;

  package Sensors
    extends Icons.PackageIcon;

    partial record InlineSensorIcon "should be extended in partial base classes"
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      annotation (Icon(
          graphics={
            Ellipse(
              extent={{-100,100},{100,-98}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              lineThickness=0.5),
            Ellipse(
              extent={{-80,81},{80,-79}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}));
    end InlineSensorIcon;

    partial record WaterSensorIcon "should be extended in partial base classes"
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      annotation (Icon(
          graphics={
            Ellipse(
              extent={{-100,100},{100,-98}},
              lineColor={0,0,0},
              fillColor={28,108,200},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-80,81},{80,-79}},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None)}));
    end WaterSensorIcon;

    partial record MoistAirSensorIcon "should be extended in partial base classes"
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      annotation (Icon(
          graphics={
            Ellipse(
              extent={{-100,100},{100,-98}},
              lineColor={0,0,0},
              fillColor={85,170,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-80,81},{80,-79}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None)}));
    end MoistAirSensorIcon;

    partial record FlueGasesSensorIcon "should be extended in partial base classes"
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      annotation (Icon(
          graphics={
            Ellipse(
              extent={{-100,100},{100,-98}},
              lineColor={0,0,0},
              fillColor={95,95,95},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-80,81},{80,-79}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None)}));
    end FlueGasesSensorIcon;

    partial record FuelSensorIcon "should be extended in partial base classes"
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      annotation (Icon(
          graphics={
            Ellipse(
              extent={{-100,100},{100,-98}},
              lineColor={0,0,0},
              fillColor={213,213,0},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-80,81},{80,-79}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None)}));
    end FuelSensorIcon;

    partial record PowerSensorIcon "should be extended in partial base classes"
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      annotation (Icon(
          graphics={
            Ellipse(
              extent={{-100,100},{100,-98}},
              lineColor={0,0,0},
              fillColor={244,125,35},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-80,81},{80,-79}},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None)}));
    end PowerSensorIcon;

    partial record OutlineSensorIcon "should be extended in partial base classes"
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      annotation (Icon(
          graphics={
            Ellipse(
              extent={{-100,100},{100,-98}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineThickness=0.5)}));
    end OutlineSensorIcon;

    partial record AbstractSensorIcon "should be extended in partial base classes"
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      annotation (Icon(graphics={
            Polygon(
              points={{-80,60},{0,-100},{80,60},{-80,60}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              lineThickness=0.5),
            Line(points={{0,-132},{0,-100}}, color={0,0,0}),
            Polygon(
              points={{-10,-114},{0,-134},{10,-114},{0,-118},{-10,-114}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid)}));
    end AbstractSensorIcon;

    partial record AbstractDifferenceSensorIcon "should be extended in partial base classes"
      extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
      annotation (Icon(graphics={
            Line(points={{0,-16},{0,16}},    color={0,0,0},
              origin={116,0},
              rotation=90),
            Polygon(
              points={{-10,10},{0,-10},{10,10},{0,6},{-10,10}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              origin={138,0},
              rotation=90),
            Polygon(
              points={{0,100},{-100,0},{0,-100},{100,0},{0,100}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(points={{0,-16},{0,16}},    color={0,0,0},
              origin={-116,0},
              rotation=270),
            Polygon(
              points={{-10,10},{0,-10},{10,10},{0,6},{-10,10}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              origin={-138,0},
              rotation=270)}));
    end AbstractDifferenceSensorIcon;

    partial record TemperatureIcon
      annotation (Icon(graphics={Text(
              extent={{-60,60},{60,-60}},
              textColor={0,0,0},
              textString="T")}));
    end TemperatureIcon;

    partial record PressureIcon
      annotation (Icon(graphics={Text(
              extent={{-60,60},{60,-60}},
              textColor={0,0,0},
              textString="P")}));
    end PressureIcon;

    partial record DeltaPressureIcon
      annotation (Icon(graphics={Text(
              extent={{-100,56},{100,-56}},
              textColor={0,0,0},
              textString="DP"),
            Line(
              points={{-100,120},{100,120}},
              color={0,0,0},
              thickness=1),
            Polygon(
              points={{80,106},{100,120},{80,120},{80,106}},
              lineColor={0,0,0},
              lineThickness=1,
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{80,134},{100,120},{80,120},{80,134}},
              lineColor={0,0,0},
              lineThickness=1,
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid)}));
    end DeltaPressureIcon;

    partial record FlowIcon
      annotation (Icon(graphics={Text(
              extent={{-60,60},{60,-60}},
              textColor={0,0,0},
              textString="Q")}));
    end FlowIcon;

    partial record PowerIcon
      annotation (Icon(graphics={Text(
              extent={{-60,60},{60,-60}},
              textColor={0,0,0},
              textString="W")}));
    end PowerIcon;

    partial record OpeningIcon
      annotation (Icon(graphics={Text(
              extent={{-60,60},{60,-60}},
              textColor={0,0,0},
              textString="O")}));
    end OpeningIcon;

    partial record VRotIcon
      annotation (Icon(graphics={Text(
              extent={{-60,60},{60,-60}},
              textColor={0,0,0},
              textString="V")}));
    end VRotIcon;

    partial record LevelIcon
      annotation (Icon(graphics={Text(
              extent={{-60,60},{60,-60}},
              textColor={0,0,0},
              textString="L")}));
    end LevelIcon;
    annotation (Icon(graphics={
          Ellipse(
            fillColor={245,245,245},
            fillPattern=FillPattern.Solid,
            extent={{-70,-70},{70,70}}),
          Line(points={{0,70},{0,40}}),
          Line(points={{22.9,32.8},{40.2,57.3}}),
          Line(points={{-22.9,32.8},{-40.2,57.3}}),
          Line(points={{37.6,13.7},{65.8,23.9}}),
          Line(points={{-37.6,13.7},{-65.8,23.9}}),
          Ellipse(
            lineColor={64,64,64},
            fillColor={255,255,255},
            extent={{-12,-12},{12,12}}),
          Polygon(
            rotation=-17.5,
            fillColor={64,64,64},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-5.0,0.0},{-2.0,60.0},{0.0,65.0},{2.0,60.0},{5.0,0.0}}),
          Ellipse(
            fillColor={64,64,64},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            extent={{-7,-7},{7,7}})}));
  end Sensors;

  package Tests
    extends Icons.PackageIcon;

    partial package WaterSteamTestPackageIcon
      extends Icons.PackageIcon;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={28,108,200},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}),
                                                                     Diagram(coordinateSystem(preserveAspectRatio=false)));
    end WaterSteamTestPackageIcon;

    partial record WaterSteamTestIcon
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(lineColor={28,108,200},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid,
                    extent={{-100,-100},{100,100}}),
            Polygon(lineColor={0,0,255},
                    fillColor={28,108,200},
                    pattern=LinePattern.None,
                    fillPattern=FillPattern.Solid,
                    points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end WaterSteamTestIcon;

    partial package FlueGasesTestPackageIcon
      extends Icons.PackageIcon;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={95,95,95},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}),
                                                                     Diagram(coordinateSystem(preserveAspectRatio=false)));
    end FlueGasesTestPackageIcon;

    partial record FlueGasesTestIcon
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(lineColor={95,95,95},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid,
                    extent={{-100,-100},{100,100}}),
            Polygon(lineColor={0,0,255},
                    fillColor={95,95,95},
                    pattern=LinePattern.None,
                    fillPattern=FillPattern.Solid,
                    points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end FlueGasesTestIcon;

    partial package FuelTestPackageIcon
      extends Icons.PackageIcon;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={213,213,0},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}),
                                                                     Diagram(coordinateSystem(preserveAspectRatio=false)));
    end FuelTestPackageIcon;

    partial record FuelTestIcon
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(lineColor={213,213,0},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid,
                    extent={{-100,-100},{100,100}}),
            Polygon(lineColor={0,0,255},
                    fillColor={213,213,0},
                    pattern=LinePattern.None,
                    fillPattern=FillPattern.Solid,
                    points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end FuelTestIcon;

    partial package MoistAirTestPackageIcon
      extends Icons.PackageIcon;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={85,170,255},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}),
                                                                     Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MoistAirTestPackageIcon;

    partial record MoistAirTestIcon
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(lineColor={85,170,255},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid,
                    extent={{-100,-100},{100,100}}),
            Polygon(lineColor={0,0,255},
                    fillColor={85,170,255},
                    pattern=LinePattern.None,
                    fillPattern=FillPattern.Solid,
                    points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MoistAirTestIcon;

    partial package PowerTestPackageIcon
      extends Icons.PackageIcon;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={255,128,0},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}),
                                                                     Diagram(coordinateSystem(preserveAspectRatio=false)));
    end PowerTestPackageIcon;

    partial record PowerTestIcon
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(lineColor={255,128,0},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid,
                    extent={{-100,-100},{100,100}}),
            Polygon(lineColor={0,0,255},
                    fillColor={255,128,0},
                    pattern=LinePattern.None,
                    fillPattern=FillPattern.Solid,
                    points={{-36,60},{64,0},{-36,-60},{-36,60}})}), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end PowerTestIcon;

    partial package MultifluidTestPackageIcon
      extends Icons.PackageIcon;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={95,95,95},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}}),
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={213,213,0},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58,46},{-4,14},{-58,-14},{-58,46}}),
            Polygon(
              origin={8,14},
              lineColor={78,138,73},
              fillColor={28,108,200},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}),    Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MultifluidTestPackageIcon;

    partial record MultifluidTestIcon
      annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Ellipse(lineColor={0,0,0},
                    fillColor={255,255,255},
                    fillPattern=FillPattern.Solid,
                    extent={{-100,-100},{100,100}}),
            Polygon(
              origin={20,14},
              lineColor={78,138,73},
              fillColor={95,95,95},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}}),
            Polygon(
              origin={20,14},
              lineColor={78,138,73},
              fillColor={213,213,0},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58,46},{-4,14},{-58,-14},{-58,46}}),
            Polygon(
              origin={20,14},
              lineColor={78,138,73},
              fillColor={28,108,200},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              points={{-58,-14},{-2,-40},{-58,-74},{-58,-14}})}),   Diagram(coordinateSystem(preserveAspectRatio=false)));
    end MultifluidTestIcon;
    annotation (Icon(graphics={
          Polygon(
            origin={8,14},
            lineColor={78,138,73},
            fillColor={78,138,73},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-58.0,46.0},{42.0,-14.0},{-58.0,-74.0},{-58.0,46.0}})}));
  end Tests;
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
          radius=25.0),                    Polygon(
            origin={-8.167,-17},
            fillColor={128,128,128},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            points={{-15.833,20.0},{-15.833,30.0},{14.167,40.0},{24.167,20.0},{
                4.167,-30.0},{14.167,-30.0},{24.167,-30.0},{24.167,-40.0},{-5.833,
                -50.0},{-15.833,-30.0},{4.167,20.0},{-5.833,20.0}},
            smooth=Smooth.Bezier), Ellipse(
            origin={-0.5,56.5},
            fillColor={128,128,128},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            extent={{-12.5,-12.5},{12.5,12.5}})}));
end Icons;
