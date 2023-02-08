within MetroscopeModelingLibrary;
package Utilities
  extends Modelica.Icons.UtilitiesPackage;
  package Icons
    partial record KeepingScaleIcon
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

    package Connectors
      extends Icons.PackageIcon;

      partial package PartialConnectorsPackageIcon
        //extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
        extends MetroscopeModelingLibrary.Utilities.Icons.PackageIcon;
        annotation (Icon(graphics={
            Ellipse(
              extent={{-80,78},{80,-82}},
              lineColor={215,215,215},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-55,53},{55,-57}},
              lineColor={255,255,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
              Line(
                points={{-26,-2},{22,-2}},
                color={0,0,0},
                thickness=1),
              Rectangle(
                extent={{-76,24},{-26,-26}},
                lineColor={0,0,0},
                lineThickness=1,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{22,28},{80,-30}},
                lineColor={0,0,0},
                lineThickness=1,
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-60,14},{60,-14}},
              lineColor={215,215,215},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid,
              rotation=45,
                origin={0,-2})}));
      end PartialConnectorsPackageIcon;

      partial record FluidInletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={95,95,95},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid)}));
      end FluidInletIcon;

      partial record FluidOutletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end FluidOutletIcon;

      partial package WaterConnectorsPackageIcon
        //extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
        extends MetroscopeModelingLibrary.Utilities.Icons.PackageIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{20,30},{78,-28}},
                lineColor={28,108,200},
                lineThickness=1,
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-28,0},{20,0}},
                color={28,108,200},
                thickness=1),
              Rectangle(
                extent={{-78,26},{-28,-24}},
                lineColor={28,108,200},
                lineThickness=1,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end WaterConnectorsPackageIcon;

      partial record WaterInletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid)}));
      end WaterInletIcon;

      partial record WaterOutletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={28,108,200},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end WaterOutletIcon;

      partial package MoistAirConnectorsPackageIcon
        //extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
        extends MetroscopeModelingLibrary.Utilities.Icons.PackageIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{20,30},{78,-28}},
                lineColor={85,170,255},
                lineThickness=1,
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-28,0},{20,0}},
                color={85,170,255},
                thickness=1),
              Rectangle(
                extent={{-78,26},{-28,-24}},
                lineColor={85,170,255},
                lineThickness=1,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end MoistAirConnectorsPackageIcon;

      partial record MoistAirInletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={85,170,255},
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid)}));
      end MoistAirInletIcon;

      partial record MoistAirOutletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={85,170,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end MoistAirOutletIcon;

      partial package PowerConnectorsPackageIcon
        //extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
        extends MetroscopeModelingLibrary.Utilities.Icons.PackageIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{20,30},{78,-28}},
                lineColor={244,125,35},
                lineThickness=1,
                fillColor={244,125,35},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-28,0},{20,0}},
                color={244,125,35},
                thickness=1),
              Rectangle(
                extent={{-78,26},{-28,-24}},
                lineColor={244,125,35},
                lineThickness=1,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end PowerConnectorsPackageIcon;

      partial record PowerInletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={244,125,35},
                fillColor={244,125,35},
                fillPattern=FillPattern.Solid)}));
      end PowerInletIcon;

      partial record PowerOutletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={244,125,35},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end PowerOutletIcon;

      partial package FlueGasesConnectorsPackageIcon
        //extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
        extends MetroscopeModelingLibrary.Utilities.Icons.PackageIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{20,30},{78,-28}},
                lineColor={95,95,95},
                lineThickness=1,
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-28,0},{20,0}},
                color={95,95,95},
                thickness=1),
              Rectangle(
                extent={{-78,26},{-28,-24}},
                lineColor={95,95,95},
                lineThickness=1,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end FlueGasesConnectorsPackageIcon;

      partial record FlueGasesInletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid)}));
      end FlueGasesInletIcon;

      partial record FlueGasesOutletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={95,95,95},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end FlueGasesOutletIcon;

      partial package FuelConnectorsPackageIcon
        //extends MetroscopeModelingLibrary.Icons.KeepingScaleIcon;
        extends MetroscopeModelingLibrary.Utilities.Icons.PackageIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{20,30},{78,-28}},
                lineColor={213,213,0},
                lineThickness=1,
                fillColor={213,213,0},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-28,0},{20,0}},
                color={213,213,0},
                thickness=1),
              Rectangle(
                extent={{-78,26},{-28,-24}},
                lineColor={213,213,0},
                lineThickness=1,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end FuelConnectorsPackageIcon;

      partial record FuelInletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={213,213,0},
                fillColor={213,213,0},
                fillPattern=FillPattern.Solid)}));
      end FuelInletIcon;

      partial record FuelOutletIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={213,213,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}));
      end FuelOutletIcon;
    annotation (Icon(graphics={
            Line(
              points={{-30,-2},{18,-2}},
              color={0,0,0},
              thickness=1),
            Rectangle(
              extent={{-80,24},{-30,-26}},
              lineColor={0,0,0},
              lineThickness=1,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{18,28},{76,-30}},
              lineColor={0,0,0},
              lineThickness=1,
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid)}));
    end Connectors;

    package BoundaryConditions
      extends Icons.PackageIcon;

      partial record FluidSourceIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-80,60},{40,-60}},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid,
                lineThickness=0.5,
                pattern=LinePattern.None,
                lineColor={0,0,0})}));
      end FluidSourceIcon;

      partial record FluidSinkIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-40,60},{80,-60}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,50},{70,-50}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{-16,36},{57,-37}}, color={0,0,0},
                thickness=1),
              Line(points={{-16,-36},{57,37}}, color={0,0,0},
                thickness=1)}));
      end FluidSinkIcon;

      partial record WaterSourceIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-80,60},{40,-60}},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid,
                lineThickness=0.5,
                pattern=LinePattern.None,
                lineColor={0,0,0})}));
      end WaterSourceIcon;

      partial record WaterSinkIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-40,60},{80,-60}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,50},{70,-50}},
                lineColor={28,108,200},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{-16,36},{57,-37}}, color={28,108,200},
                thickness=1),
              Line(points={{-16,-36},{55,35}}, color={28,108,200},
                thickness=1)}));
      end WaterSinkIcon;

      partial record MoistAirSourceIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-80,60},{40,-60}},
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid,
                lineThickness=0.5,
                pattern=LinePattern.None,
                lineColor={0,0,0})}));
      end MoistAirSourceIcon;

      partial record MoistAirSinkIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-40,60},{80,-60}},
                lineColor={85,170,255},
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,50},{70,-50}},
                lineColor={85,170,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{-18,38},{55,-35}}, color={85,170,255},
                thickness=1),
              Line(points={{-18,-38},{55,35}}, color={85,170,255},
                thickness=1)}));
      end MoistAirSinkIcon;

      partial record PowerSourceIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-80,60},{40,-60}},
                fillColor={244,125,35},
                fillPattern=FillPattern.Solid,
                lineThickness=0.5,
                pattern=LinePattern.None,
                lineColor={0,0,0})}));
      end PowerSourceIcon;

      partial record PowerSinkIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-40,60},{80,-60}},
                lineColor={255,128,0},
                fillColor={244,125,35},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,50},{70,-50}},
                lineColor={255,128,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{-18,38},{55,-35}}, color={255,128,0},
                thickness=1),
              Line(points={{-18,-38},{55,35}}, color={255,128,0},
                thickness=1)}));
      end PowerSinkIcon;

      partial record FlueGasesSourceIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-80,60},{40,-60}},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid,
                lineThickness=0.5,
                pattern=LinePattern.None,
                lineColor={0,0,0})}));
      end FlueGasesSourceIcon;

      partial record FlueGasesSinkIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-40,60},{80,-60}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-30,50},{70,-50}},
                lineColor={95,95,95},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{-18,38},{55,-35}}, color={95,95,95},
                thickness=1),
              Line(points={{-18,-38},{55,35}}, color={95,95,95},
                thickness=1)}));
      end FlueGasesSinkIcon;

      partial record FuelSourceIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-80,60},{40,-60}},
                fillColor={213,213,0},
                fillPattern=FillPattern.Solid,
                lineThickness=0.5,
                pattern=LinePattern.None,
                lineColor={0,0,0})}));
      end FuelSourceIcon;

      partial record FuelSinkIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(graphics={
              Ellipse(
                extent={{-40,60},{80,-60}},
                lineColor={0,0,0},
                fillColor={213,213,0},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Ellipse(
                extent={{-30,50},{70,-50}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid,
                pattern=LinePattern.None),
              Line(points={{-15,35},{55,-35}}, color={213,213,0},
                thickness=0.5),
              Line(points={{-15,-35},{55,35}}, color={213,213,0},
                thickness=0.5)}));
      end FuelSinkIcon;
      annotation (Icon(graphics={
            Ellipse(
              extent={{-74,60},{46,-60}},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid,
              lineThickness=1,
              pattern=LinePattern.None,
              lineColor={0,0,0}),
          Line(
            points={{54,0},{84,0}},
            color={0,0,0},
            thickness=1),
            Rectangle(
              extent={{45,11},{67,-11}},
              lineColor={0,0,0},
              lineThickness=0.5,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}));
    end BoundaryConditions;

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

    package Pipes
      extends Icons.PackageIcon;

      partial record WaterPipeIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                   Rectangle(
                extent={{-100,30},{100,-30}},
                lineColor={28,108,200},
                fillColor={28,108,200},
                fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false)));
      end WaterPipeIcon;

      partial record MoistAirPipeIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                     Rectangle(
                extent={{-100,30},{100,-30}},
                lineColor={85,170,255},
                fillColor={85,170,255},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end MoistAirPipeIcon;

      partial record FlueGasesPipeIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                     Rectangle(
                extent={{-100,30},{100,-30}},
                lineColor={95,95,95},
                fillColor={95,95,95},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end FlueGasesPipeIcon;

      partial record FuelPipeIcon
        extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                                     Rectangle(
                extent={{-100,30},{100,-30}},
                lineColor={213,213,0},
                fillColor={213,213,0},
                fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
      end FuelPipeIcon;
      annotation (Icon(graphics={  Rectangle(
              extent={{-100,28},{100,-32}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid)}));
    end Pipes;
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

  package Units
    extends Modelica.Icons.Package;
     import Modelica.Units.SI;

    package Inputs
      import MetroscopeModelingLibrary.Utilities.Units;

      connector InputAngularVelocity = input Units.AngularVelocity;
      connector InputArea = input Units.Area;
      connector InputCv = input Units.Cv;
      connector InputCst = input Units.Cst;
      connector InputDifferentialPressure = input Units.DifferentialPressure;
      connector InputDifferentialTemperature = input Units.DifferentialTemperature;
      connector InputFraction = input Units.Fraction;
      connector InputPercentage = input Units.Percentage;
      connector InputFrictionCoefficient = input Units.FrictionCoefficient;
      connector InputHeatExchangeCoefficient = input Units.HeatExchangeCoefficient;
      connector InputHeatCapacity = input Units.HeatCapacity;
      connector InputHeight =input Units.Height;
      connector InputDifferentialHeight =input Units.DifferentialHeight;
      connector InputMassFlowRate = input Units.MassFlowRate;
      connector InputPositiveMassFlowRate = input Units.PositiveMassFlowRate;
      connector InputNegativeMassFlowRate=input Units.NegativeMassFlowRate;
      connector InputMassFraction = input Units.MassFraction;
      connector InputPower = input Units.Power;
      connector InputPositivePower = input Units.PositivePower;
      connector InputNegativePower=input Units.NegativePower;
      connector InputPressure = input Units.Pressure;
      connector InputSpecificEnthalpy = input Units.SpecificEnthalpy;
      connector InputReal = input Real;
      connector InputVelocity = input Units.Velocity;
      connector InputVolumeFlowRate = input VolumeFlowRate;
      connector InputPositiveVolumeFlowRate = input VolumeFlowRate;
      connector InputNegativeVolumeFlowRate=input VolumeFlowRate;
      connector InputTemperature = input Units.Temperature;
      connector InputYield = input Units.Yield;
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
            Line(
              points={{-48,0},{0,0}},
              color={0,140,72},
              thickness=1),
            Rectangle(
              extent={{0,30},{58,-28}},
              lineColor={0,140,72},
              lineThickness=1,
              fillColor={0,140,72},
              fillPattern=FillPattern.Solid)}));
    end Inputs;

    type AngularVelocity = SI.AngularVelocity;
    type Area = SI.Area(nominal=100);
    type AtomicMass = SI.RelativeAtomicMass(min=0);
    type MolecularMass = SI.RelativeMolecularMass(min=0);
    type Cv = Real(final quantity="Cv U.S.", final unit="m4/(s.N5)");
    type Cst = Real(final quantity="Stodola constant", min=0, nominal=1e4, start=1e4);
    type Density = SI.Density;
    type DifferentialPressure = SI.PressureDifference(nominal=1.0e5, start=1.0e5, min=-1.0e8, max=1.0e8);
    type DifferentialTemperature = SI.TemperatureDifference(nominal=100, start=0, min=-2000, max=2000);
    type FrictionCoefficient = Real(quantity="FrictionCoefficient", unit="m-4", nominal=1e-3);
    type Fraction = SI.PerUnit(min=0., max=1., nominal=0.5);
    type Percentage = Real(min=0., max=100., nominal=50);
    type GasDensity = Density(start=5, nominal = 5) "start value for gases/vapours";
    type HeatExchangeCoefficient = SI.CoefficientOfHeatTransfer(start=1e3, nominal= 1e3);
    type HeatCapacity = SI.SpecificHeatCapacity;
    type Height = SI.Position(nominal=1);
    type DifferentialHeight = SI.Length(nominal=1);
    type LiquidDensity = Density(start=1000, nominal = 1000) "start value for liquids";
    type MassFlowRate = SI.MassFlowRate(nominal=1e3);
    type PositiveMassFlowRate = SI.MassFlowRate(min=0, start=1e3, nominal=1e3);
    type NegativeMassFlowRate=SI.MassFlowRate(max=0, start=-1e3, nominal=-1e3);
    type MassFraction = SI.MassFraction;
    type Temperature = SI.Temperature(start=300, nominal=300) "in K";
    type Power = SI.Power(displayUnit="MW");
    type PositivePower =
                      SI.Power(min=0, nominal=1e5, start=1e5, displayUnit="MW");
    type NegativePower=SI.Power(max=0, nominal=-1e5, start=-1e5, displayUnit="MW");
    type Pressure = SI.AbsolutePressure "Absolute pressure, in Pa";
    type SpecificEnthalpy = SI.SpecificEnthalpy(start=5e5, nominal=5e5);
    type Velocity = SI.Velocity;
    type VolumeFlowRate = SI.VolumeFlowRate(start=0.1, nominal=0.1);
    type PositiveVolumeFlowRate =
                               VolumeFlowRate(min=0, nominal=0.1, start=0.1);
    type NegativeVolumeFlowRate=VolumeFlowRate(max=0, start=-0.1, nominal=-0.1);
    type Yield = Fraction;
    annotation(Icon(graphics={
        Polygon(
          fillColor = {128,128,128},
          pattern = LinePattern.None,
          fillPattern = FillPattern.Solid,
          points = {{-80,-40},{-80,-40},{-55,50},{-52.5,62.5},{-65,60},{-65,65},{-35,77.5},{-32.5,60},{-50,0},{-50,0},{-30,15},{-20,27.5},{-32.5,27.5},{-32.5,27.5},{-32.5,32.5},{-32.5,32.5},{2.5,32.5},{2.5,32.5},{2.5,27.5},{2.5,27.5},{-7.5,27.5},{-30,7.5},{-30,7.5},{-25,-25},{-17.5,-28.75},{-10,-25},{-5,-26.25},{-5,-32.5},{-16.25,-41.25},{-31.25,-43.75},{-40,-33.75},{-45,-5},{-45,-5},{-52.5,-10},{-52.5,-10},{-60,-40},{-60,-40}},
          smooth = Smooth.Bezier),
        Polygon(
          fillColor = {128,128,128},
          pattern = LinePattern.None,
          fillPattern = FillPattern.Solid,
          points = {{87.5,30},{62.5,30},{62.5,30},{55,33.75},{36.25,35},{16.25,25},{7.5,6.25},{11.25,-7.5},{22.5,-12.5},{22.5,-12.5},{6.25,-22.5},{6.25,-35},{16.25,-38.75},{16.25,-38.75},{21.25,-41.25},{21.25,-41.25},{45,-48.75},{47.5,-61.25},{32.5,-70},{12.5,-65},{7.5,-51.25},{21.25,-41.25},{21.25,-41.25},{16.25,-38.75},{16.25,-38.75},{6.25,-41.25},{-6.25,-50},{-3.75,-68.75},{30,-76.25},{65,-62.5},{63.75,-35},{27.5,-26.25},{22.5,-20},{27.5,-15},{27.5,-15},{30,-7.5},{30,-7.5},{27.5,-2.5},{28.75,11.25},{36.25,27.5},{47.5,30},{53.75,22.5},{51.25,8.75},{45,-6.25},{35,-11.25},{30,-7.5},{30,-7.5},{27.5,-15},{27.5,-15},{43.75,-16.25},{65,-6.25},{72.5,10},{70,20},{70,20},{80,20}},
          smooth = Smooth.Bezier)}));
  end Units;

  package Constants "Stores all constants used in MML"
    extends Modelica.Icons.Package;
    import MetroscopeModelingLibrary.Utilities.Units;
    // Gravity
    final constant Modelica.Units.SI.Acceleration g = Modelica.Constants.g_n;
    // Temperature conversions
    final constant Units.Temperature T0_degC_in_K = 273.15;
    final constant Real T0_degC_in_degF(unit="degF") = 32;
    final constant Real degC_to_degF(unit="degF/degC") = 1.8;

    // Pressure conversions
    final constant Real Pa_to_barA(unit="bar/Pa") = 1e-5;
    final constant Real Pa_to_mbar(unit="mbar/Pa") = 1e-2;
    final constant Real Pa_to_psiA(unit="1/Pa") = 0.000145038;
    final constant Real Pa_to_inHg(unit="1/Pa") = 0.0002953006;
    final constant Real Pa_to_kPa(unit="1/Pa") = 0.001;
    final constant Real Pa_to_MPa(unit="1/Pa") = 0.000001;
    final constant Real P0_barG_in_barA(unit="bar") = 1;
    final constant Real P0_psiG_in_psiA(unit="bar") = 14.50377377;

    // Mass flow conversions
    final constant Real kgs_to_th(unit="(1/h)/(kg/s)") = 3.6;
    final constant Real kgs_to_lbs(unit="1/kg") = 0.453592428;
    final constant Real kgs_to_Mlbh(unit="(1/h)/(kg/s)") = 0.0079366414387;

    // Atomic/Molecular masses
    final constant Units.AtomicMass m_H = 1.00798;
    final constant Units.AtomicMass m_C = 12.0106;
    final constant Units.AtomicMass m_O = 15.9994;
    final constant Units.MolecularMass m_CH4 = m_C + m_H*4;
    final constant Units.MolecularMass m_C2H6 = m_C*2 + m_H*6;
    final constant Units.MolecularMass m_C3H8 = m_C*3 + m_H*8;
    final constant Units.MolecularMass m_C4H10 = m_C*4 + m_H*10;
    final constant Units.MolecularMass m_CO2 = m_C + m_O*2;
    final constant Units.MolecularMass m_H2O = m_H*2 + m_O;

    // Heating values: based on ISO 6976 at 25°C
    // Methane CH4
    final constant Real hhv_molar_CH4 = 891.51 "kJ/mol";
    final constant Real hhv_mass_CH4 = hhv_molar_CH4/m_CH4 "MJ/kg";
    // Ethane C2H6
    final constant Real hhv_molar_C2H6 = 1562.06 "kJ/mol";
    final constant Real hhv_mass_C2H6 = hhv_molar_C2H6/m_C2H6 "MJ/kg";
    // Propane C3H8
    final constant Real hhv_molar_C3H8 = 2220.99 "kJ/mol";
    final constant Real hhv_mass_C3H8 = hhv_molar_C3H8/m_C3H8 "MJ/kg";
    // n-Butane C4H10
    final constant Real hhv_molar_C4H10 = 2879.63 "kJ/mol";
    final constant Real hhv_mass_C4H10 = hhv_molar_C4H10/m_C4H10 "MJ/kg";

    annotation (
    Icon(coordinateSystem(extent={{-100.0,-100.0},{100.0,100.0}}), graphics={
        Polygon(
          origin={-9.2597,25.6673},
          fillColor={102,102,102},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{48.017,11.336},{48.017,11.336},{10.766,11.336},{-25.684,10.95},{-34.944,-15.111},{-34.944,-15.111},{-32.298,-15.244},{-32.298,-15.244},{-22.112,0.168},{11.292,0.234},{48.267,-0.097},{48.267,-0.097}},
          smooth=Smooth.Bezier),
        Polygon(
          origin={-19.9923,-8.3993},
          fillColor={102,102,102},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{3.239,37.343},{3.305,37.343},{-0.399,2.683},{-16.936,-20.071},{-7.808,-28.604},{6.811,-22.519},{9.986,37.145},{9.986,37.145}},
          smooth=Smooth.Bezier),
        Polygon(
          origin={23.753,-11.5422},
          fillColor={102,102,102},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          points={{-10.873,41.478},{-10.873,41.478},{-14.048,-4.162},{-9.352,-24.8},{7.912,-24.469},{16.247,0.27},{16.247,0.27},{13.336,0.071},{13.336,0.071},{7.515,-9.983},{-3.134,-7.271},{-2.671,41.214},{-2.671,41.214}},
          smooth=Smooth.Bezier)}));
  end Constants;

  package Media "all media used in MML"
    package WaterSteamMedium
      extends Modelica.Media.Water.StandardWater annotation(IconMap(primitivesVisible=false));

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
            Ellipse(
              lineColor={102,102,102},
              fillColor={28,108,200},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-60,-60},{60,60}})}));
    end WaterSteamMedium;

    package MoistAirMedium
      extends Modelica.Media.Air.MoistAir annotation(IconMap(primitivesVisible=false));

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
            Ellipse(
              lineColor={102,102,102},
              fillColor={204,204,204},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Sphere,
              extent={{-60,-60},{60,60}}),
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
            Ellipse(
              lineColor={102,102,102},
              fillColor={85,170,255},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-60,-60},{60,60}})}));
    end MoistAirMedium;

    package FlueGasesMedium
      extends Modelica.Media.IdealGases.Common.MixtureGasNasa(
        mediumName="MediaMonomeld",
        data={Modelica.Media.IdealGases.Common.SingleGasesData.N2,
              Modelica.Media.IdealGases.Common.SingleGasesData.O2,
              Modelica.Media.IdealGases.Common.SingleGasesData.H2O,
              Modelica.Media.IdealGases.Common.SingleGasesData.CO2,
              Modelica.Media.IdealGases.Common.SingleGasesData.SO2},
        fluidConstants={Modelica.Media.IdealGases.Common.FluidData.N2,
                        Modelica.Media.IdealGases.Common.FluidData.O2,
                        Modelica.Media.IdealGases.Common.FluidData.H2O,
                        Modelica.Media.IdealGases.Common.FluidData.CO2,
                        Modelica.Media.IdealGases.Common.FluidData.SO2},
        substanceNames={"Nitrogen","Oxygen","Water","Carbondioxide","Sulfurdioxide"},
        reference_X={0.768,0.232,0.0,0.0,0.0}) annotation(IconMap(primitivesVisible=false));

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
            Ellipse(
              lineColor={102,102,102},
              fillColor={204,204,204},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Sphere,
              extent={{-60,-60},{60,60}}),
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
            Ellipse(
              lineColor={102,102,102},
              fillColor={95,95,95},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-60,-60},{60,60}})}));
    end FlueGasesMedium;

    package FuelMedium
            extends Modelica.Media.IdealGases.Common.MixtureGasNasa(
             mediumName="SimpleNaturalGas",
             data={Modelica.Media.IdealGases.Common.SingleGasesData.CH4,
            Modelica.Media.IdealGases.Common.SingleGasesData.C2H6,
            Modelica.Media.IdealGases.Common.SingleGasesData.C3H8,
            Modelica.Media.IdealGases.Common.SingleGasesData.C4H10_n_butane,
            Modelica.Media.IdealGases.Common.SingleGasesData.N2,
            Modelica.Media.IdealGases.Common.SingleGasesData.CO2},
             fluidConstants={Modelica.Media.IdealGases.Common.FluidData.CH4,
               Modelica.Media.IdealGases.Common.FluidData.C2H6,
               Modelica.Media.IdealGases.Common.FluidData.C3H8,
               Modelica.Media.IdealGases.Common.FluidData.C4H10_n_butane,
               Modelica.Media.IdealGases.Common.FluidData.N2,
               Modelica.Media.IdealGases.Common.FluidData.CO2},
             substanceNames = {"Methane","Ethane","Propane","N-Butane,","Nitrogen","Carbondioxide"},
             reference_X={0.92,0.048,0.005,0.002,0.015,0.01}) annotation(IconMap(primitivesVisible=false));

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
            Ellipse(
              lineColor={102,102,102},
              fillColor={204,204,204},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Sphere,
              extent={{-60,-60},{60,60}}),
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
            Ellipse(
              lineColor={102,102,102},
              fillColor={213,213,0},
              pattern=LinePattern.None,
              fillPattern=FillPattern.Solid,
              extent={{-60,-60},{60,60}})}));
    end FuelMedium;
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
          Ellipse(
            origin={10,10},
            fillColor={95,95,95},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            extent={{-80.0,-80.0},{-20.0,-20.0}},
          lineColor={0,0,0}),
          Ellipse(
            origin={10,10},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            extent={{0.0,-80.0},{60.0,-20.0}},
          lineColor={0,0,0},
          fillColor={213,213,0}),
          Ellipse(
            origin={10,10},
            fillColor={85,170,255},
            pattern=LinePattern.None,
            fillPattern=FillPattern.Solid,
            extent={{0.0,0.0},{60.0,60.0}},
          lineColor={0,0,0}),
          Ellipse(
            origin={10,10},
            lineColor={128,128,128},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid,
            extent={{-80.0,0.0},{-20.0,60.0}})}));
  end Media;
end Utilities;
