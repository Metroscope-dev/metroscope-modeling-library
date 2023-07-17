within MetroscopeModelingLibrary.Utilities.Icons;
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
          Ellipse(
            extent={{-118,60},{-78,20}},
            lineColor={0,0,0},
            lineThickness=1,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{80,60},{120,20}},
            lineColor={0,0,0},
            lineThickness=1,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-2,14},{-2,-14},{2,-14},{2,14},{-2,14}},
            lineColor={0,0,0},
            lineThickness=1,
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid,
            origin={-98,40},
            rotation=90),
          Polygon(
            points={{-2,14},{-2,-14},{2,-14},{2,14},{-2,14}},
            lineColor={0,0,0},
            lineThickness=1,
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid,
            origin={100,40},
            rotation=180),
          Polygon(
            points={{-2,14},{-2,-14},{2,-14},{2,14},{-2,14}},
            lineColor={0,0,0},
            lineThickness=1,
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid,
            origin={100,40},
            rotation=90)}));
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
