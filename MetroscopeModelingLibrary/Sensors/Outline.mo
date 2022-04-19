within MetroscopeModelingLibrary.Sensors;
package Outline

  model OpeningSensor
    extends MetroscopeModelingLibrary.Icons.Sensors.OutlineSensorIcon;
    extends MetroscopeModelingLibrary.Icons.Sensors.OpeningIcon;
    import MetroscopeModelingLibrary.Units.Inputs;

    parameter Real Opening_pc_0=15;
    Inputs.InputReal Opening_pc(start=Opening_pc_0, min=0, max=100, nominal=Opening_pc_0); // Opening in percentage
    Modelica.Blocks.Interfaces.RealOutput Opening(min=0, max=1, nominal=Opening_pc_0/100, start=Opening_pc_0/100)
      annotation (Placement(transformation(
          extent={{-27,-27},{27,27}},
          rotation=270,
          origin={1,-17}), iconTransformation(extent={{-27,-27},{27,27}},
          rotation=270,
          origin={0,-102})));
  equation
    Opening_pc = Opening * 100;
  end OpeningSensor;

  model VRotSensor
    extends MetroscopeModelingLibrary.Icons.Sensors.OutlineSensorIcon;
    extends MetroscopeModelingLibrary.Icons.Sensors.VRotIcon;
    import MetroscopeModelingLibrary.Units.Inputs;

    Modelica.Blocks.Interfaces.RealOutput VRot(min=0, nominal=2000, start=2000) "rotations per minute" annotation (Placement(transformation(
          extent={{-27,-27},{27,27}},
          rotation=270,
          origin={1,-17}), iconTransformation(extent={{-27,-27},{27,27}},
          rotation=270,
          origin={0,-102})));
  end VRotSensor;
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
end Outline;
