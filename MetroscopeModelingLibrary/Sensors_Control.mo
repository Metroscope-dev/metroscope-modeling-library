within MetroscopeModelingLibrary;
package Sensors_Control

  package WaterSteam
    model TemperatureSensor
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      extends Partial.Sensors_Control.TemperatureSensor(
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.WaterSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

    end TemperatureSensor;

    model PressureSensor
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      extends Partial.Sensors_Control.PressureSensor(
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

        extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.WaterSensorIcon;
        extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;
    end PressureSensor;

    model DeltaPressureSensor
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      extends Partial.Sensors_Control.DeltaPressureSensor(
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
        redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

    end DeltaPressureSensor;

    model FlowSensor
      package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

      extends Partial.Sensors_Control.FlowSensor(
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.WaterSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;
    end FlowSensor;
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
  end WaterSteam;

  package FlueGases
    model TemperatureSensor
      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

      extends Partial.Sensors_Control.TemperatureSensor(
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlueGasesSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

    end TemperatureSensor;

    model PressureSensor
      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

      extends Partial.Sensors_Control.PressureSensor(
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlueGasesSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

    end PressureSensor;

    model DeltaPressureSensor
      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

      extends Partial.Sensors_Control.DeltaPressureSensor(
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
        redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

    end DeltaPressureSensor;

    model FlowSensor
      package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

      extends Partial.Sensors_Control.FlowSensor(
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlueGasesSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

    end FlowSensor;
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
  end FlueGases;

  package Fuel

    model TemperatureSensor
      package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

      extends Partial.Sensors_Control.TemperatureSensor(
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

    end TemperatureSensor;

    model PressureSensor
      package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

      extends Partial.Sensors_Control.PressureSensor(
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

    end PressureSensor;

    model DeltaPressureSensor
      package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

      extends Partial.Sensors_Control.DeltaPressureSensor(
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
        redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

    end DeltaPressureSensor;

    model FlowSensor
      package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

      extends Partial.Sensors_Control.FlowSensor(
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

    end FlowSensor;
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
  end Fuel;

  package MoistAir
    model TemperatureSensor
      package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

      extends Partial.Sensors_Control.TemperatureSensor(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

    end TemperatureSensor;

    model PressureSensor
      package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

      extends Partial.Sensors_Control.PressureSensor(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=false));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

    end PressureSensor;

    model DeltaPressureSensor
      package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

      extends Partial.Sensors_Control.DeltaPressureSensor(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
        redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

    end DeltaPressureSensor;

    model FlowSensor
      package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

      extends Partial.Sensors_Control.FlowSensor(
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
        redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
        redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
        redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

    end FlowSensor;
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
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid,
            extent={{-60,-60},{60,60}},
            pattern=LinePattern.None)}));
  end MoistAir;

  package Power
    model PowerSensor

      import MetroscopeModelingLibrary.Utilities.Units.Inputs;

      Utilities.Units.Power W;
                      // Power in W
      Real W_MW(min=0, nominal=100, start=100); // Power in MW

      // Icon parameters
      parameter String sensor_function = "Unidentified" "Specify if the sensor is a BC or used for calibration"
        annotation(choices(choice="Unidentified" "No specific function", choice="BC" "Boundary condition", choice="Calibration" "Used for calibration"));
      parameter String causality = "" "Specify which parameter is calibrated by this sensor";
      outer parameter Boolean show_causality = true "Used to show or not the causality";
      parameter String display_unit = "MW" "Specify the display unit"
        annotation(choices(choice="MW", choice="W"));
      outer parameter Boolean display_output = false "Used to switch ON or OFF output display";
      parameter String output_signal_unit = "MW" annotation(choices(choice="MW", choice="W"));

      MetroscopeModelingLibrary.Power.Connectors.Inlet C_in annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
      MetroscopeModelingLibrary.Power.Connectors.Outlet C_out annotation (Placement(transformation(extent={{88,-10},{108,10}}), iconTransformation(extent={{88,-10},{108,10}})));
      Modelica.Blocks.Interfaces.RealOutput W_sensor annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,100}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,100})));
    equation
      // Conservation of power
      C_in.W + C_out.W = 0; // C_out.W < 0 if power flows out of component, as for mass flows

      // Measure
      W = C_in.W;
      W_MW = W/1e6;

      if output_signal_unit == "MW" then
        W_sensor = W_MW;
      else
        W_sensor = W;
      end if;

      annotation (Icon(graphics={Text(
          extent={{-100,-160},{102,-200}},
          textColor={0,0,0},
          textString=if display_output then
                     if display_unit == "W" then DynamicSelect("",String(W)+" W")
                     else DynamicSelect("",String(W_MW)+" MW")
                     else ""),
         Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillColor=if sensor_function == "BC" then {238, 46, 47} elseif sensor_function == "Calibration" then {107, 175, 17} else {255, 255, 255},
            fillPattern=if sensor_function == "BC" or sensor_function == "Calibration" then FillPattern.Solid else FillPattern.None),
          Text(
            extent={{-100,160},{100,120}},
            textColor={85,170,255},
            textString="%name"),
          Text(
            extent={{-100,-120},{100,-160}},
            textColor={107,175,17},
            textString=if show_causality then "%causality" else ""),
          Line(
            points={{100,-60},{140,-60},{140,-140},{100,-140}},
            color={107,175,17},
            arrow=if causality == "" or show_causality == false then {Arrow.None,Arrow.None} else {Arrow.None,Arrow.Filled},
            thickness=0.5,
            pattern=if causality == "" or show_causality == false then LinePattern.None else LinePattern.Solid,
            smooth=Smooth.Bezier),
            Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={244,125,35},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-80,80},{80,-80}},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              pattern=LinePattern.None),
            Text(
            extent={{-60,60},{60,-60}},
            textColor={0,0,0},
            textString="W")}));
    end PowerSensor;
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
          fillColor={244,125,35},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}})}));
  end Power;

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
          extent={{-7,-7},{7,7}})}),
           Documentation(info="<html>
  <p>Licensed by Metroscope under the Modelica License 2 </p>
<p>Copyright © 2023, Metroscope.</p>
<p>This Modelica package is free software and the use is completely at your own risk; it can be redistributed and/or modified under the terms of the Modelica License 2. </p>
</html>"));
end Sensors_Control;
