within MetroscopeModelingLibrary.Sensors.Outline;
model OpeningSensor

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  parameter Utilities.Units.Percentage Opening_pc_0(unit="1") = 15;
  Inputs.InputPercentage Opening_pc(unit="1", start=Opening_pc_0, min=0, max=100, nominal=Opening_pc_0); // Opening in percentage


  // Icon parameters
  parameter String sensor_function = "Unidentified" "Specify if the sensor is a BC or used for calibration"
    annotation(choices(choice="Unidentified" "No specific function", choice="BC" "Boundary condition", choice="Calibration" "Used for calibration"));
  parameter String causality = "" "Specify which parameter is calibrated by this sensor";
  outer parameter Boolean show_causality = true "Used to switch show or not the causality";
  outer parameter Boolean display_output = false "Used to switch ON or OFF output display";

  Modelica.Blocks.Interfaces.RealOutput Opening(unit="1", min=0, max=1, nominal=Opening_pc_0/100, start=Opening_pc_0/100)
    annotation (Placement(transformation(
        extent={{-27,-27},{27,27}},
        rotation=270,
        origin={1,-17}), iconTransformation(extent={{-27,-27},{27,27}},
        rotation=270,
        origin={0,-102})));
equation
  Opening_pc = Opening * 100;
  annotation (Icon(graphics={ Text(
          extent={{-100,200},{100,160}},
          textColor={0,0,0},
          textString=if display_output then DynamicSelect("",String(Opening_pc)+" %%")
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
          extent={{-100,100},{100,-98}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
      Text(
          extent={{-60,60},{60,-60}},
          textColor={0,0,0},
          textString="O")}));
end OpeningSensor;
