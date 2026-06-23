within MetroscopeModelingLibrary.Sensors_Control.Outline;
model OpeningSensor

  import MetroscopeModelingLibrary.Utilities.Units.Inputs;

  parameter Utilities.Units.Percentage Opening_pc_0 = 15 annotation(Dialog(tab="Initialization", group="Start values"));
  Inputs.InputPercentage Opening_pc(unit="1", start=Opening_pc_0, min=0, max=100, nominal=Opening_pc_0); // Opening in percentage

  // Causality display parameters
  parameter String sensor_function = "Unidentified" "Specify if the sensor is a BC or used for calibration"
    annotation(choices(choice="Unidentified" "No specific function", choice="BC" "Boundary condition", choice="Calibration" "Used for calibration"),
    Dialog(tab="General", group="Causality display parameters"));
  parameter String causality = "" "Specify which parameter is calibrated by this sensor" annotation(Dialog(tab="General", group="Causality display parameters"));
  parameter Boolean display_output = true "Used to switch ON or OFF output display";

  // Sensor signal parameters
  parameter Real Opening_start = 15 "Write here the build value of the quantity. This value will be used in the simulation." annotation(Dialog(tab="General", group="Sensor signal parameters"));
  parameter String signal_unit = "%" "Specify the signal unit. This should be the unit of Opening_start and of the tag linked to the sensor." annotation(choices(choice="%" "percentage, between 0 and 100", choice="" "No unit, between 0 and 1"),
  Dialog(tab="General", group="Sensor signal parameters"));

  Modelica.Blocks.Interfaces.RealOutput Opening(unit="1", min=0, max=1, nominal=Opening_pc_0/100, start=Opening_pc_0/100)
    annotation (Placement(transformation(
        extent={{-27,-27},{27,27}},
        rotation=270,
        origin={0,-20}), iconTransformation(extent={{-27,-27},{27,27}},
        rotation=270,
        origin={0,-102})));
  Utilities.Interfaces.GenericReal opening_sensor(start=Opening_start) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,102}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,102})));
equation
  Opening_pc = Opening * 100;

  if signal_unit == "%" then
    opening_sensor = Opening_pc;
  else
    opening_sensor = Opening;
  end if;
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
        extent={{-100,-120},{100,-160}},
        textColor={107,175,17},
        textString=if causality <> "" then "%causality" else ""),
      Line(
        points={{100,-60},{140,-60},{140,-140},{100,-140}},
        color={107,175,17},
        arrow=if causality == "" then {Arrow.None,Arrow.None} else {Arrow.None,Arrow.Filled},
        thickness=0.5,
        pattern=if causality == "" then LinePattern.None else LinePattern.Solid,
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
