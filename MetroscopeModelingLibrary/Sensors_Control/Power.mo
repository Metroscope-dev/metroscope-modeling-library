within MetroscopeModelingLibrary.Sensors_Control;
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
