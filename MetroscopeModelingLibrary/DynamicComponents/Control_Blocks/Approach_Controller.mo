within MetroscopeModelingLibrary.DynamicComponents.Control_Blocks;
block Approach_Controller
  import Modelica.Units.SI.Time;

  // Parameters
  parameter Real valve_step = 0.15;
  parameter Real valve_max = 0.99;
  parameter Real valve_min = 0.001;
  parameter Real step_interval = 120.0;
  parameter Real control_interval = 5.0;
  parameter Real hysteresis = 0.05;

protected
  discrete Integer stepCount(start=0);
  discrete Real valveOpening_internal(start=valve_min);

public
  Modelica.Blocks.Interfaces.RealInput setpoint "Activation threshold (K)" annotation (Placement(transformation(extent={{-140,-20},{-100,20}}), iconTransformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealInput input_signal "Real delta T (K)" annotation (Placement(transformation(extent={{-140,-20},{-100,20}}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Modelica.Blocks.Interfaces.RealOutput y "Valve opening" annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));


equation
  when sample(0, control_interval) then
    // Update valve opening
    if input_signal < setpoint - hysteresis then
      valveOpening_internal = valve_max;
      stepCount = 0;
    elseif input_signal > setpoint + hysteresis and pre(stepCount)*control_interval >= step_interval then
      valveOpening_internal = max(valve_min, pre(valveOpening_internal) - valve_step);
      stepCount = 0;
    else
      valveOpening_internal = pre(valveOpening_internal);
      stepCount = pre(stepCount) + 1;
    end if;
  end when;

  y = valveOpening_internal;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-100,20},{100,-20}},
          textColor={0,0,0},
          textString="Step Controller")}),                       Diagram(coordinateSystem(preserveAspectRatio=false)));


end Approach_Controller;
