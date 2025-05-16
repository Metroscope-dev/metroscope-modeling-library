within MetroscopeModelingLibrary.DynamicComponents.Control_Blocks;
block Approach_Controller
  import Modelica.Units.SI.Time;

  // Parameters
  parameter Real valve_step = 0.15 "Step change in valve opening";
  parameter Real valve_max = 0.99 "Max valve opening";
  parameter Real valve_min = 0.001 "Min valve opening";
  parameter Time step_interval = 120.0 "Interval between valve steps (seconds)";
  parameter Time control_interval = 1.0 "Sampling interval for control logic (s)";
  parameter Real hysteresis = 0.05 "Hysteresis to avoid chattering";

protected
discrete Real lastUpdate(start=0.0, fixed=true) "Last update time";
discrete Real valveOpening_internal(start=0.0, fixed=true);

public
  Modelica.Blocks.Interfaces.RealInput setpoint "Activation threshold (K)" annotation (Placement(transformation(extent={{-140,-20},{-100,20}}), iconTransformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealInput input_signal "Real delta T (K)" annotation (Placement(transformation(extent={{-140,-20},{-100,20}}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={0,-120})));
  Modelica.Blocks.Interfaces.RealOutput y "Valve opening" annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));
initial equation
//   valveOpening_internal = valve_min;
//   lastUpdate = 0.0;
equation
when sample(0, control_interval) then
  if input_signal < setpoint - hysteresis then
    valveOpening_internal = valve_max;
    lastUpdate = pre(lastUpdate);
  elseif input_signal > setpoint + hysteresis and time - pre(lastUpdate) >= step_interval then
    valveOpening_internal = max(valve_min, pre(valveOpening_internal) - valve_step);
    lastUpdate = time;
  else
    valveOpening_internal = pre(valveOpening_internal);
    lastUpdate = pre(lastUpdate);
  end if;
end when;
  y = min(max(valveOpening_internal, valve_min), valve_max);
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-100,20},{100,-20}},
          textColor={0,0,0},
          textString="Step Controller")}),                       Diagram(coordinateSystem(preserveAspectRatio=false)));

                                  // no update          // hold
                                  // also hold

end Approach_Controller;
