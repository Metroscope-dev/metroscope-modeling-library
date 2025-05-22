within MetroscopeModelingLibrary.DynamicComponents.Control_Blocks;
model SmoothedValveController

  import Modelica.Units.SI.Time;

  // Parameters
  parameter Real valve_step = 0.15 "Step size for closing (15%)";
  parameter Real valve_max = 0.99;
  parameter Real valve_min = 0.001;
  parameter Real hysteresis = 0.05;
  parameter Time control_interval = 120.0 "Control action every 120s";
  parameter Real setpoint = 10.0 "Target threshold";

protected
  discrete Real valveTarget(start=valve_min, fixed=true);


public
  Modelica.Blocks.Interfaces.RealInput input_signal annotation (Placement(transformation(extent={{-140,-20},{-100,20}}), iconTransformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput valveOpening annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));
initial equation
  valveOpening = valve_min;
equation
  // Discrete logic, evaluated only every 120s
  when sample(0, control_interval) then
    if input_signal < setpoint - hysteresis then
      // Cold → open valve fully
      valveTarget = valve_max;
    elseif input_signal > setpoint + hysteresis then
      // Hot → step down (close)
      valveTarget = max(valve_min, pre(valveTarget) - valve_step);
    else
      // Hold last value
      valveTarget = pre(valveTarget);
    end if;
  end when;

  // Output updated valve target (no filter here for fastest response)
  valveOpening = valveTarget;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200})}),
                                                                 Diagram(coordinateSystem(preserveAspectRatio=false)));

end SmoothedValveController;
