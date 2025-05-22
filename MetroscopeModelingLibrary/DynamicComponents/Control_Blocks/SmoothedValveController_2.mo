within MetroscopeModelingLibrary.DynamicComponents.Control_Blocks;
model SmoothedValveController_2

  // Parameters
  parameter Real valve_min = 0.001;
  parameter Real valve_max = 0.99;
  parameter Real open_threshold = 3.0;
  parameter Real close_threshold = 5.0;

  parameter Real open_rate = 0.083333/60 "Rate per second (linear opening)";
  parameter Real close_rate = -0.04167/60 "Rate per second (linear closing)";

protected
  Real dydt;

public
  Modelica.Blocks.Interfaces.RealInput input_signal annotation (Placement(transformation(extent={{-140,-20},{-100,20}}), iconTransformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput valveOpening annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));
initial equation
  valveOpening = valve_min;
equation
  // Determine rate of change based on ΔT
  if input_signal < open_threshold and valveOpening < valve_max then
    dydt = open_rate;
  elseif input_signal > close_threshold and valveOpening > valve_min then
    dydt = close_rate;
  else
    dydt = 0.0;
  end if;

  // Evolve the valve position
  der(valveOpening) = dydt;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200})}),
                                                                 Diagram(coordinateSystem(preserveAspectRatio=false)));

end SmoothedValveController_2;
