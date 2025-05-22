within MetroscopeModelingLibrary.DynamicComponents.Control_Blocks;
model SmoothedValveController_1
  import Modelica.Units.SI.Time;

  // Parameters
  parameter Real valve_max = 0.99;
  parameter Real valve_min = 0.001;
  parameter Real valve_open_step = 0.10 "Step to open (cooling)";
  parameter Real valve_close_step = 0.05 "Step to close (heating)";
  parameter Real open_threshold = 3.0 "Open below this ΔT";
  parameter Real close_threshold = 5.0 "Close above this ΔT";
  parameter Time control_interval = 120.0 "Step interval [s]";

  protected
  discrete Real valveTarget(start=valve_min, fixed=true);

public
  Modelica.Blocks.Interfaces.RealInput input_signal annotation (Placement(transformation(extent={{-140,-20},{-100,20}}), iconTransformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput valveOpening annotation (Placement(transformation(extent={{100,-10},{120,10}}), iconTransformation(extent={{100,-10},{120,10}})));
initial equation
  valveOpening = valve_min;
equation
  // Step logic every 2 minutes
  when sample(0, control_interval) then
    if input_signal < open_threshold then
      // Too cold → open in +10% steps
      valveTarget = min(valve_max, pre(valveTarget) + valve_open_step);

    elseif input_signal > close_threshold then
      // Too hot → close in −5% steps
      valveTarget = max(valve_min, pre(valveTarget) - valve_close_step);

    else
      // In comfort band → hold
      valveTarget = pre(valveTarget);
    end if;
  end when;

  // Output valve target
  valveOpening = valveTarget;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200})}),
                                                                 Diagram(coordinateSystem(preserveAspectRatio=false)));

end SmoothedValveController_1;
