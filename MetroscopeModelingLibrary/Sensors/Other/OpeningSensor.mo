within MetroscopeModelingLibrary.Sensors.Other;
model OpeningSensor
  extends Partial.Sensors.InformationSensorIcon;
  import MetroscopeModelingLibrary.Units.Inputs;

  Inputs.InputReal Opening_pc(min=0, max=100, nominal=50); // Opening in percent
  Modelica.Blocks.Interfaces.RealOutput Opening(min=0, max=1, nominal=0.5)
    annotation (Placement(transformation(
        extent={{-27,-27},{27,27}},
        rotation=270,
        origin={1,-17}), iconTransformation(extent={{-27,-27},{27,27}},
        rotation=270,
        origin={0,-102})));
equation
  Opening_pc = Opening * 100;  // Opening (percentage)
  annotation (Icon(coordinateSystem(preserveAspectRatio=true), graphics={
                             Text(
          extent={{-41,52},{41,-56}},
          textColor={0,0,0},
          textString="O")}),                                     Diagram(coordinateSystem(preserveAspectRatio=true)));
end OpeningSensor;
