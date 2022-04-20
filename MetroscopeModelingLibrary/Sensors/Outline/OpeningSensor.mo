within MetroscopeModelingLibrary.Sensors.Outline;
model OpeningSensor
  extends MetroscopeModelingLibrary.Icons.Sensors.OutlineSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.OpeningIcon;
  import MetroscopeModelingLibrary.Units.Inputs;

  parameter Units.Percentage Opening_pc_0(unit="1") = 15;
  Inputs.InputPercentage Opening_pc(unit="1", start=Opening_pc_0, min=0, max=100, nominal=Opening_pc_0); // Opening in percentage
  Modelica.Blocks.Interfaces.RealOutput Opening(unit="1", min=0, max=1, nominal=Opening_pc_0/100, start=Opening_pc_0/100)
    annotation (Placement(transformation(
        extent={{-27,-27},{27,27}},
        rotation=270,
        origin={1,-17}), iconTransformation(extent={{-27,-27},{27,27}},
        rotation=270,
        origin={0,-102})));
equation
  Opening_pc = Opening * 100;
end OpeningSensor;
