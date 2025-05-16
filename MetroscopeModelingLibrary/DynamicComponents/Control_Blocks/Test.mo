within MetroscopeModelingLibrary.DynamicComponents.Control_Blocks;
model Test
  Approach_Controller approach_Controller(step_interval=5) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Sources.Constant const(k=3) annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  Modelica.Blocks.Sources.Sine sine(
    amplitude=-2,
    f=0.01,
    offset=3.01) annotation (Placement(transformation(extent={{-60,-70},{-40,-50}})));
equation
  connect(approach_Controller.setpoint, const.y) annotation (Line(points={{-12,0},{-55,0}}, color={0,0,127}));
  connect(sine.y, approach_Controller.input_signal) annotation (Line(points={{-39,-60},{0,-60},{0,-12}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=100,
      Interval=1,
      __Dymola_Algorithm="Dassl"));
end Test;
