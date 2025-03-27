within MetroscopeModelingLibrary.FlueGases.Machines;
model InletGuideVanes

  import MetroscopeModelingLibrary.Utilities.Units;

  Units.PositiveVolumeFlowRate Qv;

  BaseClasses.IsoPHFlowModel isoPHFlowModel annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Connectors.Inlet C_in annotation (Placement(transformation(extent={{-60,-10},{-40,10}}), iconTransformation(extent={{-60,-10},{-40,10}})));
  Connectors.Outlet C_out annotation (Placement(transformation(extent={{40,-10},{60,10}}), iconTransformation(extent={{40,-10},{60,10}})));
  Modelica.Blocks.Interfaces.RealInput Opening(
    unit="1",
    min=0.,
    max=1.,
    nominal=0.5)                                                                      annotation (Placement(
        transformation(extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,80}),                               iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={0,80})));
equation

  Qv = isoPHFlowModel.Qv_in;

  connect(isoPHFlowModel.C_in, C_in) annotation (Line(points={{-10,0},{-50,0},{-50,0}}, color={95,95,95}));
  connect(isoPHFlowModel.C_out, C_out) annotation (Line(points={{10,0},{30,0},{30,0},{50,0}}, color={95,95,95}));
  annotation (Icon(graphics={
        Polygon(
          points={{-36,80},{36,80},{40,76},{40,-76},{36,-80},{-36,-80},{-40,-76},{-40,76},{-36,80}},
          lineColor={95,95,95},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          smooth=Smooth.Bezier,
          lineThickness=1),
        Polygon(
          points={{53,26},{-13,52},{-33,64},{-29,44},{53,26}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          smooth=Smooth.Bezier),
        Polygon(
          points={{53,-6},{-13,20},{-33,32},{-29,12},{53,-6}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          smooth=Smooth.Bezier),
        Polygon(
          points={{53,-38},{-13,-12},{-33,0},{-29,-20},{53,-38}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          smooth=Smooth.Bezier),
        Polygon(
          points={{53,-68},{-13,-42},{-33,-30},{-29,-50},{53,-68}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          smooth=Smooth.Bezier)}));
end InletGuideVanes;
