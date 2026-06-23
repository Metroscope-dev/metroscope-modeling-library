within MetroscopeModelingLibrary.WaterSteam.Pipes;
model Desuperheating
  Real Water_Q;
  Real Delta_T;
  parameter Real Gain = 1000 "To adjust between 100 and 10000";

  BaseClasses.IsoPHFlowModel
               Nozzle annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-50,0})));
  Connectors.Inlet C_in annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Connectors.Outlet C_out annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Connectors.Inlet Water_in annotation (Placement(transformation(extent={{-80,20},{-60,40}}), iconTransformation(extent={{-80,20},{-60,40}})));
  PressureCut injector annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={0,50})));
  BaseClasses.IsoPHFlowModel Spray annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=180,
        origin={30,0})));
  Utilities.Interfaces.GenericReal T_setpoint annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={60,34}), iconTransformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={60,34})));
  Utilities.Interfaces.GenericReal T_measured annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={60,34}), iconTransformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={60,-34})));
equation
  Delta_T = T_measured - T_setpoint;
  Water_Q = injector.Q;
  Water_Q = max(0.01, Gain*(Delta_T));          // Be carefull with the units !

  connect(Nozzle.C_in, C_in) annotation (Line(points={{-60,0},{-100,0}}, color={28,108,200}));
  connect(C_in, C_in) annotation (Line(points={{-100,0},{-100,0}}, color={28,108,200}));
  connect(Water_in, injector.C_in) annotation (Line(points={{-70,30},{-70,80},{0,80},{0,60}},
                                                                             color={28,108,200}));
  connect(Nozzle.C_out, Spray.C_in) annotation (Line(points={{-40,0},{20,0}}, color={28,108,200}));
  connect(Spray.C_out, C_out) annotation (Line(points={{40,0},{100,0}}, color={28,108,200}));
  connect(injector.C_out, Spray.C_in) annotation (Line(points={{0,40},{0,0},{20,0}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
                             Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-74,30},{-66,-10}},
          lineThickness=1,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-70,10},{-2,-10}},
          lineThickness=1,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          radius=0),
        Ellipse(
          extent={{-62,8},{-58,4}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-62,2},{-58,-2}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-62,-4},{-58,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-52,8},{-48,4}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-52,2},{-48,-2}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-52,-4},{-48,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-42,8},{-38,4}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-42,2},{-38,-2}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-42,-4},{-38,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-32,8},{-28,4}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-32,2},{-28,-2}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-32,-4},{-28,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-22,8},{-18,4}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-22,2},{-18,-2}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-22,-4},{-18,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-12,8},{-8,4}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-12,2},{-8,-2}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{-12,-4},{-8,-8}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          lineThickness=1,
          fillPattern=FillPattern.Solid,
          fillColor={170,213,255},
          startAngle=0,
          endAngle=360,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent={{56,-8},{72,-12}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{2,4},{18,-2}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{26,-2},{38,-6}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{24,8},{44,4}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{46,2},{58,-2}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{60,16},{72,12}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{62,4},{74,0}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false)));
end Desuperheating;
