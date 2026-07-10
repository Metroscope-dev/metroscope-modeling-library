within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model OTC
  import MetroscopeModelingLibrary.Utilities.Units;

  parameter Units.Area S = 1000;
  FlueGases.Connectors.Inlet C_hot_in                                        annotation (Placement(transformation(
          extent={{-252,2},{-232,22}}), iconTransformation(extent={{-90,-50},{
            -70,-30}})));
  FlueGases.Connectors.Outlet C_hot_out                                                                          annotation (Placement(transformation(
          extent={{50,2},{70,22}}),   iconTransformation(extent={{-90,150},{-70,
            170}})));
  WaterSteam.Connectors.Inlet C_cold_in                                          annotation (Placement(transformation(
          extent={{30,82},{50,102}}),  iconTransformation(extent={{-50,110},{
            -30,130}})));
  WaterSteam.Connectors.Outlet C_cold_out                                                                            annotation (Placement(transformation(
          extent={{-150,-78},{-130,-58}}),
                                       iconTransformation(extent={{-52,-10},{
            -32,10}})));
  Evaporator evaporator(S_parameter=false)
    annotation (Placement(transformation(extent={{-70,-38},{30,152}})));
  Superheater superheater(S_parameter=false)
    annotation (Placement(transformation(extent={{-190,62},{-90,-38}})));
  FlueGases.Pipes.FrictionPipe Kfr_hot_pipe annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-90,12})));
  Utilities.Interfaces.GenericReal Kth annotation (Placement(transformation(
          extent={{-40,76},{-48,84}}), iconTransformation(
        extent={{4,-4},{-4,4}},
        rotation=0,
        origin={-124,60})));
  Utilities.Interfaces.GenericReal Kfr_cold annotation (Placement(
        transformation(extent={{-40,76},{-48,84}}), iconTransformation(
        extent={{4,-4},{-4,4}},
        rotation=180,
        origin={-36,80})));
  Utilities.Interfaces.GenericReal Kfr_hot annotation (Placement(transformation(
          extent={{-40,76},{-48,84}}), iconTransformation(
        extent={{4,-4},{-4,4}},
        rotation=180,
        origin={-36,40})));
equation
  evaporator.S_eq + superheater.S_eq = S;
  connect(C_hot_in, C_hot_in)
    annotation (Line(points={{-242,12},{-242,12}},
                                                 color={95,95,95},
      thickness=1));
  connect(C_cold_out, C_cold_out)
    annotation (Line(points={{-140,-68},{-140,-68}}, color={28,108,200},
      thickness=1));
  connect(superheater.C_hot_out, Kfr_hot_pipe.C_in)
    annotation (Line(points={{-120,12},{-100,12}},
                                             color={95,95,95},
      thickness=1));
  connect(Kfr_hot_pipe.Kfr, Kfr_hot)
    annotation (Line(points={{-90,16},{-90,48},{-90,80},{-44,80}},
                                              color={0,0,127},
      thickness=1));
  connect(C_hot_in, superheater.C_hot_in) annotation (Line(
      points={{-242,12},{-160,12}},
      color={95,95,95},
      thickness=1));
  connect(Kfr_hot_pipe.C_out, evaporator.C_hot_in) annotation (Line(
      points={{-80,12},{-60,12}},
      color={95,95,95},
      thickness=1));
  connect(evaporator.C_hot_out, C_hot_out) annotation (Line(
      points={{20,12},{60,12}},
      color={95,95,95},
      thickness=1));
  connect(Kfr_cold, superheater.Kfr_cold) annotation (Line(
      points={{-44,80},{-104,80},{-104,-28},{-162,-28}},
      color={0,0,127},
      thickness=1));
  connect(evaporator.C_cold_in, C_cold_in) annotation (Line(
      points={{15,92},{40,92}},
      color={28,108,200},
      thickness=1));
  connect(superheater.C_cold_out, C_cold_out) annotation (Line(
      points={{-140,-38},{-140,-68}},
      color={28,108,200},
      thickness=1));
  connect(superheater.C_cold_in, evaporator.C_cold_out) annotation (Line(
      points={{-140,62},{-140,132},{-55,132}},
      color={28,108,200},
      thickness=1));
  connect(Kth, superheater.Kth)
    annotation (Line(points={{-44,80},{-104,80},{-104,52},{-162,52}},
                                                   color={0,0,127}));
  connect(Kth, evaporator.Kth) annotation (Line(points={{-44,80},{-178,80},{
          -178,80},{-72,80},{-72,-28},{-62,-28}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-160,
            -40},{0,160}},
        grid={2,2},
        initialScale=0.5),
                         graphics={
        Ellipse(
          extent={{-120,160},{-40,80}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-120,36},{-40,-42}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-120,120},{-40,-2}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{-120,120},{-120,0},{-122,0},{-122,56},{-108,48},{-146,6}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(
          points={{-156,58},{-124,-8}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(points={{-120,120},{-120,-2}}, color={0,0,0}),
        Line(points={{-26,124}}, color={0,0,0}),
        Rectangle(
          extent={{-40,120},{-40,-2}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-114,154},{-46,86}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-114,32},{-46,-36}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-114,122},{-46,-4}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-63,3},{63,-3}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          rotation=270,
          origin={-105,59}),
        Rectangle(
          extent={{-63,3},{63,-3}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          rotation=270,
          origin={-93,59}),
        Rectangle(
          extent={{-63,3},{63,-3}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          rotation=270,
          origin={-69,59}),
        Rectangle(
          extent={{-63,3},{63,-3}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          rotation=270,
          origin={-57,59}),
        Rectangle(
          extent={{-34,3},{34,-3}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          rotation=180,
          origin={-74,119}),
        Line(points={{-114,124},{-114,-4}}, color={0,0,0}),
        Line(points={{-46,116},{-46,-10}}, color={0,0,0}),
        Rectangle(
          extent={{-60,3},{60,-3}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          rotation=270,
          origin={-81,56}),
        Rectangle(
          extent={{-34,3},{34,-3}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          rotation=180,
          origin={-74,-1}),
        Line(
          points={{-108,116},{-102,116}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(
          points={{-36,106},{-28,106},{-38,106},{-30,106}},
          color={0,0,0},
          pattern=LinePattern.None),
        Line(
          points={{-108,116},{-102,116}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-96,116},{-90,116}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-84,116},{-78,116}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-72,116},{-66,116}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-60,116},{-54,116}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-108,2},{-102,2}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-96,2},{-90,2}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-84,2},{-78,2}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-72,2},{-66,2}},
          color={28,108,200},
          thickness=0.5),
        Line(
          points={{-60,2},{-54,2}},
          color={28,108,200},
          thickness=0.5),
        Line(points={{-108,118},{-108,114}}, color={0,0,0}),
        Line(points={{-108,4},{-108,0}}, color={0,0,0})}),
                                            Diagram(coordinateSystem(
          preserveAspectRatio=false, extent={{-160,-40},{0,160}},
        grid={2,2},
        initialScale=0.5)));
end OTC;
