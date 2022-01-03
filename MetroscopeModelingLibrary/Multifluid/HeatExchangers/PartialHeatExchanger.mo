within MetroscopeModelingLibrary.Multifluid.HeatExchangers;
partial model PartialHeatExchanger
  replaceable package ColdMedium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium;
  replaceable package HotMedium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium;
  Real K_friction_cold(start=1.e3) "Pressure loss coefficient";
  Real K_friction_hot(start=1.e3) "Pressure loss coefficient";
  Modelica.Units.SI.Power W(start=1e8);
  Modelica.Units.SI.MassFlowRate Q_cold(start=100) "Inlet Mass flow rate";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_cold "Singular pressure loss";
  Modelica.Units.SI.MassFlowRate Q_hot(start=100) "Inlet Mass flow rate";
  MetroscopeModelingLibrary.Common.Units.DifferentialPressure deltaP_hot "Singular pressure loss";
  replaceable Common.Partial.BasicTransportModel coldSide( redeclare package
      Medium =
        ColdMedium)
    annotation (Placement(transformation(extent={{-28,-52},{22,-16}})));
  replaceable Common.Partial.BasicTransportModel hotSide( redeclare package
      Medium =
        HotMedium)
    annotation (Placement(transformation(extent={{-28,4},{22,40}})));
  Common.Connectors.FluidInlet C_hot_in( redeclare package Medium =
        HotMedium)
    annotation (Placement(transformation(extent={{-108,-8},{-88,12}}),
        iconTransformation(extent={{-108,-8},{-88,12}})));
  Common.Connectors.FluidOutlet C_hot_out(redeclare package Medium = HotMedium)
    annotation (Placement(transformation(extent={{88,-10},{108,10}}),
        iconTransformation(extent={{88,-10},{108,10}})));
  Common.Connectors.FluidInlet C_cold_in( redeclare package Medium =
        ColdMedium)
    annotation (Placement(transformation(extent={{-10,90},{10,110}})));
  Common.Connectors.FluidOutlet C_cold_out(redeclare package Medium =
        ColdMedium)
    annotation (Placement(transformation(extent={{-10,-108},{10,-88}})));
equation
  deltaP_cold = coldSide.P_in - coldSide.P_out;
  deltaP_hot = hotSide.P_in - hotSide.P_out;
  Q_cold = coldSide.Q_in;
  Q_hot = hotSide.Q_in;
  coldSide.Q_in + coldSide.Q_out = 0;
  deltaP_cold = K_friction_cold*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_cold, coldSide.eps)/coldSide.rhom;
  coldSide.Q_in*coldSide.Xi_in = -coldSide.Q_out*coldSide.Xi_out;
  hotSide.Q_in + hotSide.Q_out = 0;
  deltaP_hot = K_friction_hot*MetroscopeModelingLibrary.Common.Functions.ThermoSquare(Q_hot, hotSide.eps)/hotSide.rhom;
  hotSide.Q_in*hotSide.Xi_in = -hotSide.Q_out*hotSide.Xi_out;
  //Energy balance
  coldSide.Q_in*coldSide.h_in + coldSide.Q_out*coldSide.h_out = -W;
  hotSide.Q_in*hotSide.h_in + hotSide.Q_out*hotSide.h_out = W;
  connect(C_hot_in, hotSide.C_in) annotation (Line(points={{-98,2},{-52,2},{-52,
          22},{-28,22}},      color={0,0,255}));
  connect(hotSide.C_out, C_hot_out) annotation (Line(points={{22.5,22},{62,22},{
          62,0},{98,0}},       color={238,46,47}));
  connect(coldSide.C_in, C_cold_in) annotation (Line(points={{-28,-34},{-68,-34},
          {-68,68},{0,68},{0,100}},color={0,0,255}));
  connect(coldSide.C_out, C_cold_out) annotation (Line(points={{22.5,-34},{48,
          -34},{48,-66},{0,-66},{0,-98}},
                                     color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100.5,15.5},{100.5,-15.5}},
          lineColor={0,0,255},
          fillColor={255,255,0},
          fillPattern=FillPattern.Backward,
          origin={73.5,0.5},
          rotation=90),
        Rectangle(
          extent={{-100,59},{100,-59}},
          lineColor={0,0,255},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid,
          origin={0,1},
          rotation=90),
        Line(
          points={{-92,-1},{-42,-1},{-22,47},{18,-47},{38,-1},{92,-1}},
          color={0,0,0},
          thickness=0.5,
          origin={-2,-1},
          rotation=-90),
        Text(
          extent={{-38,10},{38,-4}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175},
          textString="Cold Side"),
        Rectangle(
          extent={{-100,15},{100,-15}},
          lineColor={0,0,255},
          fillColor={255,255,0},
          fillPattern=FillPattern.Backward,
          origin={-74,1},
          rotation=-90),
        Text(
          extent={{-31,8},{31,-8}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175},
          textString="Hot Side",
          origin={75,-2},
          rotation=90),
        Text(
          extent={{-31,8},{31,-8}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={175,175,175},
          textString="Hot Side",
          origin={-75,0},
          rotation=90)}),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PartialHeatExchanger;
