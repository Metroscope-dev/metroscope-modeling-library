within MetroscopeModelingLibrary.WaterSteam.Junctions;
model SteamDryer
  Real Q_in_0;
  parameter Real x_out_0 = 0.99;
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  Modelica.Units.SI.MassFlowRate Q_in(start=4000) "Inlet Mass flow rate";
  Modelica.Units.SI.AbsolutePressure P_in(start=71e5) "Inlet Pressure";
  Modelica.Units.SI.SpecificEnthalpy h_in(start=1e5) "Inlet specific enthalpy";
  Modelica.Units.SI.SpecificEnthalpy hesat(start=2e6)
    "Enthalpy of saturated water";
  Modelica.Units.SI.SpecificEnthalpy hvsat(start=1e6)
    "Enthalpy of saturated vapor";
  Modelica.Units.SI.MassFraction x_out(start=x_out_0)
    "Vapor mass fraction at outlet (0 < x <= 1 and x > x_in)";
  Modelica.Units.SI.MassFraction x_in(start=0.8)
    "Vapor mass fraction at the inlet";
  Modelica.Units.SI.MassFraction x(start=x_out_0)
    "Desired vapor mass fraction at the outlet";
  replaceable Common.Partial.FlowModel liquidSide(redeclare package
      Medium = WaterSteamMedium) annotation (Placement(transformation(
        extent={{19,-8},{-19,8}},
        rotation=180,
        origin={21,-90})));
  Common.Connectors.FluidOutlet C_liquid_out(redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{100,-108},{120,-88}}),
        iconTransformation(extent={{100,-108},{120,-88}})));
  replaceable Common.Partial.FlowModel vaporSide(redeclare package
      Medium =
        WaterSteamMedium) annotation (Placement(transformation(
        extent={{-18,-8},{18,8}},
        rotation=0,
        origin={20,-30})));
  Common.Connectors.FluidOutlet C_vapor_out(redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{100,-30},{120,-10}}),
        iconTransformation(extent={{100,-30},{120,-10}})));
  Common.Connectors.FluidInlet C_in(redeclare package Medium =
        WaterSteamMedium)
    annotation (Placement(transformation(extent={{-110,-32},{-90,-12}}),
        iconTransformation(extent={{-110,-32},{-90,-12}})));
equation
  // Definition of all intermediate variables
  hvsat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P_in));
  hesat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P_in));
  Q_in = liquidSide.Q_in + vaporSide.Q_in;
  Q_in_0 = liquidSide.Q_in_0 + vaporSide.Q_in_0;
  homotopy(h_in*Q_in, h_in*Q_in_0) = homotopy(liquidSide.h_in*liquidSide.Q_in + vaporSide.h_in*vaporSide.Q_in,
                                              liquidSide.h_in*liquidSide.Q_in_0 + vaporSide.h_in*vaporSide.Q_in_0);

  x_in = noEvent(max(0,(h_in - hesat)/(hvsat-hesat)));

  x_out = noEvent(max(x_in, x));  // Efficiency determination
  // Mass balance
  //vaporSide.Q_in + vaporSide.Q_out  = 0;
  //liquidSide.Q_in +liquidSide.Q_out = 0;
  //Energy balance
  vaporSide.W + liquidSide.W = 0;
  // Saturation
  vaporSide.h_out = homotopy(x_out * hvsat + (1 - x_out) * hesat,
                             x_out_0 * hvsat + (1 - x_out_0) * hesat);
  liquidSide.h_out = hesat;
  //Mechanical Balance
  P_in = vaporSide.P_out;
  P_in = liquidSide.P_out;
  P_in = vaporSide.P_in;

  connect(vaporSide.C_out, C_vapor_out)
    annotation (Line(points={{38.36,-30},{56,-30},{56,-30},{74,-30},{74,-20},{
          110,-20}},                                 color={0,0,0}));
  connect(liquidSide.C_out, C_liquid_out)
    annotation (Line(points={{40.38,-90},{76,-90},{76,-98},{110,-98}},
                                                     color={0,0,0}));
  connect(C_in, vaporSide.C_in)
    annotation (Line(points={{-100,-22},{-50,-22},{-50,-30},{2,-30}},
                                                  color={63,81,181}));
  connect(C_in, liquidSide.C_in) annotation (Line(points={{-100,-22},{-50,-22},
          {-50,-90},{2,-90}}, color={63,81,181}));
  connect(C_in, C_in) annotation (Line(points={{-100,-22},{-100,-22},{-100,-22}},
        color={63,81,181}));
  connect(C_vapor_out, C_vapor_out) annotation (Line(points={{110,-20},{110,-18},
          {110,-20},{110,-20}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,-20}}),
                         graphics={
        Rectangle(
          extent={{-100,-20},{110,-100}},
          lineColor={0,0,0},
          fillColor={236,238,248},
          fillPattern=FillPattern.Solid,
          lineThickness=1),
        Line(
          points={{-70,-20},{-70,-80}},
          color={64,82,185},
          thickness=1),
        Polygon(
          points={{-100,-94},{108,-80},{110,-100},{-100,-100},{-100,-94}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(
          points={{-40,-38},{-40,-100}},
          color={64,82,185},
          thickness=1),
        Line(
          points={{-10,-20},{-10,-80}},
          color={64,82,185},
          thickness=1),
        Line(
          points={{20,-38},{20,-100}},
          color={64,82,185},
          thickness=1),
        Line(
          points={{50,-20},{50,-80}},
          color={64,82,185},
          thickness=1),
        Line(
          points={{80,-42},{80,-100}},
          color={64,82,185},
          thickness=1),
        Ellipse(
          extent={{-86,-32},{-76,-42}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-82,-64},{-72,-74}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-56,-72},{-46,-82}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-54,-26},{-44,-36}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-26,-68},{-16,-78}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{6,-28},{16,-38}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{32,-64},{42,-74}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{52,-32},{62,-42}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-36,-38},{-26,-48}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Ellipse(
          extent={{-64,-50},{-54,-60}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={79,188,247},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Rectangle(
          extent={{-100,-20},{110,-100}},
          lineColor={64,82,185},
          lineThickness=1)}),                                    Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
            -20}})),
    Documentation(info="<html>
<p><b>V1</b> Creation of the component and the single test (09/04/2019)</p>
<p><br><b>Parameters</b> :</p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\" width=\"100%\"><tr>
<td><p>Symbol</p></td>
<td><p>Meaning</p></td>
<td><p>Unit</p></td>
</tr>
<tr>
<td><p>x</p></td>
<td><p>Vapor fraction at the outlet</p></td>
<td></td>
</tr>
</table>
<p><br> x the vapor fraction at the outlet</p>
<p><b>Direct mode</b>l : Fixed x, Q_in, h_in, P_in</p>
<p>Output Q_out, h_out, P_out on the liquid and vapor side</p>
<p><b>Modelling choices</b> : - No pressure loss in the component</p>
<p>- The efficiency of the dryer is determined by x, the vapor fraction at the outlet, and does not depend on any other parameter.</p>
<p>- If the vapor at the inlet is already more dry than what the output should be, the dryer has no effect on it and does not humidify it.</p>
<p>- The water evacuated is at saturation</p>
<p>- At the inlet, water cannot be pure liquid, but has to be a mix of liquid and vapor. To make this component more robust, this could be changed in new versions.</p>
</html>"));
end SteamDryer;
