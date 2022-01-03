within MetroscopeModelingLibrary.Common.Sensors;
model DeltaPressureSensor

  replaceable package Medium =
      MetroscopeModelingLibrary.Common.Medium.PartialMedium;

  Real DP; // Pressure difference in Pa (SI units)
  Real DP_bar; // Pressure difference in bar
  Real DP_mbar; // Pressure difference in mbar
  Real DP_psi; // Pressure difference in PSI

  Connectors.FluidInlet C_in( redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-110,-8},{-90,12}})));
  Connectors.FluidOutlet C_out( redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
equation

  C_in.Q = 0;
  C_in.H = 0;
  C_in.Qi = zeros(Medium.nXi);

  C_out.Q = 0;
  C_out.H = 0;
  C_out.Qi = zeros(Medium.nXi);

  DP = C_out.P - C_in.P;
  DP_bar = DP * 1e-5;
  DP_mbar = DP * 1e-2;
  DP_psi = DP*0.000145038;

  annotation (Icon(graphics={Text(
          extent={{-106,46},{110,-48}},
          textColor={0,0,0},
          textString="DP"),
        Line(
          points={{-100,-100},{100,-100},{80,-80},{80,-120},{100,-100}},
          color={0,0,0},
          thickness=0.5),
        Polygon(
          points={{80,-120},{80,-80},{100,-100},{80,-120}},
          lineColor={0,0,0},
          lineThickness=0.5,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-82,82},{84,-84}},
          lineColor={0,0,0},
          lineThickness=0.5),
        Line(
          points={{-82,0},{-94,0}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{96,0},{84,0}},
          color={0,0,0},
          thickness=1)}));
end DeltaPressureSensor;
