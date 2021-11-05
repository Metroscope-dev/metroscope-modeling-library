within MetroscopeModelingLibrary.Common.Sensors;
model DeltaPressureSensor
  extends BaseSensor;

  Real DP; // Pressure difference in Pa (SI units)
  Real DP_bar; // Pressure difference in bar
  Real DP_mbar; // Pressure difference in mbar
  Real DP_psi; // Pressure difference in PSI

equation

  DP = P_out - P_in;
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
          fillPattern=FillPattern.Solid)}));
end DeltaPressureSensor;
