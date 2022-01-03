within MetroscopeModelingLibrary.Common.Sensors;
model PressureSensor
  extends BaseSensor;

  Real P; // Absolute pressure in Pa (SI units)
  Real P_barA; // Absolute pressure in bar
  Real P_barG; // Relative (gauge) pressure in bar
  Real P_mbar; // Pressure in mbar
  Real P_psi; // Pressure in PSI


equation

  P = P_in;
  P_barA = P_in * 1e-5;
  P_mbar = P_in * 1e-2;
  P_barG = P_in*1e-5 - 1;
  P_psi = P_in*0.000145038;

  annotation (Icon(graphics={Text(
          extent={{-102,46},{114,-48}},
          textColor={0,0,0},
          textString="P")}));
end PressureSensor;
