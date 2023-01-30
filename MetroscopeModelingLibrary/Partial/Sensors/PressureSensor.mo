within MetroscopeModelingLibrary.Partial.Sensors;
partial model PressureSensor
  extends MetroscopeModelingLibrary.Icons.Sensors.InlineSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.PressureIcon;
  extends BaseSensor                                   annotation(IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Constants;

  Real P_barA(unit="bar",
              nominal=P_0*Constants.Pa_to_barA,
              start=P_0*Constants.Pa_to_barA); // Absolute pressure in bar
  Real P_barG(unit="bar",
              nominal=P_0*Constants.Pa_to_barA - Constants.P0_barG_in_barA,
              start=P_0*Constants.Pa_to_barA - Constants.P0_barG_in_barA); // Relative (gauge) pressure in bar
  Real P_mbar(unit="mbar",
              nominal=Constants.Pa_to_mbar,
              start=P_0*Constants.Pa_to_mbar); // Pressure in milibar
  Real P_psiA(nominal=P_0*Constants.Pa_to_psiA,
              start=P_0*Constants.Pa_to_psiA); // Absolute pressure in psi
  Real P_psiG(nominal=P_0*Constants.Pa_to_psiA - Constants.P0_psiG_in_psiA,
              start=P_0*Constants.Pa_to_psiA - Constants.P0_psiG_in_psiA); // Relative (gauge) pressure in psi
  Real P_inHg(nominal=P_0*Constants.Pa_to_inHg,
             start=P_0*Constants.Pa_to_inHg); // Absolute pressure in inches of mercury
  Real P_MPa(nominal=P_0*Constants.Pa_to_MPa,
             start=P_0*Constants.Pa_to_MPa); // Absolute pressure in inches of mercury
  Real P_kPa(nominal=P_0*Constants.Pa_to_kPa,
             start=P_0*Constants.Pa_to_kPa); // Absolute pressure in inches of mercury
equation
  P * Constants.Pa_to_barA = P_barA;
  P * Constants.Pa_to_mbar = P_mbar;
  P * Constants.Pa_to_psiA = P_psiA;
  P * Constants.Pa_to_inHg = P_inHg;
  P * Constants.Pa_to_MPa = P_MPa;
  P * Constants.Pa_to_kPa = P_kPa;
  P_barA = P_barG + Constants.P0_barG_in_barA;
  P_psiA = P_psiG + Constants.P0_psiG_in_psiA;
end PressureSensor;
