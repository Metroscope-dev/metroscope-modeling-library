within MetroscopeModelingLibrary.Partial.Sensors;
partial model PressureSensor
  extends BaseSensor                                   annotation(IconMap(primitivesVisible=true));
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.InlineSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Constants;

  // All nominal values of gauge pressures are set to the absolute pressure value to avoid zero nominal value

  Real P_barG(nominal=P_0, start=P_0*Constants.Pa_to_barA - Constants.atmospheric_pressure_in_bar); // Relative (gauge) pressure in bar

  Real P_psiG(nominal = P_0*Constants.Pa_to_psiA, start = P_0*Constants.Pa_to_psiA - Constants.P0_psiG_in_psiA); // Relative (gauge) pressure in psi
  Real P_MPaG(nominal = P_0*Constants.Pa_to_MPaA, start = P_0*Constants.Pa_to_MPaA - Constants.P0_MPaG_in_MPaA); // Relative (gauge) pressure in mega pascal
  Real P_kPaG(nominal = P_0*Constants.Pa_to_kPaA, start = P_0*Constants.Pa_to_kPaA - Constants.P0_kPaG_in_kPaA); // Relative (gauge) pressure in kilo pascal
  Real P_barA(nominal = P_0*Constants.Pa_to_barA, start = P_0*Constants.Pa_to_barA, unit="bar"); // Absolute pressure in bar
  Real P_psiA(nominal = P_0*Constants.Pa_to_psiA, start = P_0*Constants.Pa_to_psiA); // Absolute pressure in psi
  Real P_MPaA(nominal = P_0*Constants.Pa_to_MPaA, start = P_0*Constants.Pa_to_MPaA); // Absolute pressure in mega pascal
  Real P_kPaA(nominal = P_0*Constants.Pa_to_kPaA, start = P_0*Constants.Pa_to_kPaA); // Absolute pressure in kilo pascal

  Real P_inHg(nominal = P_0*Constants.Pa_to_inHg, start = P_0*Constants.Pa_to_inHg); // Absolute pressure in inches of mercury
  Real P_mbar(nominal = P_0*Constants.Pa_to_mbar, start = P_0*Constants.Pa_to_mbar, unit="mbar"); // Absolute pressure in milibar

  outer parameter Boolean display_output = false "Used to switch ON or OFF output display";
  parameter String display_unit = "barA" "Specify the display unit"
    annotation(choices(choice="barA", choice="barG", choice="mbar", choice="MPaA", choice="kPaA"));

equation
  P_barA = P * Constants.Pa_to_barA;
  P_psiA = P * Constants.Pa_to_psiA;
  P_MPaA = P * Constants.Pa_to_MPaA;
  P_kPaA = P * Constants.Pa_to_kPaA;

  P_barG =P_barA - Constants.atmospheric_pressure_in_bar;
  P_psiG = P_psiA - Constants.P0_psiG_in_psiA;
  P_MPaG = P_MPaA - Constants.P0_MPaG_in_MPaA;
  P_kPaG = P_kPaA - Constants.P0_kPaG_in_kPaA;

  P_mbar = P * Constants.Pa_to_mbar;
  P_inHg = P * Constants.Pa_to_inHg;

  annotation (Icon(graphics={Text(
          extent={{-100,-160},{102,-200}},
          textColor={0,0,0},
          textString=if display_output then
                     if display_unit == "barG" then DynamicSelect("",String(P_barG)+" barG")
                     else if display_unit == "mbar" then DynamicSelect("",String(P_mbar)+" mbar")
                     else if display_unit == "MPaA" then DynamicSelect("",String(P_MPaA)+" MPaA")
                     else if display_unit == "kPaA" then DynamicSelect("",String(P_kPaA)+" kPaA")
                     else DynamicSelect("",String(P_barA)+" barA")
                     else "")}));
end PressureSensor;
