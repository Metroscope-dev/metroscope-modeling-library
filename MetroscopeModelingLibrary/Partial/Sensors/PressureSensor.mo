within MetroscopeModelingLibrary.Partial.Sensors;
partial model PressureSensor
  extends MetroscopeModelingLibrary.Icons.Sensors.FluidSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.PressureIcon;
  extends Partial.BaseClasses.IsoPHFlowModel annotation(IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Constants;

  //Inputs.InputPressure P_in;
  Real P_barA(unit="bar", nominal=1, start=P_0*Constants.Pa_to_barA); // Absolute pressure in bar
  Real P_barG(unit="bar", nominal=0, start=P_0*Constants.Pa_to_barA - Constants.P0_barG_in_barA); // Relative (gauge) pressure in bar
  Real P_mbar(unit="mbar", nominal=Constants.Pa_to_mbar, start=P_0*Constants.Pa_to_mbar); // Pressure in mbar
  Real P_psi(nominal=P_0 * Constants.Pa_to_psi, start=P_0*Constants.Pa_to_mbar); // Pressure in PSI (unit psi is not recognized by modelica)
equation
  P * Constants.Pa_to_barA = P_barA;
  P * Constants.Pa_to_mbar = P_mbar;
  P * Constants.Pa_to_psi = P_psi;
  P_barA = P_barG + Constants.P0_barG_in_barA;
end PressureSensor;
