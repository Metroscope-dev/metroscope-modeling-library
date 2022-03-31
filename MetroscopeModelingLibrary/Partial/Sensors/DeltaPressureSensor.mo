within MetroscopeModelingLibrary.Partial.Sensors;
partial model DeltaPressureSensor
  extends MetroscopeModelingLibrary.Icons.Sensors.OtherSensorIcon;
  extends MetroscopeModelingLibrary.Icons.Sensors.DifferentialPressureIcon;
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  replaceable Partial.Connectors.FluidInlet C_in(redeclare package Medium = Medium);
  replaceable Partial.Connectors.FluidOutlet C_out(redeclare package Medium = Medium);

  Units.DifferentialPressure DP;
  Real DP_bar; // Pressure difference in bar
  Real DP_mbar; // Pressure difference in mbar
  Real DP_psi; // Pressure difference in PSI
equation
  // No inlet except pressure
  C_in.Q = 0;
  inStream(C_in.h_outflow) = 0;
  inStream(C_in.Xi_outflow) = zeros(Medium.nXi);

  // No outlet except pressure
  C_out.Q = 0;
  C_out.h_outflow = 0;
  C_out.Xi_outflow = zeros(Medium.nXi);

  // Conversions
  DP = C_out.P - C_in.P;
  DP_bar = DP * Constants.Pa_to_barA;
  DP_mbar = DP * Constants.Pa_to_mbar;
  DP_psi = DP * Constants.Pa_to_psi;
end DeltaPressureSensor;
