within MetroscopeModelingLibrary.Partial.Sensors;
partial model DeltaPressureSensor
  extends InformationSensorIcon;
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

  /*(Icon(graphics={Text(          
          extent={{-56,64},{62,-64}},
          textColor={0,0,0},
          textString="DP")}));*/
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Text(
          extent={{-106,46},{110,-48}},
          textColor={0,0,0},
          textString="DP")}),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
end DeltaPressureSensor;
