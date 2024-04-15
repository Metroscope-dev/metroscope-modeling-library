within MetroscopeModelingLibrary.Partial.Sensors;
partial model DeltaPressureSensor
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;
  replaceable package Medium =
      MetroscopeModelingLibrary.Partial.Media.PartialMedium;

  parameter Utilities.Units.DifferentialPressure DP_0=1e4;
  Utilities.Units.DifferentialPressure DP(start=DP_0, nominal=DP_0);
  Real DP_bar(unit="bar", start=DP_0*Utilities.Constants.Pa_to_barA); // Pressure difference in bar
  Real DP_mbar(unit="mbar", start=DP_0*Utilities.Constants.Pa_to_mbar); // Pressure difference in mbar
  Real DP_psi(start=DP_0*Utilities.Constants.Pa_to_psiA); // Pressure difference in PSI

  // Icon parameters
  parameter String sensor_function = "Unidentified" "Specify if the sensor is a BC or used for calibration"
    annotation(choices(choice="Unidentified" "No specific function", choice="BC" "Boundary condition", choice="Calibration" "Used for calibration"));
  parameter String causality = "" "Specify which parameter is calibrated by this sensor";
  outer parameter Boolean show_causality = true "Used to switch show or not the causality";
  outer parameter Boolean display_output = false "Used to switch ON or OFF output display";
  parameter String display_unit = "bar" "Specify the display unit"
    annotation(choices(choice="bar", choice="mbar", choice="psi", choice="Pa"));

  replaceable Partial.Connectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  replaceable Partial.Connectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}})));
equation
  // No inlet except pressure
  C_in.Q = 0;
  C_in.h_outflow = 0;
  C_in.Xi_outflow = zeros(Medium.nXi);

  // No outlet except pressure
  C_out.Q = 0;
  C_out.h_outflow = 0;
  C_out.Xi_outflow = zeros(Medium.nXi);

  // Conversions
  DP = C_out.P - C_in.P;
  DP_bar =DP*Utilities.Constants.Pa_to_barA;
  DP_mbar =DP*Utilities.Constants.Pa_to_mbar;
  DP_psi =DP*Utilities.Constants.Pa_to_psiA;

  annotation (Icon(graphics={Text(
          extent={{-100,-160},{102,-200}},
          textColor={0,0,0},
          textString=if display_output then
                     if display_unit == "mbar" then DynamicSelect("",String(DP_mbar)+" mbar")
                     else if display_unit == "psi" then DynamicSelect("",String(DP_psi)+" psi")
                     else if display_unit == "Pa" then DynamicSelect("",String(DP)+" Pa")
                     else DynamicSelect("",String(DP_bar)+" bar")
                     else ""),
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillColor=if sensor_function == "BC" then {238, 46, 47} elseif sensor_function == "Calibration" then {107, 175, 17} else {255, 255, 255},
            fillPattern=if sensor_function == "BC" or sensor_function == "Calibration" then FillPattern.Solid else FillPattern.None),
          Text(
            extent={{-100,160},{100,120}},
            textColor={85,170,255},
            textString="%name"),
          Text(
            extent={{-100,-120},{100,-160}},
            textColor={107,175,17},
            textString=if show_causality then "%causality" else ""),
          Line(
            points={{100,-60},{140,-60},{140,-140},{100,-140}},
            color={107,175,17},
            arrow=if causality == "" or show_causality == false then {Arrow.None,Arrow.None} else {Arrow.None,Arrow.Filled},
            thickness=0.5,
            pattern=if causality == "" or show_causality == false then LinePattern.None else LinePattern.Solid,
            smooth=Smooth.Bezier)}));
end DeltaPressureSensor;
