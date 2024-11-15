within MetroscopeModelingLibrary.Partial.Sensors;
partial model TemperatureSensor
  extends BaseSensor                                   annotation(IconMap(primitivesVisible=true));
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.InlineSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Constants;

  // Initialization parameters
  parameter Units.Temperature T_0 = 300;

  Units.Temperature T(start=T_0); // Temperature in SI Units : K
  Real T_degC(unit="degC", start=T_0 +Constants.T0_degC_in_K,  nominal=T_0 +Constants.T0_degC_in_K);   // Temperature in degC
  Real T_degF(unit="degF",
              start=(T_0 +Constants.T0_degC_in_K) *Constants.degC_to_degF +Constants.T0_degC_in_degF,
              nominal=(T_0 +Constants.T0_degC_in_K) *Constants.degC_to_degF +Constants.T0_degC_in_degF);   // Temperature in degF

  parameter String display_unit = "degC" "Specify the display unit"
    annotation(choices(choice="degC", choice="K", choice="degF"));
  parameter Boolean display_output = true "Used to switch ON or OFF output display";

equation
  T =flow_model.T;
  T_degC +Constants.T0_degC_in_K  = T; // Conversion K to Celsius
  T_degF = T_degC*Constants.degC_to_degF +Constants.T0_degC_in_degF;   // Conversion Celsius to Farenheit

  annotation (Icon(graphics={Text(
        extent={{-100,-160},{102,-200}},
        textColor={0,0,0},
        textString=if display_output then
                   if display_unit == "K" then DynamicSelect("",String(T)+" K")
                   else if display_unit == "degF" then DynamicSelect("",String(T_degF)+" degF")
                   else DynamicSelect("",String(T_degC)+" degC")
                   else "")}));
end TemperatureSensor;
