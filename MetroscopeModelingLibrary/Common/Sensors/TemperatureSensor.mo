within MetroscopeModelingLibrary.Common.Sensors;
model TemperatureSensor
  extends BaseSensor;

  Real T; // Temperature in SI Units : K
  Real T_degC;  // Temperature in degC
  Real T_degF;  // Temperature in degF


equation

  T = T_in;
  T_degC = T_in - 273.15; // Conversion to Celsius
  T_degF = (T_in-273.15)*1.8 + 32.0;  // Conversion to Farenheit

  annotation (Icon(graphics={Text(
          extent={{-108,44},{108,-50}},
          textColor={0,0,0},
          textString="T")}));
end TemperatureSensor;
