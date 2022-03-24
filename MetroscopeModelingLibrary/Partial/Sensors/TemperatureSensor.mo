within MetroscopeModelingLibrary.Partial.Sensors;
partial model TemperatureSensor
  extends Partial.BaseClasses.IsoPHFlowModel annotation(IconMap(primitivesVisible=false));
  extends FluidSensorIcon annotation(primitivesVisible=false);

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Constants;
  Units.Temperature T(start=T_0); // Temperature in SI Units : K
  Real T_degC(unit="degC", start=T_0 + Constants.T0_degC_in_K, nominal=T_0 + Constants.T0_degC_in_K);  // Temperature in degC
  Real T_degF(unit="degF",
              start=(T_0 + Constants.T0_degC_in_K)*Constants.degC_to_degF + Constants.T0_degC_in_degF,
              nominal=(T_0 + Constants.T0_degC_in_K)*Constants.degC_to_degF + Constants.T0_degC_in_degF);  // Temperature in degF
equation
  T = T_in;
  T_degC + Constants.T0_degC_in_K = T_in; // Conversion K to Celsius
  T_degF = T_degC*Constants.degC_to_degF + Constants.T0_degC_in_degF;  // Conversion Celsius to Farenheit
  annotation (Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
                   graphics={Text(
          extent={{-42,48},{40,-60}},
          textColor={0,0,0},
          textString="T")}), Diagram(coordinateSystem(extent={{-100,-100},{100,100}})));
end TemperatureSensor;
