within MetroscopeModelingLibrary.Fuel.Sensors;
model PressureDifferenceFuel
      package FuelMedium =
      MetroscopeModelingLibrary.Fuel.Medium.FuelMedium;
    extends MetroscopeModelingLibrary.Common.Sensors.PressureDifferenceSensor(
                                                                          redeclare package Medium =
        FuelMedium);
    annotation (defaultComponentName = "pressureDifference",
    Documentation(info="<html>
<p><b>V2</b> Creation of the heritage relationship and modification of the component accordingly (23/05/2019)</p>
<p><b>V1</b> Creation of the component and the single test (07/05/2019)</p>
<p><br><b>Parameters</b> :</p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\" width=\"100%\"><tr>
<td><p>Symbol</p></td>
<td><p>Meaning</p></td>
<td><p>Unit</p></td>
</tr>
<tr>
<td><p>DeltaP</p></td>
<td><p>Pressure difference measured by the sensor</p></td>
<td><p>Pa</p></td>
</tr>
</table>
<p><br><br><b>Direct mode</b>l : No fixed value. Gives pressure difference as an output</p>
<p><b>Modelling choices</b> : - No pressure loss in the component</p>
<p>- Pressure difference is measured as P_in - P_out</p>
</html>"));
end PressureDifferenceFuel;
