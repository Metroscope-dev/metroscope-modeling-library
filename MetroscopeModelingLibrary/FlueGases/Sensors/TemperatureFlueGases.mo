within MetroscopeModelingLibrary.FlueGases.Sensors;
model TemperatureFlueGases
    replaceable package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
    extends MetroscopeModelingLibrary.Common.Sensors.TemperatureSensor( redeclare package
              Medium = FlueGasesMedium);
    annotation (defaultComponentName = "Temperature",
    Documentation(info="<html>
<p><b>V3</b> Component now only has one port (31/05/2019)</p>
<p><b>V2</b> Creation of the heritage relationship and modification of the component accordingly (23/05/2019)</p>
<p><b>V1</b> Creation of the component and the single test (07/05/2019)</p>
<p><br><b>Parameters</b> :</p>
<table cellspacing=\"0\" cellpadding=\"2\" border=\"1\" width=\"100%\"><tr>
<td><p>Symbol</p></td>
<td><p>Meaning</p></td>
<td><p>Unit</p></td>
</tr>
<tr>
<td><p>T</p></td>
<td><p>Temperature measured by the sensor</p></td>
<td><p>K</p></td>
</tr>
</table>
<p><br><br><br><b>Direct mode</b>l : No fixed value. Gives temperature as an output</p>
<p><b>Modelling choices</b> : - No pressure loss in the component</p>
</html>"));
end TemperatureFlueGases;
