within MetroscopeModelingLibrary.Sensors.MoistAir;
model RelativeHumiditySensor
  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

  extends MetroscopeModelingLibrary.Partial.Sensors.BaseSensor(
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
    redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RelativeHumidityIcon;

  // Relative Humidity
  parameter Real relative_humidity_0 = 0.5;
  Real relative_humidity(start=relative_humidity_0, min=0, max=1);
  Real relative_humidity_pc(start=relative_humidity_0*100, min=0, max=100);

  // Display
  outer parameter Boolean display_output = false "Used to switch ON or OFF output display";

equation
  flow_model.Xi[1] = MoistAirMedium.massFraction_pTphi(P, flow_model.T_in, relative_humidity);
  relative_humidity_pc = relative_humidity*100;

  annotation (Icon(graphics={Text(
        extent={{-100,-160},{102,-200}},
        textColor={0,0,0},
        textString=if display_output then
                   DynamicSelect("",String(relative_humidity_pc)+" %%")
                   else "")}));

end RelativeHumiditySensor;
