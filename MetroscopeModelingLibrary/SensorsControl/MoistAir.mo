within MetroscopeModelingLibrary.SensorsControl;
package MoistAir

  model TemperatureSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

    extends Partial.SensorsControl.TemperatureSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

  end TemperatureSensor;

  model PressureSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

    extends Partial.SensorsControl.PressureSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

  end PressureSensor;

  model DeltaPressureSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

    extends Partial.SensorsControl.DeltaPressureSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
      redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

  end DeltaPressureSensor;

  model FlowSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

    extends Partial.SensorsControl.FlowSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

  end FlowSensor;

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
    parameter Real relative_humidity_0 = 0.5 annotation(Dialog(tab="Initialization", group="Start values"));
    Real relative_humidity(start=relative_humidity_0, min=0, max=1);
    Real relative_humidity_pc(start=relative_humidity_0*100, min=0, max=100);

    // Sensor signal parameters
    parameter Real H_start = 50 "Write here the build value of the quantity. This value will be used in the simulation." annotation(Dialog(tab="General", group="Sensor signal parameters"));
    parameter String display_unit = "%" "Specify the display unit" annotation(choices(choice="", choice="%"),
    Dialog(tab="General", group="Sensor signal parameters"));
    parameter String signal_unit = "%" "Specify the signal unit. This should be the unit of H_start and of the tag linked to the sensor." annotation (choices(choice="1", choice="%"),
    Dialog(tab="General", group="Sensor signal parameters"));

    Utilities.Interfaces.GenericReal      H_sensor(start=H_start) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,100})));
  equation
    flow_model.Xi[1] = MoistAirMedium.massFraction_pTphi(P, flow_model.T_in, relative_humidity);
    relative_humidity_pc = relative_humidity*100;

    if signal_unit == "" then
      H_sensor = relative_humidity;
    elseif signal_unit == "%" then
      H_sensor = relative_humidity_pc;
    end if;

    annotation (Icon(graphics={Text(
          extent={{-100,-160},{102,-200}},
          textColor={0,0,0},
          textString=if display_output then
                     DynamicSelect("",String(relative_humidity_pc)+" %%")
                     else "")}));

  end RelativeHumiditySensor;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Ellipse(
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}},
          pattern=LinePattern.None)}));
end MoistAir;
