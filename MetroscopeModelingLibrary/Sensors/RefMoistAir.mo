within MetroscopeModelingLibrary.Sensors;
package RefMoistAir
  model TemperatureSensor
    package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

    extends Partial.Sensors.TemperatureSensor(
      redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RefMoistAirSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

  end TemperatureSensor;

  model PressureSensor
    package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

    extends Partial.Sensors.PressureSensor(
      redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RefMoistAirSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

  end PressureSensor;

  model DeltaPressureSensor
    package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

    extends Partial.Sensors.DeltaPressureSensor(
      redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
      redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

  end DeltaPressureSensor;

  model FlowSensor
    package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

    extends Partial.Sensors.FlowSensor(
      redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RefMoistAirSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;

  end FlowSensor;

  model RelativeHumiditySensor
    package RefMoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.RefMoistAirMedium;

    extends MetroscopeModelingLibrary.Partial.Sensors.BaseSensor(
      redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.RefMoistAir.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = RefMoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RefMoistAirSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.RelativeHumidityIcon;

    // Relative Humidity
    parameter Real relative_humidity_0 = 0.5;
    Real relative_humidity(start=relative_humidity_0, min=0, max=1);
    Real relative_humidity_pc(start=relative_humidity_0*100, min=0, max=100);
    Real pds;
    parameter Real k_mair = RefMoistAirMedium.k_mair;

    // Display
    outer parameter Boolean display_output = false "Used to switch ON or OFF output display";

  equation
    pds = RefMoistAirMedium.Utilities.pds_pT(P, flow_model.T_in);
    flow_model.Xi = {relative_humidity*k_mair/(P/pds - relative_humidity)};
    relative_humidity_pc = relative_humidity*100;

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
          fillColor={0,255,128},
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}},
          pattern=LinePattern.None)}));
end RefMoistAir;
