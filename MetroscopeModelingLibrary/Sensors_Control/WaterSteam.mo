within MetroscopeModelingLibrary.Sensors_Control;
package WaterSteam
  model TemperatureSensor
    package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

    extends Partial.Sensors_Control.TemperatureSensor(
      redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.WaterSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

  end TemperatureSensor;

  model PressureSensor
    package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

    extends Partial.Sensors_Control.PressureSensor(
      redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.WaterSensorIcon;
      extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;
  end PressureSensor;

  model DeltaPressureSensor
    package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

    extends Partial.Sensors_Control.DeltaPressureSensor(
      redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
      redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

  end DeltaPressureSensor;

  model FlowSensor
    package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

    extends Partial.Sensors_Control.FlowSensor(
      redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.WaterSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlowIcon;
  end FlowSensor;
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
          lineColor={102,102,102},
          fillColor={28,108,200},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}})}));
end WaterSteam;
