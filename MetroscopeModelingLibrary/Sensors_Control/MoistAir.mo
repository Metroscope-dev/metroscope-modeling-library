within MetroscopeModelingLibrary.Sensors_Control;
package MoistAir
  model TemperatureSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

    extends Partial.Sensors_Control.TemperatureSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

  end TemperatureSensor;

  model PressureSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

    extends Partial.Sensors_Control.PressureSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=false));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

  end PressureSensor;

  model DeltaPressureSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

    extends Partial.Sensors_Control.DeltaPressureSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
      redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

  end DeltaPressureSensor;

  model FlowSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

    extends Partial.Sensors_Control.FlowSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.MoistAirSensorIcon;
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
          fillColor={85,170,255},
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}},
          pattern=LinePattern.None)}));
end MoistAir;
