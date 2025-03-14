within MetroscopeModelingLibrary.Sensors_Control;
package Fuel

  model TemperatureSensor
    package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

    extends Partial.Sensors_Control.TemperatureSensor(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

  end TemperatureSensor;

  model PressureSensor
    package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

    extends Partial.Sensors_Control.PressureSensor(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

  end PressureSensor;

  model DeltaPressureSensor
    package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

    extends Partial.Sensors_Control.DeltaPressureSensor(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
      redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

  end DeltaPressureSensor;

  model FlowSensor
    package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

    extends Partial.Sensors_Control.FlowSensor(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FuelSensorIcon;
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
          fillColor={204,204,204},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Sphere,
          extent={{-60,-60},{60,60}}),
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
          fillColor={204,204,204},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Sphere,
          extent={{-60,-60},{60,60}}),
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
          fillColor={213,213,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}})}));
end Fuel;
