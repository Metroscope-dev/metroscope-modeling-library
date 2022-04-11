within MetroscopeModelingLibrary.Sensors;
package Fuel
  model FuelTemperatureSensor
    package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
    extends MetroscopeModelingLibrary.Icons.Sensors.FuelSensorIcon;
    extends MetroscopeModelingLibrary.Icons.Sensors.TemperatureIcon;

    extends Partial.Sensors.TemperatureSensor(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
      redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
  end FuelTemperatureSensor;

  model FuelPressureSensor
    package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
    extends MetroscopeModelingLibrary.Icons.Sensors.FuelSensorIcon;
    extends MetroscopeModelingLibrary.Icons.Sensors.PressureIcon;

    extends Partial.Sensors.PressureSensor(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
      redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
  end FuelPressureSensor;

  model FuelDeltaPressureSensor
    package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
    extends MetroscopeModelingLibrary.Icons.Sensors.OtherSensorIcon;
    extends MetroscopeModelingLibrary.Icons.Sensors.DeltaPressureIcon;

    extends Partial.Sensors.DeltaPressureSensor(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
      redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
  end FuelDeltaPressureSensor;

  model FuelFlowSensor
    package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
    extends MetroscopeModelingLibrary.Icons.Sensors.FuelSensorIcon;
    extends MetroscopeModelingLibrary.Icons.Sensors.FlowIcon;

    extends Partial.Sensors.FlowSensor(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
      redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=false));
  end FuelFlowSensor;
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
