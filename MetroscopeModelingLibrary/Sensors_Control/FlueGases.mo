within MetroscopeModelingLibrary.Sensors_Control;
package FlueGases
  model TemperatureSensor
    package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

    extends Partial.Sensors_Control.TemperatureSensor(
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlueGasesSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.TemperatureIcon;

  end TemperatureSensor;

  model PressureSensor
    package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

    extends Partial.Sensors_Control.PressureSensor(
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlueGasesSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

  end PressureSensor;

  model DeltaPressureSensor
    package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

    extends Partial.Sensors_Control.DeltaPressureSensor(
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
      redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.OutlineSensorIcon;
    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.DeltaPressureIcon;

  end DeltaPressureSensor;

  model FlowSensor
    package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

    extends Partial.Sensors_Control.FlowSensor(
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

    extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlueGasesSensorIcon;
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
          fillColor={95,95,95},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}})}));
end FlueGases;
