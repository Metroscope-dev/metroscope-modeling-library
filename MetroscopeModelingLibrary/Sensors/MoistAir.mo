within MetroscopeModelingLibrary.Sensors;
package MoistAir
  model MoistAirTemperatureSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
    extends Partial.Sensors.TemperatureSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
      redeclare package Medium = MoistAirMedium);
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end MoistAirTemperatureSensor;

  model MoistAirPressureSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
    extends Partial.Sensors.PressureSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
      redeclare package Medium = MoistAirMedium);

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end MoistAirPressureSensor;

  model MoistAirDeltaPressureSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
    extends Partial.Sensors.DeltaPressureSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
      redeclare package Medium = MoistAirMedium);
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end MoistAirDeltaPressureSensor;

  model MoistAirFlowSensor
    package MoistAirMedium = MetroscopeModelingLibrary.Media.MoistAirMedium;
    extends Partial.Sensors.FlowSensor(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirInlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.MoistAirOutlet C_out,
      redeclare package Medium = MoistAirMedium);
  end MoistAirFlowSensor;
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
          fillColor={204,204,204},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Sphere,
          extent={{-60,-60},{60,60}})}));
end MoistAir;
