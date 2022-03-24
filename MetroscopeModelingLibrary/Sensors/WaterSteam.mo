within MetroscopeModelingLibrary.Sensors;
package WaterSteam

  model WaterPressureSensor
    package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
    extends Partial.Sensors.PressureSensor(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidInlet C_in,
                                           redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidOutlet C_out,
                                           redeclare package Medium = WaterSteamMedium);

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end WaterPressureSensor;

  model WaterTemperatureSensor
    package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
    extends Partial.Sensors.TemperatureSensor(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidInlet C_in,
                                              redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidOutlet C_out,
                                              redeclare package Medium = WaterSteamMedium);
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end WaterTemperatureSensor;

  model WaterDeltaPressureSensor
    package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;
    extends Partial.Sensors.DeltaPressureSensor(redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidInlet C_in,
                                           redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.WaterFluidOutlet C_out,
                                           redeclare package Medium = WaterSteamMedium);
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end WaterDeltaPressureSensor;
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
end WaterSteam;
