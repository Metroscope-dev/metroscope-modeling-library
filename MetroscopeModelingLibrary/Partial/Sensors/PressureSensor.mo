within MetroscopeModelingLibrary.Partial.Sensors;
partial model PressureSensor
  extends FluidSensorIcon;
  extends Partial.BaseClasses.IsoPHFlowModel annotation(IconMap(primitivesVisible=false));

  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;
  import MetroscopeModelingLibrary.Constants;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end PressureSensor;
