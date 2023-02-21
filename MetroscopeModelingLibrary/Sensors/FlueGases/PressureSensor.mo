within MetroscopeModelingLibrary.Sensors.FlueGases;
model PressureSensor
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.FlueGasesSensorIcon;
  extends MetroscopeModelingLibrary.Utilities.Icons.Sensors.PressureIcon;

  extends Partial.Sensors.PressureSensor(
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
    redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
    redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel isoPHFlowModel,
    redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=false));
end PressureSensor;
