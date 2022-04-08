within MetroscopeModelingLibrary.Fuel.Connectors;
connector FuelOutlet
  extends MetroscopeModelingLibrary.Icons.Connectors.FuelOutletIcon;

  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
end FuelOutlet;
