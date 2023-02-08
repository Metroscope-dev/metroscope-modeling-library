within MetroscopeModelingLibrary.Fuel.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Utilities.Icons.Connectors.FuelInletIcon;

  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
end Inlet;
