within MetroscopeModelingLibrary.Fuel.Connectors;
connector Inlet
  extends MetroscopeModelingLibrary.Icons.Connectors.FuelInletIcon;

  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
end Inlet;
