within MetroscopeModelingLibrary.Fuel.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Utilities.Icons.Connectors.FuelOutletIcon;

  package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
end Outlet;
