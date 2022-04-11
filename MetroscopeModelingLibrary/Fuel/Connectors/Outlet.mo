within MetroscopeModelingLibrary.Fuel.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Icons.Connectors.FuelOutletIcon;

  package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
end Outlet;
