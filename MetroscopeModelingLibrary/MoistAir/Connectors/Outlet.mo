within MetroscopeModelingLibrary.MoistAir.Connectors;
connector Outlet
  extends MetroscopeModelingLibrary.Utilities.Icons.Connectors.MoistAirOutletIcon;

  package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium =
        MoistAirMedium)                                                             annotation(IconMap(primitivesVisible=false));
end Outlet;
