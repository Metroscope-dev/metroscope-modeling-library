within MetroscopeModelingLibrary.FlueGases.Connectors;
connector FlueGasesOutlet
  extends MetroscopeModelingLibrary.Icons.Connectors.FlueGasesOutletIcon;

  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends Partial.Connectors.FluidOutlet(redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));
end FlueGasesOutlet;
