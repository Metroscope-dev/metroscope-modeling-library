within MetroscopeModelingLibrary.FlueGases.Connectors;
connector FlueGasesInlet
  extends MetroscopeModelingLibrary.Icons.Connectors.FlueGasesInletIcon;

  package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
  extends Partial.Connectors.FluidInlet(redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));
end FlueGasesInlet;
