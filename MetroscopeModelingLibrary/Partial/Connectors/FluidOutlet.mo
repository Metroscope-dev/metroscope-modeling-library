within MetroscopeModelingLibrary.Partial.Connectors;
partial connector FluidOutlet
  extends MetroscopeModelingLibrary.Utilities.Icons.Connectors.FluidOutletIcon;

  import MetroscopeModelingLibrary.Utilities.Units;
  replaceable package Medium = Modelica.Media.Interfaces.PartialMedium "Medium model";

  extends MetroscopeModelingLibrary.Partial.Connectors.FluidPort(redeclare Units.NegativeMassFlowRate Q(start=-500, nominal=-500));
                                                                                                                                  // Q out of component is negative
end FluidOutlet;
