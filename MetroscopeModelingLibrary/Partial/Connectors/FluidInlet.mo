within MetroscopeModelingLibrary.Partial.Connectors;
partial connector FluidInlet
  extends MetroscopeModelingLibrary.Icons.Connectors.FluidInletIcon;

  import MetroscopeModelingLibrary.Units;
  replaceable package Medium = Modelica.Media.Interfaces.PartialMedium "Medium model";

  extends MetroscopeModelingLibrary.Partial.Connectors.FluidPort(redeclare Units.PositiveMassFlowRate Q(start=500, nominal=500));
                                                                                                                               // Q out of component is negative
end FluidInlet;
