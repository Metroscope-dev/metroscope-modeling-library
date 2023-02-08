within MetroscopeModelingLibrary.Partial.Connectors;
partial connector FluidPort
  import MetroscopeModelingLibrary.Utilities.Units;
  replaceable package Medium = Modelica.Media.Interfaces.PartialMedium "Medium model";

  replaceable flow Units.MassFlowRate Q constrainedby Units.MassFlowRate;
  Units.Pressure P(start=1e5);
  stream Units.SpecificEnthalpy h_outflow(start=1e5);
  stream Medium.MassFraction Xi_outflow[Medium.nXi];
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end FluidPort;
