within MetroscopeModelingLibrary.Connectors;
partial connector FluidPort
  import MetroscopeModelingLibrary.Units;
  replaceable package Medium = Modelica.Media.Interfaces.PartialMedium "Medium model";

  flow Units.MassFlowRate Q(start=500);
  Units.Pressure P(start=1e5);
  stream Units.SpecificEnthalpy h_outflow(start=1e5);
  stream Units.MassFraction Xi_outflow[Medium.nXi];
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end FluidPort;