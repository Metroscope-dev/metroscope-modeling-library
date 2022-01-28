within MetroscopeModelingLibrary.Common.Connectors;
connector FluidPort
  //extends Modelica.Fluid.Interfaces.FluidPort
  replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
    "Medium model" annotation (choicesAllMatching=true);
  flow Modelica.Units.SI.MassFlowRate Q(start=500);
  Modelica.Units.SI.AbsolutePressure P(start=1.e5);
  stream Modelica.Units.SI.SpecificEnthalpy h_outflow(start=1.e5);
  stream Medium.MassFraction Xi_outflow[Medium.nXi];
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FluidPort;
