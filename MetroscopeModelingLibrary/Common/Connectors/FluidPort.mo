within MetroscopeModelingLibrary.Common.Connectors;
connector FluidPort
  //extends Modelica.Fluid.Interfaces.FluidPort
  replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
    "Medium model" annotation (choicesAllMatching=true);
  flow Modelica.SIunits.MassFlowRate Q(start=500);
  flow Modelica.SIunits.MassFlowRate Qi[Medium.nXi];
  flow Modelica.SIunits.Power H(start=5.e7);
  Modelica.SIunits.AbsolutePressure P(start=1.e5);
  Modelica.SIunits.SpecificEnthalpy h_vol(start=1.e5);
  Medium.MassFraction Xi_vol[Medium.nXi];
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end FluidPort;
