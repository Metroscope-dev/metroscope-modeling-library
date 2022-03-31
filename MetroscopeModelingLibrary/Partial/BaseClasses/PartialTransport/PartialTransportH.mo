within MetroscopeModelingLibrary.Partial.BaseClasses.PartialTransport;
partial model PartialTransportH
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Partial.Connectors;

  // ------ Initialization parameters ------
  parameter Units.SpecificEnthalpy h_in_0 = 1e6;
  parameter Units.SpecificEnthalpy h_out_0 = 1e6;

  // ------ Input Quantities ------
  Units.SpecificEnthalpy h_in(start=h_in_0) "Inlet specific enthalpy";
  Units.SpecificEnthalpy h_out(start=h_out_0) "Outlet specific enthalpy";
  Units.SpecificEnthalpy hm(start=hm_0) "Average specific enthalpy";

  // Connectors
  replaceable Connectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-120,-20},{-80,20}})));
  replaceable Connectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{80,-20},{120,20}})));
protected
  parameter Units.SpecificEnthalpy hm_0 = (h_in_0 + h_out_0)/2;
equation
  h_out = C_out.h_outflow;
  h_in = inStream(C_in.h_outflow);
  hm = (h_in + h_out) / 2;

  C_in.h_outflow = 1e5; // Never used, as it is assumed that there is no flow reversal

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{-100,60},{100,-62}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=1)}),                                    Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end PartialTransportH;