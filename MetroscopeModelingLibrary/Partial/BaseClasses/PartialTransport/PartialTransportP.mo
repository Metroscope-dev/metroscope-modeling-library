within MetroscopeModelingLibrary.Partial.BaseClasses.PartialTransport;
partial model PartialTransportP
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Units;

  // ------ Initialization parameters ------
  parameter Units.Pressure P_in_0 = 1e5;
  parameter Units.Pressure P_out_0 = 1e5;

  // ------ Input Quantities ------
  Units.Pressure P_in(start=P_in_0) "Inlet Pressure";
  Units.Pressure P_out(start=P_out_0) "Outlet Pressure";
  Units.Pressure Pm(start=Pm_0) "Average fluid pressure";
  // Connectors
  replaceable Connectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-120,-20},{-80,20}})));
  replaceable Connectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{80,-20},{120,20}})));
protected
  parameter Units.Pressure Pm_0 = (P_in_0 + P_out_0)/2;
equation
  P_in = C_in.P;
  P_out = C_out.P;
  Pm = (P_in + P_out)/2;

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
end PartialTransportP;
