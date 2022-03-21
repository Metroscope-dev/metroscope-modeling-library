within MetroscopeModelingLibrary.Partial.BaseClasses.PartialTransport;
partial model PartialTransportXi
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Units;

  // ------ Initialization parameters ------
  parameter Units.MassFraction Xi_in_0[Medium.nXi] = zeros(Medium.nXi);
  parameter Units.MassFraction Xi_out_0[Medium.nXi] = zeros(Medium.nXi);

  Units.MassFraction Xi_in[Medium.nXi](start=Xi_in_0) "Inlet species mass fraction";
  Units.MassFraction Xi_out[Medium.nXi](start=Xi_out_0) "Outlet species mass fraction";
  // Connectors
  Connectors.FluidConnectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Connectors.FluidConnectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}})));
protected
  parameter Units.MassFraction Xim_0[Medium.nXi] = (Xi_in_0 + Xi_out_0)/2;
equation
  Xi_in = inStream(C_in.Xi_outflow);
  Xi_out = C_out.Xi_outflow;
  C_in.Xi_outflow = zeros(Medium.nXi);

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{-100,46},{100,-48}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5)}),                                  Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end PartialTransportXi;
