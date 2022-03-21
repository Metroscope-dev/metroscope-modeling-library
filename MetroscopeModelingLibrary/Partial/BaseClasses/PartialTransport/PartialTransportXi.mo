within MetroscopeModelingLibrary.Partial.BaseClasses.PartialTransport;
partial model PartialTransportXi
  replaceable package Medium = MetroscopeModelingLibrary.Partial.Media.PartialMedium;
  import MetroscopeModelingLibrary.Units;

  // ------ Initialization parameters ------
  parameter Units.MassFraction Xi_in_0[Medium.nXi] = zeros(Medium.nXi);
  parameter Units.MassFraction Xi_out_0[Medium.nXi] = zeros(Medium.nXi);

  Units.MassFraction Xi_in[Medium.nXi](start=Xi_in_0) "Inlet species mass fraction";
  Units.MassFraction Xi_out[Medium.nXi](start=Xi_out_0) "Outlet species mass fraction";
  Units.MassFraction Xim[Medium.nXi](start=Xim_0) "Outlet species mass fraction";
  // Connectors
  Connectors.FluidConnectors.FluidInlet C_in(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-44,-12},{-24,8}})));
  Connectors.FluidConnectors.FluidOutlet C_out(redeclare package Medium = Medium) annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{26,-12},{46,8}})));
protected
  parameter Units.MassFraction Xim_0[Medium.nXi] = (Xi_in_0 + Xi_out_0)/2;
equation
  Xi_in = inStream(C_in.Xi_outflow);
  Xi_out = C_out.Xi_outflow;
  Xim = (Xi_in + Xi_out)/2;
  C_in.Xi_outflow = zeros(Medium.nXi); // No flow reversal

  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
      Ellipse(
        extent={{-80,80},{80,-80}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-55,55},{55,-55}},
        lineColor={255,255,255},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),                                   Rectangle(
          extent={{-34,28},{36,-32}},
          lineColor={28,108,200},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
      Rectangle(
        extent={{-60,14},{60,-14}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid,
        rotation=45)}),                                          Diagram(coordinateSystem(preserveAspectRatio=false), graphics={Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={28,108,200},
          lineThickness=0.5,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end PartialTransportXi;
