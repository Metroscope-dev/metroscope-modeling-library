within MetroscopeModelingLibrary.Partial;
partial package Connectors
  extends Modelica.Icons.Package;

  partial connector FluidPort
    import MetroscopeModelingLibrary.Units;
    replaceable package Medium = Modelica.Media.Interfaces.PartialMedium "Medium model";

    replaceable flow Units.MassFlowRate Q constrainedby Units.MassFlowRate;
    Units.Pressure P(start=1e5);
    stream Units.SpecificEnthalpy h_outflow(start=1e5);
    stream Medium.MassFraction Xi_outflow[Medium.nXi];
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end FluidPort;

  partial connector FluidInlet
    // Initialization parameters
    parameter Units.InletMassFlowRate Q_in_0=100;
    parameter Units.Pressure P_in_0=1e5;
    parameter Units.SpecificEnthalpy h_outflow_0=1e5;

    import MetroscopeModelingLibrary.Units;
    replaceable package Medium = Modelica.Media.Interfaces.PartialMedium "Medium model";

    extends MetroscopeModelingLibrary.Partial.Connectors.FluidPort(redeclare Units.InletMassFlowRate Q(start=Q_in_0, nominal=Q_in_0),
                                                                   P(start=P_in_0, nominal=P_in_0),
                                                                   h_outflow(start=h_outflow_0, nominal=h_outflow_0)); // Q out of component is negative
    annotation (Icon(coordinateSystem(preserveAspectRatio=true),
        graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={28,108,200},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio=true)));
  end FluidInlet;

  partial connector FluidOutlet
    // Initialization parameters
    parameter Units.InletMassFlowRate Q_out_0=100;
    parameter Units.Pressure P_out_0=1e5;
    parameter Units.SpecificEnthalpy h_outflow_0=0;

    import MetroscopeModelingLibrary.Units;
    replaceable package Medium = Modelica.Media.Interfaces.PartialMedium "Medium model";

    extends MetroscopeModelingLibrary.Partial.Connectors.FluidPort(redeclare Units.OutletMassFlowRate Q(start=Q_out_0, nominal=Q_out_0),
                                                                   P(start=P_out_0, nominal=P_out_0),
                                                                   h_outflow(start=h_outflow_0, nominal=h_outflow_0)); // Q out of component is negative
    annotation (Icon(coordinateSystem(extent={{-100,-100},{100,100}}),
                     graphics={
          Rectangle(
            extent={{-100,100},{100,-102}},
            lineColor={28,108,200},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(extent={{-100,-100},{100,100}})));
  end FluidOutlet;
  annotation (Icon(graphics={
      Ellipse(
        extent={{-80,80},{80,-80}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid),
      Ellipse(
        extent={{-55,55},{55,-55}},
        lineColor={255,255,255},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),
        Line(
          points={{-26,0},{22,0}},
          color={102,102,102},
          thickness=1),
        Rectangle(
          extent={{-76,26},{-26,-24}},
          lineColor={102,102,102},
          lineThickness=1,
          fillColor={102,102,102},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{22,30},{80,-28}},
          lineColor={102,102,102},
          lineThickness=1,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
      Rectangle(
        extent={{-60,14},{60,-14}},
        lineColor={215,215,215},
        fillColor={215,215,215},
        fillPattern=FillPattern.Solid,
        rotation=45)}));
end Connectors;
