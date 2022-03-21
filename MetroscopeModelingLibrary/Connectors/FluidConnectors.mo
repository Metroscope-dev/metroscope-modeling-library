within MetroscopeModelingLibrary.Connectors;
package FluidConnectors
  extends Modelica.Icons.Package;
  connector FluidPort
    import MetroscopeModelingLibrary.Units;
    replaceable package Medium = Modelica.Media.Interfaces.PartialMedium "Medium model";

    flow Units.MassFlowRate Q(start=500);
    Units.Pressure P(start=1e5);
    stream Units.SpecificEnthalpy h_outflow(start=1e5);
    stream Units.MassFraction Xi_outflow[Medium.nXi];
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end FluidPort;

  connector FluidInlet
    extends MetroscopeModelingLibrary.Connectors.FluidConnectors.FluidPort(Q(min=0, start=500)); // Q into component is positive
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={28,108,200},
            lineThickness=1,
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid)}),                      Diagram(coordinateSystem(preserveAspectRatio=false)));
  end FluidInlet;

  connector FluidOutlet
    extends MetroscopeModelingLibrary.Connectors.FluidConnectors.FluidPort(Q(max=0, start=-500)); // Q out of component is negative
    annotation (Icon(coordinateSystem(extent={{80,-100},{100,-80}}),
                     graphics={
          Rectangle(
            extent={{80,-80},{100,-100}},
            lineColor={28,108,200},
            lineThickness=1,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(extent={{80,-100},{100,-80}})));
  end FluidOutlet;
  annotation (Icon(graphics={
        Rectangle(
          extent={{-74,26},{-24,-24}},
          lineColor={28,108,200},
          lineThickness=1),
        Line(
          points={{-24,0},{24,0}},
          color={28,108,200},
          thickness=1),
        Rectangle(
          extent={{24,30},{82,-28}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid)}));
end FluidConnectors;
