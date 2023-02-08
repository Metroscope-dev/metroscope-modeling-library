within MetroscopeModelingLibrary.Partial.Connectors;
partial connector FluidOutlet
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  import MetroscopeModelingLibrary.Utilities.Units;
  replaceable package Medium = Modelica.Media.Interfaces.PartialMedium "Medium model";

  extends MetroscopeModelingLibrary.Partial.Connectors.FluidPort(redeclare Units.NegativeMassFlowRate Q(start=-500, nominal=-500));
                                                                                                                                  // Q out of component is negative
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end FluidOutlet;
