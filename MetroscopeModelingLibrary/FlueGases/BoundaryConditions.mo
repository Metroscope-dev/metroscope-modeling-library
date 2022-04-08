within MetroscopeModelingLibrary.FlueGases;
package BoundaryConditions
  model FlueGasesSource
    extends MetroscopeModelingLibrary.Icons.BoundaryConditions.FlueGasesSourceIcon;
    package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
    extends Partial.BoundaryConditions.FluidSource(redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesOutlet C_out,
                                                   redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));
  end FlueGasesSource;

  model FlueGasesSink
    extends MetroscopeModelingLibrary.Icons.BoundaryConditions.FlueGasesSinkIcon;
    package FlueGasesMedium = MetroscopeModelingLibrary.Media.FlueGasesMedium;
    extends Partial.BoundaryConditions.FluidSink(redeclare MetroscopeModelingLibrary.FlueGases.Connectors.FlueGasesInlet C_in,
                                                 redeclare package Medium = FlueGasesMedium) annotation(IconMap(primitivesVisible=false));
  end FlueGasesSink;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Ellipse(
          extent={{-76,58},{44,-62}},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          lineThickness=1,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(points={{44,0},{78,0},{64,10}}),
        Line(points={{64,-10},{78,0}})}));
end BoundaryConditions;
