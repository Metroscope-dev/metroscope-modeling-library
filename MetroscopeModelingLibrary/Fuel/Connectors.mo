within MetroscopeModelingLibrary.Fuel;
package Connectors

  connector FuelInlet
    extends MetroscopeModelingLibrary.Icons.Connectors.FuelInletIcon;

    package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
    extends Partial.Connectors.FluidInlet(redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
  end FuelInlet;

  connector FuelOutlet
    extends MetroscopeModelingLibrary.Icons.Connectors.FuelOutletIcon;

    package FuelMedium = MetroscopeModelingLibrary.Media.FuelMedium;
    extends Partial.Connectors.FluidOutlet(redeclare package Medium = FuelMedium) annotation(IconMap(primitivesVisible=false));
  end FuelOutlet;
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
        Rectangle(
          extent={{-74,26},{-24,-24}},
          lineColor={213,213,0},
          lineThickness=1,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-24,0},{24,0}},
          color={213,213,0},
          thickness=1),
        Rectangle(
          extent={{24,30},{82,-28}},
          lineColor={213,213,0},
          lineThickness=1,
          fillColor={213,213,0},
          fillPattern=FillPattern.Solid)}));
end Connectors;
