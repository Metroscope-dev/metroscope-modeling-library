within MetroscopeModelingLibrary.Media;
package FlueGasesMedium
  extends Modelica.Media.IdealGases.Common.MixtureGasNasa(
    mediumName="MediaMonomeld",
    data={Modelica.Media.IdealGases.Common.SingleGasesData.N2,
          Modelica.Media.IdealGases.Common.SingleGasesData.O2,
          Modelica.Media.IdealGases.Common.SingleGasesData.H2O,
          Modelica.Media.IdealGases.Common.SingleGasesData.CO2,
          Modelica.Media.IdealGases.Common.SingleGasesData.SO2},
    fluidConstants={Modelica.Media.IdealGases.Common.FluidData.N2,
                    Modelica.Media.IdealGases.Common.FluidData.O2,
                    Modelica.Media.IdealGases.Common.FluidData.H2O,
                    Modelica.Media.IdealGases.Common.FluidData.CO2,
                    Modelica.Media.IdealGases.Common.FluidData.SO2},
    substanceNames={"Nitrogen","Oxygen","Water","Carbondioxide","Sulfurdioxide"},
    reference_X={0.768,0.232,0.0,0.0,0.0});
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
          lineColor={102,102,102},
          fillColor={204,204,204},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Sphere,
          extent={{-60,-60},{60,60}}),
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
          lineColor={102,102,102},
          fillColor={95,95,95},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}})}));
end FlueGasesMedium;
