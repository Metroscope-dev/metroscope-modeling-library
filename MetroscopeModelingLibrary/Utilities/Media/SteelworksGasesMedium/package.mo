within MetroscopeModelingLibrary.Utilities.Media;
package SteelworksGasesMedium
  extends Modelica.Media.IdealGases.Common.MixtureGasNasa(
    mediumName="SteelworksGas",
    data={Modelica.Media.IdealGases.Common.SingleGasesData.N2,
          Modelica.Media.IdealGases.Common.SingleGasesData.CH4,
          Modelica.Media.IdealGases.Common.SingleGasesData.CO2,
          Modelica.Media.IdealGases.Common.SingleGasesData.H2,
          Modelica.Media.IdealGases.Common.SingleGasesData.CO,
          Modelica.Media.IdealGases.Common.SingleGasesData.O2,
          Modelica.Media.IdealGases.Common.SingleGasesData.H2O},
    fluidConstants={Modelica.Media.IdealGases.Common.FluidData.N2,
                    Modelica.Media.IdealGases.Common.FluidData.CH4,
                    Modelica.Media.IdealGases.Common.FluidData.CO2,
                    Modelica.Media.IdealGases.Common.FluidData.H2,
                    Modelica.Media.IdealGases.Common.FluidData.CO,
                    Modelica.Media.IdealGases.Common.FluidData.O2,
                    Modelica.Media.IdealGases.Common.FluidData.H2O},
    substanceNames={"Nitrogen","Methane","Carbondioxide",
                    "Hydrogen","Carbonmonoxide","Oxygen","Water"},
    reference_X=moleToMassFractions(referenceMoleFractions, MMX))
    annotation(IconMap(primitivesVisible=false));

  constant Real referenceMolePercent[nX] =
    {48.51989,0.015602,22.06561,4.447048,24.22417,0.687878,0}
    "Reference chromatograph readings in mole percent, in medium species order";

  constant Modelica.Units.SI.MoleFraction referenceMoleFractions[nX] =
    referenceMolePercent/sum(referenceMolePercent)
    "Normalized reference mole fractions";

  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Ellipse(
          lineColor={102,102,102},
          fillColor={213,170,255},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}}),
        Rectangle(
          lineColor={128,128,128},
          extent={{-100,-100},{100,100}},
          radius=25.0)}));
end SteelworksGasesMedium;
