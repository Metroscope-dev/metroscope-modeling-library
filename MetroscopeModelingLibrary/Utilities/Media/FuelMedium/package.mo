within MetroscopeModelingLibrary.Utilities.Media;
package FuelMedium
        extends Modelica.Media.IdealGases.Common.MixtureGasNasa(
         mediumName="SimpleNaturalGas",
         data={Modelica.Media.IdealGases.Common.SingleGasesData.CH4,
        Modelica.Media.IdealGases.Common.SingleGasesData.C2H6,
        Modelica.Media.IdealGases.Common.SingleGasesData.C3H8,
        Modelica.Media.IdealGases.Common.SingleGasesData.C4H10_n_butane,
        Modelica.Media.IdealGases.Common.SingleGasesData.N2,
        Modelica.Media.IdealGases.Common.SingleGasesData.CO2},
         fluidConstants={Modelica.Media.IdealGases.Common.FluidData.CH4,
           Modelica.Media.IdealGases.Common.FluidData.C2H6,
           Modelica.Media.IdealGases.Common.FluidData.C3H8,
           Modelica.Media.IdealGases.Common.FluidData.C4H10_n_butane,
           Modelica.Media.IdealGases.Common.FluidData.N2,
           Modelica.Media.IdealGases.Common.FluidData.CO2},
         substanceNames = {"Methane","Ethane","Propane","N-Butane,","Nitrogen","Carbondioxide"},
         reference_X={0.92,0.048,0.005,0.002,0.015,0.01}) annotation(IconMap(primitivesVisible=false));

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
          fillColor={213,213,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.Solid,
          extent={{-60,-60},{60,60}})}));
end FuelMedium;
