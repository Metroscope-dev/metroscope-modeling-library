within MetroscopeModelingLibrary.Fuel.Medium;
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
         reference_X={0.92,0.048,0.005,0.002,0.015,0.01});
end FuelMedium;
