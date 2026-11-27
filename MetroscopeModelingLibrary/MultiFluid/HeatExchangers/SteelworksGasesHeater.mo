within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model SteelworksGasesHeater
  "Water-to-steelworks gas heat exchanger used by the test models"
  extends MetroscopeModelingLibrary.MultiFluid.HeatExchangers.FuelHeater(
    redeclare package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.SteelworksGasesMedium,
    redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Inlet C_cold_in,
    redeclare MetroscopeModelingLibrary.SteelworksGases.Connectors.Outlet C_cold_out,
    redeclare MetroscopeModelingLibrary.SteelworksGases.Pipes.Pipe cold_side_pipe,
    redeclare MetroscopeModelingLibrary.SteelworksGases.BaseClasses.IsoPFlowModel cold_side)
    annotation (IconMap(primitivesVisible=false));
  annotation (Icon(graphics={
          Rectangle(
          extent={{-100,60},{100,-60}},
          lineColor={0,0,0},
          fillColor={231,207,255},
          fillPattern=FillPattern.Solid), Line(
          points={{40,80},{40,-80},{20,-80},{20,80},{0,80},{0,-80},{-20,-80},{
              -20,80},{-40,80},{-40,-80}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier)}));
end SteelworksGasesHeater;
