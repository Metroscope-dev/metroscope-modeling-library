within MetroscopeModelingLibrary.Sensors;
package Displayer

  model WaterDisplayer
    package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;

    extends Partial.Sensors.Displayer(
      redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.WaterSteam.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = WaterSteamMedium) annotation (IconMap(primitivesVisible=true));

    annotation (Icon(graphics={Line(
            points={{-40,0},{42,0}},
            color={28,108,200},
            thickness=1)}));
  end WaterDisplayer;

  model FlueGasesDisplayer
    package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

    extends Partial.Sensors.Displayer(
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.FlueGases.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.FlueGases.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = FlueGasesMedium) annotation (IconMap(primitivesVisible=true));

    annotation (Icon(graphics={Line(
            points={{-40,0},{42,0}},
            color={95,95,95},
            thickness=1)}));
  end FlueGasesDisplayer;

  model FuelDisplayer
    package FuelMedium = MetroscopeModelingLibrary.Utilities.Media.FuelMedium;

    extends Partial.Sensors.Displayer(
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.Fuel.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = FuelMedium) annotation (IconMap(primitivesVisible=true));

    annotation (Icon(graphics={Line(
            points={{-40,0},{42,0}},
            color={213,213,0},
            thickness=1)}));
  end FuelDisplayer;

  model MoistAirDisplayer
    package MoistAirMedium = MetroscopeModelingLibrary.Utilities.Media.MoistAirMedium;

    extends Partial.Sensors.Displayer(
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Inlet C_in,
      redeclare MetroscopeModelingLibrary.MoistAir.Connectors.Outlet C_out,
      redeclare MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel flow_model,
      redeclare package Medium = MoistAirMedium) annotation (IconMap(primitivesVisible=true));

    annotation (Icon(graphics={Line(
            points={{-40,0},{42,0}},
            color={85,170,255},
            thickness=1)}));
  end MoistAirDisplayer;
  annotation (Icon(graphics={
        Rectangle(
          lineColor={200,200,200},
          fillColor={248,248,248},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-100},{100,100}},
          radius=25.0),
        Line(
          points={{0,72},{0,-80}},
          color={238,46,47},
          thickness=1),
        Line(
          points={{60,-20},{0,-82},{-62,-20}},
          color={238,46,47},
          thickness=1)}));
end Displayer;
