within MetroscopeModelingLibrary.MultiFluid.HeatExchangers;
model OnceThroughEvaporator "0D subcritical once-through evaporator"
  extends MetroscopeModelingLibrary.Utilities.Icons.KeepingScaleIcon;
  import MetroscopeModelingLibrary.Utilities.Units;
  package WaterSteamMedium = MetroscopeModelingLibrary.Utilities.Media.WaterSteamMedium;
  package FlueGasesMedium = MetroscopeModelingLibrary.Utilities.Media.FlueGasesMedium;

  parameter Units.Area S = 15000 "Heat transfer area";
  parameter Boolean faulty = false "Enable thermal fouling";
  Units.Percentage fouling "Reduction in heat transfer coefficient in percent";

  Units.Power W_heating "Heat absorbed during preheating";
  Units.Power W_vap "Heat absorbed during evaporation";
  Units.SpecificEnthalpy h_vap_sat;
  Units.SpecificEnthalpy h_liq_sat;
  Units.Temperature Tsat;
  Units.MassFlowRate Q_cold;
  Units.MassFlowRate Q_hot;
  Units.Temperature T_cold_in;
  Units.Temperature T_hot_in;
  Units.Temperature T_cold_out;
  Units.Temperature T_hot_out;
  Units.MassFraction x_steam_out "Vapor mass fraction at the water-steam outlet";
  Units.DifferentialTemperature T_approach
    "Water inlet temperature difference to saturation";
  Units.DifferentialTemperature DT_hot_in_side;
  Units.DifferentialTemperature DT_hot_out_side;
  Units.DifferentialTemperature pinch;

  Power.HeatExchange.NTUHeatExchange HX_vaporising(config="evaporator")
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-20,16})));
  FlueGases.BaseClasses.IsoPFlowModel hot_side_vaporising
    annotation (Placement(transformation(extent={{10,-10},{-10,10}}, rotation=180, origin={-20,0})));
  FlueGases.BaseClasses.IsoPFlowModel hot_side_heating
    annotation (Placement(transformation(extent={{10,-10},{-10,10}}, rotation=180, origin={20,0})));
  WaterSteam.Pipes.FrictionPipe cold_side_pipe
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=270, origin={58,88})));
  WaterSteam.BaseClasses.IsoPFlowModel cold_side_heating
    annotation (Placement(transformation(extent={{10,10},{-10,-10}}, origin={20,30})));
  WaterSteam.BaseClasses.IsoPFlowModel cold_side_vaporising
    annotation (Placement(transformation(extent={{10,10},{-10,-10}}, origin={-20,30})));

  FlueGases.Connectors.Inlet C_hot_in
    annotation (Placement(transformation(extent={{-48,-10},{-28,10}}),
      iconTransformation(extent={{-48,-10},{-28,10}})));
  FlueGases.Connectors.Outlet C_hot_out
    annotation (Placement(transformation(extent={{30,-10},{50,10}}),
      iconTransformation(extent={{30,-10},{50,10}})));
  WaterSteam.Connectors.Inlet C_cold_in
    annotation (Placement(transformation(extent={{48,152},{68,172}}),
      iconTransformation(extent={{-10,-108},{10,-88}})));
  WaterSteam.Connectors.Outlet C_cold_out
    annotation (Placement(transformation(extent={{-48,270},{-28,290}}),
      iconTransformation(extent={{-10,90},{10,110}})));
  Utilities.Interfaces.GenericReal Kfr_cold "Cold-side friction coefficient"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-44,80}),
      iconTransformation(extent={{-4,-4},{4,4}},
        rotation=180,
        origin={-44,80})));
  Utilities.Interfaces.GenericReal Kth(unit="W/(m2.K)")
    annotation (Placement(transformation(extent={{-40,-84},{-48,-76}}),
      iconTransformation(extent={{-40,-84},{-48,-76}})));

equation
  // Failure modes
  if not faulty then
    fouling = 0;
  end if;

  // Definitions
  Q_cold = cold_side_heating.Q;
  Q_hot = hot_side_vaporising.Q;
  T_cold_in = cold_side_pipe.T_in;
  T_cold_out = cold_side_vaporising.T_out;
  T_hot_in = hot_side_vaporising.T_in;
  T_hot_out = hot_side_heating.T_out;
  Tsat = cold_side_heating.T_out;
  h_vap_sat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(cold_side_heating.P_in));
  h_liq_sat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(cold_side_heating.P_in));

  // Preheat the water to saturated liquid.
  T_approach = Tsat - T_cold_in;
  hot_side_heating.W = -W_heating;
  cold_side_heating.W = W_heating;
  cold_side_heating.h_out = max(cold_side_heating.h_in, h_liq_sat);

  // The evaporation duty determines the enthalpy of the full outlet flow.
  hot_side_vaporising.W = -W_vap;
  cold_side_vaporising.W = W_vap;
  x_steam_out = WaterSteamMedium.vapourQuality(cold_side_vaporising.state_out);
  HX_vaporising.W = W_vap;
  HX_vaporising.Kth = Kth*(1 - fouling/100);
  HX_vaporising.S = S;
  HX_vaporising.Q_cold = Q_cold;
  HX_vaporising.Q_hot = Q_hot;
  HX_vaporising.T_cold_in = Tsat;
  HX_vaporising.T_hot_in = hot_side_vaporising.T_in;
  HX_vaporising.Cp_cold = 0;
  HX_vaporising.Cp_hot = FlueGasesMedium.specificHeatCapacityCp(hot_side_vaporising.state_in);

  DT_hot_in_side = T_hot_in - T_cold_out;
  DT_hot_out_side = T_hot_out - T_cold_in;
  pinch = min(DT_hot_in_side, DT_hot_out_side);
  assert(pinch > 0, "A negative pinch is reached", AssertionLevel.warning);
  assert(pinch > 1 or pinch < 0, "A very low pinch (<1) is reached", AssertionLevel.warning);

  connect(C_hot_in, hot_side_vaporising.C_in)
    annotation (Line(points={{-42,0},{-30,0}}, color={95,95,95}));
  connect(hot_side_vaporising.C_out, hot_side_heating.C_in)
    annotation (Line(points={{-10,0},{10,0}}, color={95,95,95}));
  connect(hot_side_heating.C_out, C_hot_out)
    annotation (Line(points={{30,0},{40,0}}, color={95,95,95}));
  connect(C_cold_in, cold_side_pipe.C_in)
    annotation (Line(points={{58,162},{58,98}}, color={28,108,200}));
  connect(cold_side_pipe.C_out, cold_side_heating.C_in)
    annotation (Line(points={{58,78},{58,30},{30,30}}, color={28,108,200}));
  connect(Kfr_cold, cold_side_pipe.Kfr)
    annotation (Line(points={{-44,80},{10,80},{10,88},{62,88}},
                                               color={0,0,127}));
  connect(cold_side_heating.C_out, cold_side_vaporising.C_in)
    annotation (Line(points={{10,30},{-10,30}}, color={28,108,200}));
  connect(cold_side_vaporising.C_out, C_cold_out)
    annotation (Line(points={{-30,30},{-38,30},{-38,280}}, color={28,108,200}));

  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,
            -100},{40,100}}),
    graphics={
      Text(extent={{-80,-105},{80,-150}}, textString="%name"),
        Rectangle(
          extent={{-100,10},{100,-10}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          origin={-30,0},
          rotation=90),
        Rectangle(
          extent={{-100,5},{100,-5}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          origin={-15,0},
          rotation=90),
        Rectangle(
          extent={{-100,10},{100,-10}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          rotation=90),
        Rectangle(
          extent={{-100,5},{100,-5}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid,
          origin={15,0},
          rotation=90),
        Rectangle(
          extent={{-100,10},{100,-10}},
          lineColor={0,0,0},
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid,
          origin={30,0},
          rotation=90),
        Rectangle(
          extent={{-10,-72},{10,-78}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-10,-44},{10,-56}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-10,-4},{10,-26}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-10,36},{10,8}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-10,72},{10,40}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-10,104},{10,74}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid)}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-40,-100},{40,
            100}},
      initialScale=0.5)),
    Documentation(info="<html>
<p>Stationary 0D evaporator for subcritical boiling with the same preheating, NTU evaporation and isobaric flow models as Evaporator. A lumped isenthalpic FrictionPipe at the water inlet applies the cold-side pressure drop using Kfr_cold and the inlet water density. Preheating and evaporation take place at the resulting outlet pressure. The single water-steam outlet carries the full mass flow. Its enthalpy comes from the heat balance and x_steam_out reports the corresponding vapor mass fraction.</p>
<p>Use this model for an outlet in the boiling range. A downstream superheater or separator can be added as a separate system component where required.</p>
</html>"));
end OnceThroughEvaporator;
