within MetroscopeModelingLibrary.Partial.Volumes;
partial model PartialDryer

  package WaterSteamMedium = MetroscopeModelingLibrary.Media.WaterSteamMedium;

  Real h_vap_sat;
  Real h_liq_sat;

  Real P;
  Real Q_in;

  Real x_out;

  WaterSteam.Connectors.WaterInlet C_in
    annotation (Placement(transformation(extent={{-110,30},{-90,50}}),
        iconTransformation(extent={{-110,30},{-90,50}})));
  WaterSteam.Connectors.WaterOutlet C_hot_steam
    annotation (Placement(transformation(extent={{90,30},{110,50}})));
  WaterSteam.Connectors.WaterOutlet C_hot_liquid
    annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
  WaterSteam.BaseClasses.WaterIsoPFlowModel steam_phase
    annotation (Placement(transformation(extent={{26,30},{46,50}})));
  WaterSteam.BaseClasses.WaterIsoPFlowModel liquid_phase
    annotation (Placement(transformation(extent={{26,-50},{46,-30}})));
equation


  P = C_in.P;
  Q_in = steam_phase.Q + steam_phase.Q;

  steam_phase.h_out = x_out * h_vap_sat + (1-x_out)*h_liq_sat;
  liquid_phase.h_out = h_liq_sat;

  steam_phase.W + liquid_phase.W = 0;

  h_vap_sat = WaterSteamMedium.dewEnthalpy(WaterSteamMedium.setSat_p(P));
  h_liq_sat = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(P));



  connect(liquid_phase.C_in, C_in) annotation (Line(points={{26,-40},{-40,-40},{
          -40,40},{-100,40}},
                            color={28,108,200}));
  connect(steam_phase.C_in, C_in) annotation (Line(points={{26,40},{-40,40},{-40,
          40},{-100,40}},
                        color={28,108,200}));
  connect(steam_phase.C_out,C_hot_steam)
    annotation (Line(points={{46,40},{100,40}}, color={28,108,200}));
  connect(liquid_phase.C_out,C_hot_liquid)
    annotation (Line(points={{46,-40},{100,-40}}, color={28,108,200}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end PartialDryer;
