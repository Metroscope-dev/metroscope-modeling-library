within MetroscopeModelingLibrary.Tests.SimpleExamples.PowerPlant.MetroscopiaNPP;
partial model MetroscopiaNPP_topological
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.OpenSteamGenerator
    steamGenerator
    annotation (Placement(transformation(extent={{-130,-66},{-66,18}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve
    HPcontrolValve
    annotation (Placement(transformation(extent={{-52,32},{-42,42}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sinkBlowOff
    annotation (Placement(transformation(extent={{-104,-94},{-112,-86}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine
    HighPressureTurbine_1
    annotation (Placement(transformation(extent={{-30,24},{-10,44}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss
    PressureLoss_SteamExtractionHP annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={6,10})));
  MetroscopeModelingLibrary.WaterSteam.Junctions.SteamExtractionSplitter
    SteamExtraction_HP
    annotation (Placement(transformation(extent={{-4,28},{16,36}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine
    HighPressureTurbine_2
    annotation (Placement(transformation(extent={{48,24},{68,44}})));
  MetroscopeModelingLibrary.WaterSteam.Junctions.SteamExtractionSplitter SteamExtraction_Tank
    annotation (Placement(transformation(extent={{76,28},{96,36}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_DryerCondensats1
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={86,10})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_ExhaustHP
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={112,34})));
  MetroscopeModelingLibrary.WaterSteam.Junctions.SteamDryer steamDryer
    annotation (Placement(transformation(extent={{132,28},{152,36}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_DryerCondensats
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={154,12})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Superheater Superheater
    annotation (Placement(transformation(extent={{152,42},{184,58}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sinkVent
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=180,
        origin={195,41})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve SuperHeaterControlValve
    annotation (Placement(transformation(extent={{30,70},{40,80}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine LowPressureTurbine_1
    annotation (Placement(transformation(extent={{208,72},{228,92}})));
  MetroscopeModelingLibrary.WaterSteam.Junctions.SteamExtractionSplitter SteamExtraction_LP
    annotation (Placement(transformation(extent={{240,76},{260,84}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_DryerCondensats2
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={250,56})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine LowPressureTurbine_2
    annotation (Placement(transformation(extent={{280,72},{300,92}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source coldSource
    annotation (Placement(transformation(extent={{288,-8},{304,8}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink coldSink
    annotation (Placement(transformation(
        extent={{-8,-8},{8,8}},
        rotation=0,
        origin={380,-8})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_Condenser
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={344,-50})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StaticCentrifugalPump LPpump
    annotation (Placement(transformation(extent={{330,-86},{310,-66}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.CondReheater LPReheater
    annotation (Placement(transformation(extent={{232,-84},{264,-68}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve PumpControlValve
    annotation (Placement(transformation(extent={{294,-78},{284,-68}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve steamDryerValve
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=-90,
        origin={157,-33})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve LPCondReheaterControlValve
    annotation (Placement(transformation(extent={{300,-128},{310,-118}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StaticCentrifugalPump HPpump
    annotation (Placement(transformation(extent={{76,-86},{56,-66}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_before_drum
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={188,-76})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_after_drum
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={106,-76})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss
    PressureLoss_SteamExtractionHP1
                                   annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={202,16})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss
    PressureLoss_SteamExtractionHP2
                                   annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={4,-46})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater HPReheater
    annotation (Placement(transformation(extent={{16,-106},{-16,-90}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve HPCondReheaterControlValve
    annotation (Placement(transformation(extent={{92,-126},{102,-116}})));
  Electrical.Machines.Generator generator
    annotation (Placement(transformation(extent={{376,112},{424,140}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve HPCondReheater_valve
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=0,
        origin={57,-15})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.CondenserSimple condenser
    annotation (Placement(transformation(extent={{330,-16},{356,6}})));
  MetroscopeModelingLibrary.Electrical.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{436,116},{458,136}})));
equation
  // Drum equation
  PressureLoss_after_drum.h_in = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(PressureLoss_after_drum.P_in));

  connect(steamGenerator.C_steam_out, HPcontrolValve.C_in) annotation (Line(
        points={{-98,18.42},{-98,34},{-52,34},{-52,33.8182}}, color={238,46,47},
      thickness=0.5));
  connect(sinkBlowOff.C_in, steamGenerator.C_drain_out) annotation (Line(points={{-104,
          -90},{-97.36,-90},{-97.36,-65.58}},      color={28,108,200},
      thickness=0.5));
  connect(HPcontrolValve.C_out, HighPressureTurbine_1.C_in) annotation (Line(
        points={{-41.9,33.8182},{-34.95,33.8182},{-34.95,34},{-30,34}}, color={238,46,
          47},
      thickness=0.5));
  connect(HighPressureTurbine_1.C_out, SteamExtraction_HP.C_in)
    annotation (Line(points={{-9.8,34},{-3.8,34}}, color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_HP.C_ext_out, PressureLoss_SteamExtractionHP.C_in)
    annotation (Line(points={{6,28},{6,20}}, color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_HP.C_main_out, HighPressureTurbine_2.C_in)
    annotation (Line(points={{16.4,34},{48,34}}, color={238,46,47},
      thickness=0.5));
  connect(HighPressureTurbine_2.C_out, SteamExtraction_Tank.C_in)
    annotation (Line(points={{68.2,34},{76.2,34}}, color={238,46,47},
      thickness=0.5));
  connect(PressureLoss_DryerCondensats1.C_in, SteamExtraction_Tank.C_ext_out)
    annotation (Line(points={{86,20},{86,28}}, color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_Tank.C_main_out, PressureLoss_ExhaustHP.C_in)
    annotation (Line(points={{96.4,34},{102,34}},color={238,46,47},
      thickness=0.5));
  connect(PressureLoss_ExhaustHP.C_out, steamDryer.C_in) annotation (Line(
        points={{122.2,34},{129.1,34},{129.1,35.8},{132,35.8}},color={238,46,47},
      thickness=0.5));
  connect(steamDryer.C_liquid_out, PressureLoss_DryerCondensats.C_in)
    annotation (Line(points={{153,28.2},{153,22.1},{154,22.1},{154,22}}, color={28,108,
          200},
      thickness=0.5));
  connect(steamDryer.C_vapor_out, Superheater.C_cold_in)
    annotation (Line(points={{153,36},{168,36},{168,42}}, color={238,46,47},
      thickness=0.5));
  connect(Superheater.C_hot_in, SuperHeaterControlValve.C_out) annotation (Line(
        points={{152,50.2},{140,50.2},{140,72},{40.1,72},{40.1,71.8182}}, color={238,46,
          47},
      thickness=0.5));
  connect(SuperHeaterControlValve.C_in, HPcontrolValve.C_in) annotation (Line(
        points={{30,71.8182},{-6,71.8182},{-6,72},{-78,72},{-78,34},{-52,34},{
          -52,33.8182}},
                     color={238,46,47},
      thickness=0.5));
  connect(Superheater.C_vent_out, sinkVent.C_in) annotation (Line(points={{184,42},
          {188,42},{188,41},{190,41}}, color={28,108,200},
      thickness=0.5));
  connect(Superheater.C_cold_out, LowPressureTurbine_1.C_in)
    annotation (Line(points={{168,58},{168,82},{208,82}}, color={238,46,47},
      thickness=0.5));
  connect(LowPressureTurbine_1.C_out, SteamExtraction_LP.C_in)
    annotation (Line(points={{228.2,82},{240.2,82}}, color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_LP.C_ext_out, PressureLoss_DryerCondensats2.C_in)
    annotation (Line(points={{250,76},{250,66}}, color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_LP.C_main_out, LowPressureTurbine_2.C_in)
    annotation (Line(points={{260.4,82},{280,82}}, color={238,46,47},
      thickness=0.5));
  connect(LPpump.C_in, PressureLoss_Condenser.C_out) annotation (Line(
      points={{330,-76},{344,-76},{344,-60.2}},
      color={28,108,200},
      thickness=0.5));
  connect(LPReheater.C_hot_in, PressureLoss_DryerCondensats2.C_out) annotation (
     Line(
      points={{249,-68},{250,-68},{250,45.8}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_DryerCondensats.C_out, steamDryerValve.C_in) annotation (
     Line(points={{154,1.8},{154,-17.1},{153.818,-17.1},{153.818,-28}},  color={28,108,
          200},
      thickness=0.5));
  connect(PumpControlValve.C_in, LPpump.C_out) annotation (Line(
      points={{294,-76.1818},{302,-76.1818},{302,-76},{309.8,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(LPReheater.C_cold_in, PumpControlValve.C_out) annotation (Line(
      points={{264,-76},{274,-76},{274,-76.1818},{283.9,-76.1818}},
      color={28,108,200},
      thickness=0.5));
  connect(LPCondReheaterControlValve.C_in, LPReheater.C_hot_out) annotation (
      Line(
      points={{300,-126.182},{246,-126.182},{246,-84},{248,-84}},
      color={28,108,200},
      thickness=0.5));
  connect(HPpump.C_in, PressureLoss_after_drum.C_out)
    annotation (Line(points={{76,-76},{95.8,-76}}, color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_after_drum.C_in, PressureLoss_before_drum.C_out)
    annotation (Line(points={{116,-76},{177.8,-76}}, color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_before_drum.C_in, LPReheater.C_cold_out) annotation (
      Line(
      points={{198,-76},{232,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(steamDryerValve.C_out, PressureLoss_before_drum.C_out) annotation (
      Line(points={{153.818,-38.1},{153.818,-76},{177.8,-76}}, color={28,108,
          200},
      thickness=0.5));
  connect(PressureLoss_DryerCondensats1.C_out, PressureLoss_before_drum.C_out)
    annotation (Line(points={{86,-0.2},{86,-54},{154,-54},{153.818,-76},{177.8,-76}},
        color={238,46,47},
      thickness=0.5));
  connect(Superheater.C_hot_out, PressureLoss_SteamExtractionHP1.C_in)
    annotation (Line(points={{184.2,50},{202,50},{202,26}}, color={28,108,200},
      thickness=0.5));
  connect(HPReheater.C_cold_in, HPpump.C_out) annotation (Line(
      points={{16,-98},{36,-98},{36,-76},{55.8,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(HPCondReheaterControlValve.C_in, HPReheater.C_hot_out) annotation (
      Line(
      points={{92,-124.182},{46,-124.182},{46,-124},{0,-124},{0,-106}},
      color={28,108,200},
      thickness=0.5));
  connect(HPCondReheaterControlValve.C_out, PressureLoss_before_drum.C_out)
    annotation (Line(points={{102.1,-124.182},{152,-124.182},{152,-76},{177.8,
          -76}},
        color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_SteamExtractionHP2.C_out, HPReheater.C_hot_in)
    annotation (Line(
      points={{4,-56.2},{4,-90.2},{0,-90.2}},
      color={238,46,47},
      thickness=0.5));
  connect(HPReheater.C_cold_out, steamGenerator.C_water_in) annotation (Line(
      points={{-16,-98},{-64,-98},{-64,-15.6},{-83.28,-15.6}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_SteamExtractionHP1.C_out, HPCondReheater_valve.C_in)
    annotation (Line(
      points={{202,5.8},{200,5.8},{200,-18.1818},{62,-18.1818}},
      color={28,108,200},
      thickness=0.5));
  connect(HPCondReheater_valve.C_out, PressureLoss_SteamExtractionHP2.C_in)
    annotation (Line(
      points={{51.9,-18.1818},{4,-18.1818},{4,-36}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_SteamExtractionHP.C_out, PressureLoss_SteamExtractionHP2.C_in)
    annotation (Line(
      points={{6,-0.2},{4,-0.2},{4,-36},{4,-36}},
      color={238,46,47},
      thickness=0.5));
  connect(HighPressureTurbine_1.C_power, generator.C_power) annotation (Line(
        points={{-8.6,42.6},{8,42.6},{8,126},{376,126}}, color={0,0,127}));
  connect(LowPressureTurbine_2.C_power, generator.C_power) annotation (Line(
        points={{301.4,90.6},{302,126},{376,126}}, color={0,0,127}));
  connect(LowPressureTurbine_1.C_power, generator.C_power) annotation (Line(
        points={{229.4,90.6},{229.4,126},{376,126}}, color={0,0,127}));
  connect(HighPressureTurbine_2.C_power, generator.C_power) annotation (Line(
        points={{69.4,42.6},{69.4,126},{376,126}}, color={0,0,127}));
  connect(PressureLoss_Condenser.C_in, condenser.C_hot_out) annotation (Line(
      points={{344,-40},{343,-16.7333}},
      color={63,81,181},
      thickness=0.5));
  connect(condenser.C_cold_out, coldSink.C_in) annotation (Line(points={{356,-6.95556},
          {364,-6.95556},{364,-8},{372,-8}}, color={63,81,181}));
  connect(coldSource.C_out, condenser.C_cold_in) annotation (Line(points={{304,
          0},{320,0},{320,0.133333},{329.74,0.133333}}, color={63,81,181}));
  connect(LowPressureTurbine_2.C_out, condenser.C_hot_in) annotation (Line(
      points={{300.2,82},{342,82},{342,10},{343,10},{343,6.24444}},
      color={238,46,47},
      thickness=0.5));
  connect(LPCondReheaterControlValve.C_out, condenser.C_hot_in) annotation (
      Line(
      points={{310.1,-126.182},{396,-126.182},{396,14},{343,14},{343,6.24444}},
      color={63,81,181},
      thickness=0.5));

  connect(generator.C_elec, sink.u)
    annotation (Line(points={{425.44,126},{436,126}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},{460,
            140}}),
        graphics={Rectangle(
          extent={{132,-58},{170,-86}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),             Rectangle(
          extent={{132,-44},{170,-58}},
          lineColor={28,108,200},
          fillColor={170,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5)}));
end MetroscopiaNPP_topological;
