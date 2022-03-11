within MetroscopeModelingLibrary.Tests.SimpleExamples.PowerPlant.MetroscopiaNPP;
partial model MetroscopiaNPP_topological
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;
  package WaterSteamMedium =
      MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium;
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.OpenSteamGenerator
    steamGenerator
    annotation (Placement(transformation(extent={{-120,-108},{-66,-56}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve
    HPcontrolValve
    annotation (Placement(transformation(extent={{-62,36},{-52,26}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sinkBlowOff
    annotation (Placement(transformation(extent={{-108,-126},{-116,-118}})));
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
    annotation (Placement(transformation(extent={{28,24},{48,44}})));
  MetroscopeModelingLibrary.WaterSteam.Junctions.SteamExtractionSplitter SteamExtraction_Tank
    annotation (Placement(transformation(extent={{62,28},{82,36}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_DryerCondensats1
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={72,-6})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_ExhaustHP
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={98,34})));
  MetroscopeModelingLibrary.WaterSteam.Junctions.SteamDryer steamDryer
    annotation (Placement(transformation(extent={{132,26},{152,34}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_DryerCondensats
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={154,-6})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Superheater Superheater
    annotation (Placement(transformation(extent={{152,50},{184,66}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sinkVent
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=90,
        origin={189,33})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve SuperHeaterControlValve
    annotation (Placement(transformation(extent={{128,56},{138,66}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine LowPressureTurbine_1
    annotation (Placement(transformation(extent={{208,72},{228,92}})));
  MetroscopeModelingLibrary.WaterSteam.Junctions.SteamExtractionSplitter SteamExtraction_LP
    annotation (Placement(transformation(extent={{240,76},{260,84}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_DryerCondensats2
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={250,22})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine LowPressureTurbine_2
    annotation (Placement(transformation(extent={{280,72},{300,92}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source coldSource
    annotation (Placement(transformation(extent={{300,-8},{316,8}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink coldSink
    annotation (Placement(transformation(
        extent={{-7,-7},{7,7}},
        rotation=0,
        origin={423,-7})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_Condenser
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={380,-60})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StaticCentrifugalPump LPpump
    annotation (Placement(transformation(extent={{348,-86},{328,-66}})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.CondReheater LPReheater
    annotation (Placement(transformation(extent={{230,-84},{262,-68}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve PumpControlValve
    annotation (Placement(transformation(extent={{298,-78},{288,-68}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve steamDryerValve
    annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=-90,
        origin={157,-31})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve LPCondReheaterControlValve
    annotation (Placement(transformation(extent={{318,-124},{328,-114}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StaticCentrifugalPump HPpump
    annotation (Placement(transformation(extent={{70,-86},{50,-66}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_before_drum
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={182,-76})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.PipePressureLoss PressureLoss_after_drum
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={126,-76})));
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
        origin={6,-46})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater HPReheater
    annotation (Placement(transformation(extent={{22,-84},{-10,-68}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve HPCondReheaterControlValve
    annotation (Placement(transformation(extent={{92,-124},{102,-114}})));
  Electrical.Machines.Generator generator
    annotation (Placement(transformation(extent={{334,112},{382,140}})));
  MetroscopeModelingLibrary.WaterSteam.PressureLosses.ControlValve HPCondReheater_valve
    annotation (Placement(transformation(
        extent={{5,-5},{-5,5}},
        rotation=0,
        origin={43,-17})));
  MetroscopeModelingLibrary.WaterSteam.HeatExchangers.CondenserSimple condenser
    annotation (Placement(transformation(extent={{367,-16},{393,6}})));
  MetroscopeModelingLibrary.Electrical.BoundaryConditions.Sink sink
    annotation (Placement(transformation(extent={{436,116},{458,136}})));
  MetroscopeModelingLibrary.Electrical.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{44,-60},{54,-50}})));
  MetroscopeModelingLibrary.Electrical.BoundaryConditions.Source source1
    annotation (Placement(transformation(extent={{326,-60},{336,-50}})));
  MetroscopeModelingLibrary.Electrical.Sensors.PowerSensor ActivePower_sensor
    annotation (Placement(transformation(extent={{400,119},{414,133}})));
  MetroscopeModelingLibrary.Common.Sensors.OpeningSensor openingSensor
    annotation (Placement(transformation(
        extent={{4.5,-4.5},{-4.5,4.5}},
        rotation=90,
        origin={-57,14.5})));
  MetroscopeModelingLibrary.Common.Sensors.OpeningSensor openingSensor1
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=90,
        origin={133,78.5})));
  MetroscopeModelingLibrary.Common.Sensors.OpeningSensor openingSensor2
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=90,
        origin={43,2.5})));
  MetroscopeModelingLibrary.Common.Sensors.OpeningSensor openingSensor3
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=90,
        origin={293,-55.5})));
  MetroscopeModelingLibrary.Common.Sensors.OpeningSensor openingSensor4
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=0,
        origin={177,-31.5})));
  MetroscopeModelingLibrary.Common.Sensors.OpeningSensor openingSensor5
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=90,
        origin={97,-103.5})));
  MetroscopeModelingLibrary.Common.Sensors.OpeningSensor openingSensor6
    annotation (Placement(transformation(
        extent={{-4.5,-4.5},{4.5,4.5}},
        rotation=90,
        origin={323,-105.5})));
  MetroscopeModelingLibrary.Common.Sensors.RotSpeedSensor rotSpeedSensor
    annotation (Placement(transformation(extent={{64,-104},{76,-92}})));
  MetroscopeModelingLibrary.Common.Sensors.RotSpeedSensor rotSpeedSensor1
    annotation (Placement(transformation(extent={{346,-106},{358,-94}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    PressureSG_sensor
    annotation (Placement(transformation(extent={{-88,28},{-76,40}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    PressureCS_sensor
    annotation (Placement(transformation(extent={{318,-6},{330,6}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor
    TemperatureCS_sensor
    annotation (Placement(transformation(extent={{334,-6},{346,6}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor VolumeFlowRateCS_sensor
    "volumic sensor"
    annotation (Placement(transformation(extent={{348,-6},{360,6}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    Po_Condenser_sensor annotation (Placement(transformation(
        extent={{-5,-5},{5,5}},
        rotation=270,
        origin={380,-27})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    Po_LPSteamExtraction_sensor annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={250,56})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    Pi_LowPressureTurbine_sensor
    annotation (Placement(transformation(extent={{174,76},{186,88}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    Po_HighPressureTurbine_sensor
    annotation (Placement(transformation(extent={{114,28},{126,40}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    Po_HPSteamExtraction_sensor annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={6,-10})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    Pi_HighPressureTurbine_sensor
    annotation (Placement(transformation(extent={{-48,28},{-36,40}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor
    Po_WaterSuctionPump_sensor
    annotation (Placement(transformation(extent={{318,-82},{306,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor Po_FeedWaterTank_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={72,-30})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor Po_ValveWaterSuctionPump_sensor
    annotation (Placement(transformation(extent={{282,-82},{270,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor Pi_FeedWaterPump_sensor
    annotation (Placement(transformation(extent={{86,-82},{74,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor Po_FeedWaterPump_sensor
    annotation (Placement(transformation(extent={{50,-82},{38,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor Pi_SteamGenerator_sensor
    annotation (Placement(transformation(extent={{-58,-82},{-70,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor Pc_Superheater_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={202,-8})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor Qo_SteamDryer_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={154,16})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor Ti_LowPressureTurbine_sensor
    "same value as To_Superheater"
    annotation (Placement(transformation(extent={{190,76},{202,88}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor To_LowPressureReheater_sensor
    annotation (Placement(transformation(extent={{194,-82},{206,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor To_HighPressureReheater_sensor
    "same value as Ti_SteamGenerator"
    annotation (Placement(transformation(extent={{-30,-82},{-18,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor Tc_HighPressureReheater_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={16,-122.182})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor Qo_WaterSuctionPump_sensor
    annotation (Placement(transformation(extent={{368,-82},{356,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor Qo_FeedWaterPump_sensor
    annotation (Placement(transformation(extent={{100,-82},{88,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor Qi_SteamGenerator_sensor
    annotation (Placement(transformation(extent={{-38,-82},{-50,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor Qi_Superheater_sensor
    annotation (Placement(transformation(extent={{110,51.8182},{122,63.8182}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor Qc_HighPressureReheater_sensor
    annotation (Placement(transformation(extent={{38,-128},{50,-116}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor To_Condenser_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={380,-40})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor To_FeedWaterTank_sensor
    annotation (Placement(transformation(extent={{112,-82},{100,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor Ti_HighPressureReheater_sensor
    annotation (Placement(transformation(extent={{38,-82},{26,-70}})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterTemperatureSensor Tc_LowPressureReheater_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=0,
        origin={262,-122.182})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterFlowSensor Qc_HPSteamExtractiontoFeedWaterTank_sensor
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=270,
        origin={72,16})));
  MetroscopeModelingLibrary.WaterSteam.Sensors.WaterPressureSensor Po_LowPressureReheater_sensor
    annotation (Placement(transformation(extent={{224,-82},{212,-70}})));
equation
  // Drum equation
  PressureLoss_after_drum.h_in = WaterSteamMedium.bubbleEnthalpy(WaterSteamMedium.setSat_p(PressureLoss_after_drum.P_in));

  connect(sinkBlowOff.C_in, steamGenerator.C_drain_out) annotation (Line(points={{-108,
          -122},{-92.46,-122},{-92.46,-107.74}},   color={28,108,200},
      thickness=0.5));
  connect(HighPressureTurbine_1.C_out, SteamExtraction_HP.C_in)
    annotation (Line(points={{-9.8,34},{-3.8,34}}, color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_HP.C_ext_out, PressureLoss_SteamExtractionHP.C_in)
    annotation (Line(points={{6,28},{6,20}}, color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_HP.C_main_out, HighPressureTurbine_2.C_in)
    annotation (Line(points={{16.4,34},{28,34}}, color={238,46,47},
      thickness=0.5));
  connect(HighPressureTurbine_2.C_out, SteamExtraction_Tank.C_in)
    annotation (Line(points={{48.2,34},{62.2,34}}, color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_Tank.C_main_out, PressureLoss_ExhaustHP.C_in)
    annotation (Line(points={{82.4,34},{88,34}}, color={238,46,47},
      thickness=0.5));
  connect(steamDryer.C_vapor_out, Superheater.C_cold_in)
    annotation (Line(points={{153,34},{168,34},{168,50}}, color={238,46,47},
      thickness=0.5));
  connect(Superheater.C_hot_in, SuperHeaterControlValve.C_out) annotation (Line(
        points={{152,58.2},{140,58.2},{140,58},{138.1,58},{138.1,57.8182}},
                                                                          color={238,46,
          47},
      thickness=0.5));
  connect(Superheater.C_vent_out, sinkVent.C_in) annotation (Line(points={{184,50},
          {188,50},{188,38},{189,38}}, color={28,108,200},
      thickness=0.5));
  connect(LowPressureTurbine_1.C_out, SteamExtraction_LP.C_in)
    annotation (Line(points={{228.2,82},{240.2,82}}, color={238,46,47},
      thickness=0.5));
  connect(SteamExtraction_LP.C_main_out, LowPressureTurbine_2.C_in)
    annotation (Line(points={{260.4,82},{280,82}}, color={238,46,47},
      thickness=0.5));
  connect(LPReheater.C_hot_in, PressureLoss_DryerCondensats2.C_out) annotation (
     Line(
      points={{247,-68},{250,-68},{250,11.8}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_DryerCondensats.C_out, steamDryerValve.C_in) annotation (
     Line(points={{154,-16.2},{154,-21.1},{153.818,-21.1},{153.818,-26}},color={28,108,
          200},
      thickness=0.5));
  connect(PressureLoss_after_drum.C_in, PressureLoss_before_drum.C_out)
    annotation (Line(points={{136,-76},{171.8,-76}}, color={28,108,200},
      thickness=0.5));
  connect(Superheater.C_hot_out, PressureLoss_SteamExtractionHP1.C_in)
    annotation (Line(points={{184.2,58},{202,58},{202,26}}, color={28,108,200},
      thickness=0.5));
  connect(HPCondReheaterControlValve.C_out, PressureLoss_before_drum.C_out)
    annotation (Line(points={{102.1,-122.182},{154,-122.182},{154,-76},{171.8,
          -76}},
        color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_SteamExtractionHP2.C_out, HPReheater.C_hot_in)
    annotation (Line(
      points={{6,-56.2},{6,-68.2}},
      color={238,46,47},
      thickness=0.5));
  connect(HPCondReheater_valve.C_out, PressureLoss_SteamExtractionHP2.C_in)
    annotation (Line(
      points={{37.9,-20.1818},{6,-20.1818},{6,-36}},
      color={238,46,47},
      thickness=0.5));
  connect(HighPressureTurbine_1.C_power, generator.C_power) annotation (Line(
        points={{-8.6,42.6},{10,42.6},{10,126},{334,126}},
                                                         color={0,0,0},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(LowPressureTurbine_2.C_power, generator.C_power) annotation (Line(
        points={{301.4,90.6},{320,90.6},{320,126},{334,126}},
                                                   color={0,0,0},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(LowPressureTurbine_1.C_power, generator.C_power) annotation (Line(
        points={{229.4,90.6},{270,90.6},{270,126},{334,126}},
                                                     color={0,0,0},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(HighPressureTurbine_2.C_power, generator.C_power) annotation (Line(
        points={{49.4,42.6},{62,42.6},{62,126},{334,126}},
                                                   color={0,0,0},
      pattern=LinePattern.Dash,
      thickness=0.5));
  connect(condenser.C_cold_out, coldSink.C_in) annotation (Line(points={{393,
          -6.95556},{402,-6.95556},{402,-7},{416,-7}},
                                             color={0,140,72},
      thickness=0.5));
  connect(LowPressureTurbine_2.C_out, condenser.C_hot_in) annotation (Line(
      points={{300.2,82},{380,82},{380,6.24444}},
      color={238,46,47},
      thickness=0.5));
  connect(LPCondReheaterControlValve.C_out, condenser.C_hot_in) annotation (
      Line(
      points={{328.1,-122.182},{434,-122.182},{434,14},{380,14},{380,6.24444}},
      color={63,81,181},
      thickness=0.5));

  connect(Po_FeedWaterTank_sensor.C_out, PressureLoss_before_drum.C_out)
    annotation (Line(
      points={{72,-36.12},{72,-40},{154,-40},{154,-76},{171.8,-76}},
      color={238,46,47},
      thickness=0.5));
  connect(HPpump.C_power, source.u)
    annotation (Line(points={{60,-64.8},{60,-55},{54,-55}}, color={0,0,0},
      pattern=LinePattern.Dash));
  connect(LPpump.C_power, source1.u) annotation (Line(points={{338,-64.8},{338,
          -55},{336,-55}}, color={0,0,0},
      pattern=LinePattern.Dash));
  connect(generator.C_elec, ActivePower_sensor.C_in)
    annotation (Line(points={{383.44,126},{399.16,126}}, color={0,0,0},
      thickness=0.5,
      pattern=LinePattern.Dash));
  connect(ActivePower_sensor.C_out, sink.u)
    annotation (Line(points={{414.84,126},{436,126}}, color={0,0,0},
      thickness=0.5,
      pattern=LinePattern.Dash));
  connect(HPcontrolValve.Opening, openingSensor.Opening)
    annotation (Line(points={{-57,25.9091},{-57,19.36}}, color={0,0,127}));
  connect(SuperHeaterControlValve.Opening, openingSensor1.Opening)
    annotation (Line(points={{133,66.0909},{133,73.64}},
                                                       color={0,0,127}));
  connect(HPCondReheater_valve.Opening, openingSensor2.Opening)
    annotation (Line(points={{43,-11.9091},{43,-2.36}}, color={0,0,127}));
  connect(PumpControlValve.Opening, openingSensor3.Opening)
    annotation (Line(points={{293,-67.9091},{293,-60.36}}, color={0,0,127}));
  connect(steamDryerValve.Opening, openingSensor4.Opening) annotation (Line(
        points={{162.091,-31},{163.115,-31},{163.115,-31.5},{172.14,-31.5}},
        color={0,0,127}));
  connect(HPCondReheaterControlValve.Opening, openingSensor5.Opening)
    annotation (Line(points={{97,-113.909},{97,-108.36}}, color={0,0,127}));
  connect(LPCondReheaterControlValve.Opening, openingSensor6.Opening)
    annotation (Line(points={{323,-113.909},{323,-110.36}}, color={0,0,127}));
  connect(HPpump.VRot, rotSpeedSensor.VRot)
    annotation (Line(points={{60,-88},{60,-98},{63.52,-98}}, color={0,0,127}));
  connect(LPpump.VRot, rotSpeedSensor1.VRot) annotation (Line(points={{338,-88},
          {338,-100},{345.52,-100}},
                                   color={0,0,127}));
  connect(steamGenerator.C_steam_out, PressureSG_sensor.C_in) annotation (Line(
      points={{-93,-55.74},{-93,34},{-88,34}},
      color={238,46,47},
      thickness=0.5));
  connect(PressureSG_sensor.C_out, HPcontrolValve.C_in) annotation (Line(
      points={{-75.88,34},{-62,34},{-62,34.1818}},
      color={238,46,47},
      thickness=0.5));
  connect(coldSource.C_out, PressureCS_sensor.C_in)
    annotation (Line(points={{316,0},{318,0}}, color={0,140,72},
      thickness=0.5));
  connect(PressureCS_sensor.C_out, TemperatureCS_sensor.C_in)
    annotation (Line(points={{330.12,0},{334,0}}, color={0,140,72},
      thickness=0.5));
  connect(condenser.C_cold_in, VolumeFlowRateCS_sensor.C_out) annotation (Line(
        points={{366.74,0.133333},{363.43,0.133333},{363.43,0},{360.12,0}},
        color={0,140,72},
      thickness=0.5));
  connect(TemperatureCS_sensor.C_out, VolumeFlowRateCS_sensor.C_in)
    annotation (Line(points={{346.12,0},{348,0}}, color={0,140,72},
      thickness=0.5));
  connect(Po_Condenser_sensor.C_in, condenser.C_hot_out)
    annotation (Line(points={{380,-22},{380,-16.7333}}, color={28,108,200},
      thickness=0.5));
  connect(SteamExtraction_LP.C_ext_out, Po_LPSteamExtraction_sensor.C_in)
    annotation (Line(
      points={{250,76},{250,62}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_LPSteamExtraction_sensor.C_out, PressureLoss_DryerCondensats2.C_in)
    annotation (Line(points={{250,49.88},{250,32}}, color={28,108,200},
      thickness=0.5));
  connect(Superheater.C_cold_out, Pi_LowPressureTurbine_sensor.C_in)
    annotation (Line(
      points={{168,66},{168,82},{174,82}},
      color={238,46,47},
      thickness=0.5));
  connect(PressureLoss_ExhaustHP.C_out, Po_HighPressureTurbine_sensor.C_in)
    annotation (Line(
      points={{108.2,34},{114,34}},
      color={238,46,47},
      thickness=0.5));
  connect(Po_HighPressureTurbine_sensor.C_out, steamDryer.C_in) annotation (
      Line(
      points={{126.12,34},{129.1,34},{129.1,33.8},{132,33.8}},
      color={238,46,47},
      thickness=0.5));
  connect(PressureLoss_SteamExtractionHP.C_out, Po_HPSteamExtraction_sensor.C_in)
    annotation (Line(
      points={{6,-0.2},{6,-4}},
      color={238,46,47},
      thickness=0.5));
  connect(Po_HPSteamExtraction_sensor.C_out, PressureLoss_SteamExtractionHP2.C_in)
    annotation (Line(
      points={{6,-16.12},{6,-36}},
      color={238,46,47},
      thickness=0.5));
  connect(HPcontrolValve.C_out, Pi_HighPressureTurbine_sensor.C_in) annotation (Line(
      points={{-51.9,34.1818},{-49.95,34.1818},{-49.95,34},{-48,34}},
      color={238,46,47},
      thickness=0.5));
  connect(HighPressureTurbine_1.C_in, Pi_HighPressureTurbine_sensor.C_out) annotation (
     Line(
      points={{-30,34},{-35.88,34}},
      color={238,46,47},
      thickness=0.5));
  connect(PumpControlValve.C_in, Po_WaterSuctionPump_sensor.C_out) annotation (
      Line(
      points={{298,-76.1818},{298,-76},{305.88,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_WaterSuctionPump_sensor.C_in, LPpump.C_out) annotation (Line(
      points={{318,-76},{327.8,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_DryerCondensats1.C_out, Po_FeedWaterTank_sensor.C_in)
    annotation (Line(
      points={{72,-16.2},{72,-24}},
      color={238,46,47},
      thickness=0.5));
  connect(LPReheater.C_cold_in, Po_ValveWaterSuctionPump_sensor.C_out)
    annotation (Line(
      points={{262,-76},{269.88,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_ValveWaterSuctionPump_sensor.C_in, PumpControlValve.C_out)
    annotation (Line(
      points={{282,-76},{286,-76},{286,-76.1818},{287.9,-76.1818}},
      color={28,108,200},
      thickness=0.5));
  connect(HPpump.C_in, Pi_FeedWaterPump_sensor.C_out) annotation (Line(
      points={{70,-76},{73.88,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_FeedWaterPump_sensor.C_in, HPpump.C_out) annotation (Line(
      points={{50,-76},{49.8,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Pi_SteamGenerator_sensor.C_out, steamGenerator.C_water_in)
    annotation (Line(
      points={{-70.12,-76},{-70.12,-76.8},{-80.58,-76.8}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_SteamExtractionHP1.C_out, Pc_Superheater_sensor.C_in)
    annotation (Line(
      points={{202,5.8},{202,-2}},
      color={28,108,200},
      thickness=0.5));
  connect(Pc_Superheater_sensor.C_out, HPCondReheater_valve.C_in) annotation (
      Line(
      points={{202,-14.12},{202,-20.1818},{48,-20.1818}},
      color={28,108,200},
      thickness=0.5));
  connect(Qo_SteamDryer_sensor.C_in, steamDryer.C_liquid_out) annotation (Line(
      points={{154,22},{154,24.1},{153,24.1},{153,26.2}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_DryerCondensats.C_in, Qo_SteamDryer_sensor.C_out)
    annotation (Line(
      points={{154,4},{154,9.88}},
      color={28,108,200},
      thickness=0.5));
  connect(Pi_LowPressureTurbine_sensor.C_out, Ti_LowPressureTurbine_sensor.C_in)
    annotation (Line(
      points={{186.12,82},{190,82}},
      color={238,46,47},
      thickness=0.5));
  connect(Ti_LowPressureTurbine_sensor.C_out, LowPressureTurbine_1.C_in)
    annotation (Line(
      points={{202.12,82},{208,82}},
      color={238,46,47},
      thickness=0.5));
  connect(PressureLoss_before_drum.C_in, To_LowPressureReheater_sensor.C_in)
    annotation (Line(
      points={{192,-76},{194,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(steamDryerValve.C_out, PressureLoss_before_drum.C_out) annotation (
      Line(points={{153.818,-36.1},{153.818,-76},{171.8,-76}}, color={28,108,
          200},
      thickness=0.5));
  connect(HPReheater.C_cold_out, To_HighPressureReheater_sensor.C_out)
    annotation (Line(
      points={{-10,-76},{-17.88,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Tc_HighPressureReheater_sensor.C_in, HPReheater.C_hot_out)
    annotation (Line(
      points={{10,-122.182},{6,-122.182},{6,-84}},
      color={28,108,200},
      thickness=0.5));
  connect(LPpump.C_in, Qo_WaterSuctionPump_sensor.C_out) annotation (Line(
      points={{348,-76},{355.88,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Qo_WaterSuctionPump_sensor.C_in, PressureLoss_Condenser.C_out)
    annotation (Line(
      points={{368,-76},{380,-76},{380,-70.2}},
      color={28,108,200},
      thickness=0.5));
  connect(Pi_FeedWaterPump_sensor.C_in, Qo_FeedWaterPump_sensor.C_out)
    annotation (Line(
      points={{86,-76},{87.88,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(To_HighPressureReheater_sensor.C_in, Qi_SteamGenerator_sensor.C_in)
    annotation (Line(
      points={{-30,-76},{-38,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Qi_SteamGenerator_sensor.C_out, Pi_SteamGenerator_sensor.C_in)
    annotation (Line(
      points={{-50.12,-76},{-58,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(SuperHeaterControlValve.C_in, Qi_Superheater_sensor.C_out)
    annotation (Line(
      points={{128,57.8182},{125.06,57.8182},{125.06,57.8182},{122.12,57.8182}},
      color={238,46,47},
      thickness=0.5));
  connect(Qi_Superheater_sensor.C_in, HPcontrolValve.C_in) annotation (Line(
      points={{110,57.8182},{110,58},{-68,58},{-68,34},{-62,34},{-62,34.1818}},
      color={238,46,47},
      thickness=0.5));
  connect(HPCondReheaterControlValve.C_in, Qc_HighPressureReheater_sensor.C_out)
    annotation (Line(
      points={{92,-122.182},{50.12,-122.182},{50.12,-122}},
      color={28,108,200},
      thickness=0.5));
  connect(Qc_HighPressureReheater_sensor.C_in, Tc_HighPressureReheater_sensor.C_out)
    annotation (Line(
      points={{38,-122},{30.06,-122},{30.06,-122.182},{22.12,-122.182}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_Condenser.C_in, To_Condenser_sensor.C_out) annotation (
      Line(
      points={{380,-50},{380,-46.12}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_Condenser_sensor.C_out, To_Condenser_sensor.C_in) annotation (Line(
      points={{380,-32.1},{380,-34}},
      color={28,108,200},
      thickness=0.5));
  connect(Qo_FeedWaterPump_sensor.C_in, To_FeedWaterTank_sensor.C_out)
    annotation (Line(
      points={{100,-76},{99.88,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_after_drum.C_out, To_FeedWaterTank_sensor.C_in)
    annotation (Line(
      points={{115.8,-76},{112,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(HPReheater.C_cold_in, Ti_HighPressureReheater_sensor.C_out)
    annotation (Line(
      points={{22,-76},{25.88,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_FeedWaterPump_sensor.C_out, Ti_HighPressureReheater_sensor.C_in)
    annotation (Line(
      points={{37.88,-76},{38,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(LPCondReheaterControlValve.C_in, Tc_LowPressureReheater_sensor.C_out)
    annotation (Line(
      points={{318,-122.182},{294,-122.182},{294,-122.182},{268.12,-122.182}},
      color={28,108,200},
      thickness=0.5));
  connect(Tc_LowPressureReheater_sensor.C_in, LPReheater.C_hot_out) annotation (
     Line(
      points={{256,-122.182},{246,-122.182},{246,-84}},
      color={28,108,200},
      thickness=0.5));
  connect(PressureLoss_DryerCondensats1.C_in,
    Qc_HPSteamExtractiontoFeedWaterTank_sensor.C_out) annotation (Line(
      points={{72,4},{72,9.88}},
      color={28,108,200},
      thickness=0.5));
  connect(Qc_HPSteamExtractiontoFeedWaterTank_sensor.C_in, SteamExtraction_Tank.C_ext_out)
    annotation (Line(
      points={{72,22},{72,28}},
      color={28,108,200},
      thickness=0.5));
  connect(LPReheater.C_cold_out, Po_LowPressureReheater_sensor.C_in)
    annotation (Line(
      points={{230,-76},{224,-76}},
      color={28,108,200},
      thickness=0.5));
  connect(Po_LowPressureReheater_sensor.C_out, To_LowPressureReheater_sensor.C_out)
    annotation (Line(
      points={{211.88,-76},{206.12,-76}},
      color={28,108,200},
      thickness=0.5));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},{460,
            140}}),
        graphics={Rectangle(
          extent={{138,-64},{170,-84}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),             Rectangle(
          extent={{138,-54},{170,-64}},
          lineColor={28,108,200},
          fillColor={170,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5)}));
end MetroscopiaNPP_topological;
