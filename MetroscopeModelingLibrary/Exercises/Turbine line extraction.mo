within ;
model UnnamedA
  MetroscopeModelingLibrary.Power.Machines.Generator generator
    annotation (Placement(transformation(extent={{38,30},{64,46}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-70,-6},{-60,4}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine steamTurbine2
    annotation (Placement(transformation(extent={{14,-10},{28,8}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation
    (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=-90,
        origin={-2,-30})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink1
    annotation (Placement(transformation(extent={{56,-7},{68,5}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.SteamTurbine steamTurbine1
    annotation (Placement(transformation(extent={{-42,-10},{-28,8}})));
  MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter
    steamExtractionSplitter
    annotation (Placement(transformation(extent={{-10,-9},{6,7}})));
  MetroscopeModelingLibrary.Power.BoundaryConditions.Sink sink2
    annotation (Placement(transformation(extent={{70,29},{88,47}})));
equation
  connect(source.C_out, steamTurbine1.C_in)
    annotation (Line(points={{-62.5,-1},{-42,-1}}, color={28,108,200}));
  connect(generator.C_in, steamTurbine1.C_W_out) annotation (Line(points={{
          42.94,38},{-22,38},{-22,6.56},{-28,6.56}}, color={244,125,35}));
  connect(generator.C_in, steamTurbine2.C_W_out) annotation (Line(points={{
          42.94,38},{34,38},{34,6.56},{28,6.56}}, color={244,125,35}));
  connect(steamTurbine2.C_out, sink1.C_in)
    annotation (Line(points={{28,-1},{59,-1}}, color={28,108,200}));
  connect(steamTurbine1.C_out, steamExtractionSplitter.C_in)
    annotation (Line(points={{-28,-1},{-10.48,-1}}, color={28,108,200}));
  connect(steamExtractionSplitter.C_main_out, steamTurbine2.C_in)
    annotation (Line(points={{6.48,-1},{14,-1}}, color={28,108,200}));
  connect(steamExtractionSplitter.C_ext_out, sink.C_in)
    annotation (Line(points={{-2,-6.44},{-2,-27}}, color={28,108,200}));
  connect(generator.C_out, sink2.C_in)
    annotation (Line(points={{60.1,38},{74.5,38}}, color={244,125,35}));
  annotation (uses(MetroscopeModelingLibrary(version="4.0.0-beta")));
end UnnamedA;
