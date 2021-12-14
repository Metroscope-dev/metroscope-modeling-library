within MetroscopeModelingLibrary.Tests.SimpleExamples.WaterSteam;
model RealHPST_withExtractions
  extends Modelica.Icons.Example;
  import MetroscopeModelingLibrary;
  MetroscopeModelingLibrary.WaterSteam.Junctions.SteamExtractionSplitter
    Extraction6
    annotation (Placement(transformation(extent={{-26,14},{-18,20}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine ST1
    annotation (Placement(transformation(extent={{-54,10},{-34,30}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine ST2
    annotation (Placement(transformation(extent={{-10,10},{10,30}})));
  MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine ST3
    annotation (Placement(transformation(extent={{32,10},{52,30}})));
  MetroscopeModelingLibrary.WaterSteam.Junctions.SteamExtractionSplitter
    Extraction5 annotation (Placement(transformation(extent={{16,14},{24,20}})));
  MetroscopeModelingLibrary.WaterSteam.Junctions.SteamExtractionSplitter
    Extraction4 annotation (Placement(transformation(extent={{60,14},{68,20}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-94,10},{-74,30}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_Ext6
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-22,-8})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_Ext5
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={20,-8})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_Ext4
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={64,-8})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink_Main
    annotation (Placement(transformation(extent={{78,10},{98,30}})));
equation
  ST1.Cst=14820;
  ST1.eta_is=0.847;
  ST1.eta_nz=1;
  ST1.area_nz=1;
  ST2.Cst=2941;
  ST2.eta_is=0.847;
  ST2.eta_nz=1;
  ST2.area_nz=1;
  ST3.Cst=1362;
  ST3.eta_is=0.847;
  ST3.eta_nz=1;
  ST3.area_nz=1;
  Extraction6.alpha=0.973;
  Extraction5.alpha=0.964;
  Extraction4.alpha=1;
  source.P_out=6.05e6;
  source.h_out=2.78e6;
  sink_Ext6.h_vol = 1e5;
  sink_Ext6.Q_in=112;
  sink_Ext5.h_vol = 1e5;
  sink_Ext5.Q_in=101;
  sink_Ext4.h_vol = 1e5;
  sink_Ext4.Q_in=130;
  sink_Main.P_in=9.69e5;
  sink_Main.h_vol = 1e5;
  connect(ST1.C_out, Extraction6.C_in) annotation (Line(points={{-33.8,20},{-30,
          20},{-30,18.5},{-25.92,18.5}},color={238,46,47}));
  connect(Extraction6.C_main_out, ST2.C_in) annotation (Line(points={{-17.84,
          18.5},{-13.76,18.5},{-13.76,20},{-10,20}},
                                                color={238,46,47}));
  connect(ST2.C_out, Extraction5.C_in) annotation (Line(points={{10.2,20},{12,
          20},{12,18.5},{16.08,18.5}},
                                  color={238,46,47}));
  connect(Extraction5.C_main_out, ST3.C_in) annotation (Line(points={{24.16,
          18.5},{30.24,18.5},{30.24,20},{32,20}},
                                             color={238,46,47}));
  connect(ST3.C_out, Extraction4.C_in) annotation (Line(points={{52.2,20},{56,
          20},{56,18.5},{60.08,18.5}},
                                  color={238,46,47}));
  connect(Extraction4.C_main_out, sink_Main.C_in) annotation (Line(points={{68.16,
          18.5},{73.24,18.5},{73.24,20},{78,20}},   color={238,46,47}));
  connect(Extraction4.C_ext_out, sink_Ext4.C_in)
    annotation (Line(points={{64,14},{64,2}}, color={238,46,47}));
  connect(Extraction5.C_ext_out, sink_Ext5.C_in)
    annotation (Line(points={{20,14},{20,2}}, color={238,46,47}));
  connect(Extraction6.C_ext_out, sink_Ext6.C_in)
    annotation (Line(points={{-22,14},{-22,2}},         color={238,46,47}));
  connect(source.C_out, ST1.C_in)
    annotation (Line(points={{-74,20},{-54,20}}, color={238,46,47}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)),                                    Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-20},{100,40}})));
end RealHPST_withExtractions;
