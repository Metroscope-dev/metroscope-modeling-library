within MetroscopeModelingLibrary.WaterSteam.Junctions;
model PressureConnector
  Common.Connectors.FluidInlet C_1(redeclare package Medium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium)
    annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
  Common.Connectors.FluidInlet C_2(redeclare package Medium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Common.Connectors.FluidInlet C_3(redeclare package Medium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium)
    annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
  Common.Connectors.FluidOutlet C_out(redeclare package Medium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Common.Partial.BasicTransportModel Line1(redeclare package Medium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium)
    annotation (Placement(transformation(extent={{-72,50},{-52,70}})));
  Common.Partial.BasicTransportModel Line2(redeclare package Medium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium)
    annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  Common.Partial.BasicTransportModel Line3(redeclare package Medium =
        MetroscopeModelingLibrary.WaterSteam.Medium.WaterSteamMedium)
    annotation (Placement(transformation(extent={{-68,-70},{-48,-50}})));
equation

  // Mass conservation in the lines
  Line1.Q_in + Line1.Q_out = 0;
  Line2.Q_in + Line2.Q_out = 0;
  Line3.Q_in + Line3.Q_out = 0;

  // Energy conservation in the lines
  Line1.Q_in*Line1.h_in + Line1.Q_out*Line1.h_out = 0;
  Line2.Q_in*Line2.h_in + Line2.Q_out*Line2.h_out = 0;
  Line3.Q_in*Line3.h_in + Line3.Q_out*Line3.h_out = 0;

  // Pressure determination
  Line1.P_out = noEvent(min(min(Line1.P_in, Line2.P_in),Line3.P_in));



  connect(Line1.C_in, C_1)
    annotation (Line(points={{-72,60},{-100,60}}, color={63,81,181}));
  connect(Line1.C_out, C_out) annotation (Line(points={{-51.8,60},{-26,60},{-26,
          0},{0,0}}, color={63,81,181}));
  connect(C_2, Line2.C_in)
    annotation (Line(points={{-100,0},{-70,0}}, color={63,81,181}));
  connect(Line2.C_out, C_out)
    annotation (Line(points={{-49.8,0},{0,0}}, color={63,81,181}));
  connect(C_3, Line3.C_in)
    annotation (Line(points={{-100,-60},{-68,-60}}, color={63,81,181}));
  connect(Line3.C_out, C_out) annotation (Line(points={{-47.8,-60},{-26,-60},{-26,
          0},{0,0}}, color={63,81,181}));
  connect(C_1, C_1)
    annotation (Line(points={{-100,60},{-100,60}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {0,100}}), graphics={Line(points={{-92,62},{-24,62},{-24,0},{-4,0},{
              -96,0},{-24,0},{-24,-60},{-96,-60}}, color={28,108,200})}),
                        Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{0,100}})));
end PressureConnector;
