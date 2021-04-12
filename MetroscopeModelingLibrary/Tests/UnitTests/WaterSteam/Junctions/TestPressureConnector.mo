within MetroscopeModelingLibrary.Tests.UnitTests.WaterSteam.Junctions;
model TestPressureConnector
  import MetroscopeModelingLibrary;
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source SOURCE1
    annotation (Placement(transformation(extent={{-90,30},{-70,50}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink SINK annotation (
     Placement(visible=true, transformation(
        origin={76,0},
        extent={{-10,-10},{10,10}},
        rotation=0)));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source SOURCE2
    annotation (Placement(transformation(extent={{-92,-10},{-72,10}})));
  MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source SOURCE3
    annotation (Placement(transformation(extent={{-90,-44},{-70,-24}})));
  MetroscopeModelingLibrary.WaterSteam.Junctions.PressureConnector pressureConnector
    annotation (Placement(transformation(extent={{-4,-10},{6,10}})));
equation

  // Forward causality

   // Source 1
  SOURCE1.P_out = 8e6;
  SOURCE1.h_out = 2.65e6;
  SOURCE1.Q_out = -100;

     // Source 2
  SOURCE2.P_out = 9e6;
  SOURCE2.h_out = 2.55e6;
  SOURCE2.Q_out = -500;

       // Source 2
  SOURCE3.P_out = 10e6;
  SOURCE3.h_out = 2.55e6;
  SOURCE3.Q_out = -500;

  SINK.h_vol=1e6;

  // Reverse causality
  // The pressure connector has no parameters, so there is no reverse causality.
  // Warning : you cannot change the pressure causlity, because the model takes the min of the inlet pressures. If you don't give all of the inlet pressures, the model will not compile.


  connect(pressureConnector.C_2, SOURCE2.C_out)
    annotation (Line(points={{-4,0},{-72,0}}, color={63,81,181}));
  connect(pressureConnector.C_1, SOURCE1.C_out) annotation (Line(points={{-4,6},
          {-38,6},{-38,40},{-70,40}}, color={63,81,181}));
  connect(SOURCE3.C_out, pressureConnector.C_3) annotation (Line(points={{-70,-34},
          {-38,-34},{-38,-6},{-4,-6}}, color={63,81,181}));
  connect(pressureConnector.C_out, SINK.C_in)
    annotation (Line(points={{6,0},{66,0}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
      Documentation(info="<html>
<p>This test coresponds to data from a real Nuclear Process</p>
<p>the inlet conditions are :</p>
<p>P=9.7 bar ; h=2.5e6 J/kg ; Q=834 kg/s and the corresponding x=0.86</p>
<p>for a dryer with a maximal efficiency (x=1 at the steam outlet)</p>
<p>The expected outlet are</p>
<p>for the steam outlet : P=9.7 bar ; h=2.78e6 J/kg ; Q=721kg/s</p>
<p>for the liquid outlet : P=9.7 bar; h=7.57e5 J/kg ; Q=113</p>
</html>"));
end TestPressureConnector;
