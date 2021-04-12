within MetroscopeModelingLibrary.Tests.UnitTests.FlueGases.PressureLosses;
model TestSingularPressureLoss
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink(h_in(start=283945),
      h_vol(start=283945))
    annotation (Placement(transformation(extent={{66,70},{86,90}})));
  MetroscopeModelingLibrary.FlueGases.PressureLosses.SingularPressureLoss
    singularPressureLoss(h_in(start=283945),h_out(start=283945))
    annotation (Placement(transformation(extent={{-2,70},{18,90}})));
  MetroscopeModelingLibrary.MoistAir.Converters.MoistAirToFlueGases
    moistAirToFlueGases
    annotation (Placement(transformation(extent={{-54,70},{-34,90}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source
    annotation (Placement(transformation(extent={{-100,70},{-80,90}})));
equation

  // Forward causality
  source.P_out = 2e5;
  source.Q_out = -100;
  source.T_vol = 24 + 273.15;
  source.relative_humidity = 0.9;

  sink.h_vol = 1e6;
  sink.Xi_vol = {0.768,0.232,0.0,0.0,0.0};

  singularPressureLoss.Kfr = 17;

  // Reverse causality
  // Determines Kfr using source and sink pressures
  /*
  source.P_out = 2e5;
  source.Q_out = -100;
  source.T_vol = 24 + 273.15;
  source.relative_humidity = 0.9;
  sink.P_in=1e5;
  sink.T_vol = 24+273.15;
  sink.Xi_vol = {0.768,0.232,0.0,0.0,0.0};
  */

  connect(singularPressureLoss.C_out, sink.C_in)
    annotation (Line(points={{18.2,80},{66,80}}, color={238,46,47}));
  connect(source.C_out,moistAirToFlueGases. C_in)
    annotation (Line(points={{-80,80},{-53.8,80}}, color={63,81,181}));
  connect(moistAirToFlueGases.C_out, singularPressureLoss.C_in)
    annotation (Line(points={{-34,80},{-2,80}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,60},
            {100,100}})), Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,60},{100,100}})));
end TestSingularPressureLoss;
