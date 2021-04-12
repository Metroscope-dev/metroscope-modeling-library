within MetroscopeModelingLibrary.Tests.UnitTests.MoistAir.Converters;
model TestMoistAirToFlueGases
  import MetroscopeModelingLibrary;
  package FlueGasesMedium =
      MetroscopeModelingLibrary.FlueGases.Medium.FlueGasesMedium;
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink FlueGasesSink
    annotation (Placement(transformation(extent={{76,-30},{96,-10}})));
  MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source AirSource
    annotation (Placement(transformation(extent={{-94,-30},{-74,-10}})));
  MetroscopeModelingLibrary.MoistAir.Converters.MoistAirToFlueGases moistAirToFlueGases
    annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
  MetroscopeModelingLibrary.MoistAir.PressureLosses.SingularPressureLoss
    MoistAirSingularPressureLoss
    annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
  MetroscopeModelingLibrary.FlueGases.PressureLosses.SingularPressureLoss
    FlueGasesSingularPressureLoss
    annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
equation

  // Forward causality
  // The converter has no paramaters, therefore there is no reverse causality

  AirSource.P_out = 5e5;
  AirSource.T_vol = 24+273.15;
  AirSource.relative_humidity = 0.01;
  AirSource.Q_out = -1;

  FlueGasesSink.h_vol = 3e5;
  FlueGasesSink.Xi_vol = {0.768,0.232,0.0,0.0,0.0};

  MoistAirSingularPressureLoss.Kfr = 1e-6;
  FlueGasesSingularPressureLoss.Kfr = 1e-6;

  connect(MoistAirSingularPressureLoss.C_out, moistAirToFlueGases.C_in)
    annotation (Line(points={{-39.8,-20},{-9.8,-20}}, color={63,81,181}));
  connect(AirSource.C_out, MoistAirSingularPressureLoss.C_in)
    annotation (Line(points={{-74,-20},{-60,-20}}, color={63,81,181}));
  connect(FlueGasesSingularPressureLoss.C_out, FlueGasesSink.C_in)
    annotation (Line(points={{60.2,-20},{76,-20}}, color={63,81,181}));
  connect(moistAirToFlueGases.C_out, FlueGasesSingularPressureLoss.C_in)
    annotation (Line(points={{10,-20},{40,-20}}, color={63,81,181}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},
            {100,0}})),                                          Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-40},{100,0}})));
end TestMoistAirToFlueGases;
