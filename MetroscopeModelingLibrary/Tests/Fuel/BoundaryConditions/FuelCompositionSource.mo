within MetroscopeModelingLibrary.Tests.Fuel.BoundaryConditions;
model FuelCompositionSource
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FuelTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  MetroscopeModelingLibrary.Fuel.BoundaryConditions.FuelCompositionSource
                                                           fuelCompositionSource
                                                                  annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
  Utilities.Interfaces.RealInput X_N2(start=1.5) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-62,-22}), iconTransformation(extent={{-244,-60},{-204,-20}})));
  Utilities.Interfaces.RealInput X_CO2(start=1) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-52,-36}), iconTransformation(extent={{-244,-60},{-204,-20}})));
  Utilities.Interfaces.RealInput X_C3H8(start=0.5) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-78,10}), iconTransformation(extent={{-244,-60},{-204,-20}})));
  Utilities.Interfaces.RealInput X_C4H10_n_butane(start=0.2) annotation (
      Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-78,-10}), iconTransformation(extent={{-244,-60},{-204,-20}})));
  Utilities.Interfaces.RealInput X_C2H6(start=4.8) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-62,22}), iconTransformation(extent={{-244,-60},{-204,-20}})));
  Utilities.Interfaces.RealInput X_CH4(start=92) annotation (Placement(
        transformation(
        extent={{-4,-4},{4,4}},
        rotation=0,
        origin={-56,34}), iconTransformation(extent={{-244,-60},{-204,-20}})));
equation
  fuelCompositionSource.P_out = source_P;
  fuelCompositionSource.h_out = source_h;
  fuelCompositionSource.Q_out = source_Q;

  // Molar fraction as input
  //fuelCompositionSource.X_molar_CH4 = 0.92;
  //fuelCompositionSource.X_molar_C2H6 = 0.048;
  //fuelCompositionSource.X_molar_C3H8 = 0.005;
  //fuelCompositionSource.X_molar_C4H10_n_butane = 0.002;
  //fuelCompositionSource.X_molar_N2 = 0.015;
  //fuelCompositionSource.X_molar_CO2 = 0.01;

  connect(fuelCompositionSource.C_out, sink.C_in)
    annotation (Line(points={{-23,0},{23,0}}, color={95,95,95}));
  connect(fuelCompositionSource.X_CH4, X_CH4)
    annotation (Line(points={{-32.2,8},{-32.2,34},{-56,34}}, color={0,0,127}));
  connect(X_C2H6, fuelCompositionSource.X_C2H6) annotation (Line(points={{-62,22},
          {-36,22},{-36,6},{-36.4,6}}, color={0,0,127}));
  connect(fuelCompositionSource.X_C3H8, X_C3H8) annotation (Line(points={{-39.5,
          2.1},{-40,2.1},{-40,10},{-78,10}}, color={0,0,127}));
  connect(fuelCompositionSource.X_C4H10_n_butane, X_C4H10_n_butane) annotation
    (Line(points={{-39.5,-1.9},{-39.5,-10},{-78,-10}}, color={0,0,127}));
  connect(fuelCompositionSource.X_N2, X_N2) annotation (Line(points={{-36.4,-6},
          {-36,-6},{-36,-22},{-62,-22}}, color={0,0,127}));
  connect(fuelCompositionSource.X_CO2, X_CO2)
    annotation (Line(points={{-32,-8},{-32,-36},{-52,-36}}, color={0,0,127}));
end FuelCompositionSource;
