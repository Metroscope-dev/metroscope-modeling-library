within MetroscopeModelingLibrary.Tests.Fuel.BoundaryConditions;
model Source
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.FuelTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
equation
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;

  // Mass fraction as input
  // source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

  // Molar fraction as input
  source.X_molar_CH4=0.92;
  source.X_molar_C2H6=0.048;
  source.X_molar_C3H8=0.005;
  source.X_molar_C4H10_n_butane=0.002;
  source.X_molar_N2=0.015;
  source.X_molar_CO2=0.01;



  connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={95,95,95}));
end Source;
