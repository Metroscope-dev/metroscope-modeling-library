within MetroscopeModelingLibrary.Tests.Fuel.BoundaryConditions;
model Source_new
  extends MetroscopeModelingLibrary.Icons.Tests.FuelTestIcon;
  import MetroscopeModelingLibrary.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
  input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
  input Real LHV = 47e6;
  parameter String LHV_source = "calculated";

  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source_new source(LHV_source=LHV_source) annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
equation
  source.P_out = source_P;
  source.h_out = source_h;
  source.Q_out = source_Q;
  source.X_molar_fuel_CH4=0.9581;
  source.X_molar_fuel_C2H6=0.0267;
  source.X_molar_fuel_C3H8=0.0019;
  source.X_molar_fuel_C4H10_n_butane=0.0006;
  source.X_molar_fuel_N2=0.0089;
  source.X_molar_fuel_CO2=0.0038;
  source.LHV_given=LHV;


  connect(sink.C_in, source.C_out) annotation (Line(points={{23,0},{-65,0}}, color={213,213,0}));
end Source_new;
