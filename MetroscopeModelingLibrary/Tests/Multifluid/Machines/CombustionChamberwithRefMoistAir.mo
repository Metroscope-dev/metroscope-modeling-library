within MetroscopeModelingLibrary.Tests.Multifluid.Machines;
model CombustionChamberwithRefMoistAir
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.MultifluidTestIcon;
  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=17e5) "Pa";
  input Units.Temperature source_T(start=409.7225) "deg_C";
  input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";

  input Units.Pressure P_fuel(start = 30e5);
  input Units.SpecificEnthalpy h_fuel(start=0.9e6);
  input Units.PositiveMassFlowRate Q_fuel(start=15);

  input Units.SpecificEnthalpy LHV_plant(start=47276868) "Directly assigned in combustion chamber modifiers";

  // Parameters
  parameter Units.FrictionCoefficient combustion_chamber_Kfr = 0.1;


  MultiFluid.Machines.CombustionChamberwithRefMoistAir
                                        combustion_chamber                annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source_fuel annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-38})));
  MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source
                                                                source_air annotation (Placement(transformation(extent={{-48,-10},{-28,10}})));
  MetroscopeModelingLibrary.FlueGases.BoundaryConditions.Sink sink_exhaust annotation (Placement(transformation(extent={{28,-10},{48,10}})));
equation

  // Boundary conditions for RefMoistAir
  source_air.P_out = source_P;
  source_air.Q_out = source_Q;
  source_air.T_out = source_T + 273.15;
  source_air.Xi_out[1] = 0;

  // Fuel boundary conditions
  source_fuel.P_out = P_fuel;
  source_fuel.h_out = h_fuel;
  source_fuel.Q_out = - Q_fuel;
  source_fuel.X_molar_CH4 = 0.92;
  source_fuel.X_molar_C2H6 = 0.048;
  source_fuel.X_molar_C3H8 = 0.005;
  source_fuel.X_molar_C4H10_n_butane = 0.002;
  source_fuel.X_molar_N2 = 0.015;
  source_fuel.X_molar_CO2 = 0.01;

  // Parameters
  combustion_chamber.Kfr = combustion_chamber_Kfr;
  combustion_chamber.eta = 0.999;

  connect(combustion_chamber.inlet1, source_fuel.C_out) annotation (Line(points={{0,-10},{0,-33},{2.77556e-16,-33}}, color={213,213,0}));
  connect(combustion_chamber.inlet, source_air.C_out) annotation (Line(points={{-10,0},{-33,0}}, color={95,95,95}));
  connect(combustion_chamber.outlet, sink_exhaust.C_in) annotation (Line(points={{10,0},{33,0}}, color={95,95,95}));
end CombustionChamberwithRefMoistAir;
