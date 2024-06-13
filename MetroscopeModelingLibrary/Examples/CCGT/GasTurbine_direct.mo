within MetroscopeModelingLibrary.Examples.CCGT;
model GasTurbine_direct

  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
  input Units.SpecificEnthalpy source_h(start=0.3e6) "J/kg";

  input Units.Pressure P_fuel(start = 30e5);
  input Units.SpecificEnthalpy h_fuel(start=0.9e6);
  input Units.NegativeMassFlowRate Q_fuel(start=-15);

  // Parameters
  input Units.SpecificEnthalpy LHV_plant(start = 48130e3) "Directly assigned in combustion chamber modifiers";
  parameter Units.FrictionCoefficient combustion_chamber_Kfr = 0.1;
  parameter Real compression_rate = 17;
  parameter Real compressor_eta_is = 0.9;
  parameter Real turbine_compression_rate = 17;
  parameter Real turbine_eta_is = 0.9;
  parameter Real eta_mech = 0.99;
  parameter Real combustionChamber_eta = 0.9999;

  FlueGases.BoundaryConditions.Source source_air annotation (Placement(transformation(extent={{-96,-10},{-76,10}})));
  FlueGases.Machines.AirCompressor air_compressor annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
  FlueGases.BoundaryConditions.Sink sink_exhaust annotation (Placement(transformation(extent={{66,-10},{86,10}})));
  FlueGases.Machines.GasTurbine gas_turbine annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  Power.BoundaryConditions.Sink                           sink_power annotation (Placement(transformation(extent={{66,30},{86,50}})));
  MultiFluid.Machines.CombustionChamber combustion_chamber(LHV=LHV_plant) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Fuel.BoundaryConditions.Source                           source_fuel annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-38})));
  FlueGases.Machines.InletGuideVanes inletGuideVanes annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
equation

  // Boundary Conditions
  source_air.P_out = source_P;
  source_air.h_out = source_h;
  source_air.Q_out = source_Q;
  source_air.Xi_out = {0.768,0.232,0.0,0.0,0.0};

  source_fuel.P_out = P_fuel;
  source_fuel.h_out = h_fuel;
  source_fuel.Q_out = Q_fuel;
  source_fuel.Xi_out = {0.90,0.05,0,0,0.025,0.025};

  // Parameters
  combustion_chamber.Kfr = combustion_chamber_Kfr;
  combustion_chamber.eta = combustionChamber_eta;
  air_compressor.tau = compression_rate;
  air_compressor.eta_is = compressor_eta_is;
  gas_turbine.tau = turbine_compression_rate;
  gas_turbine.eta_is = turbine_eta_is;
  gas_turbine.eta_mech = eta_mech;

  connect(gas_turbine.C_out, sink_exhaust.C_in) annotation (Line(points={{50,0},{71,0}}, color={95,95,95}));
  connect(gas_turbine.C_W_shaft, sink_power.C_in) annotation (Line(
      points={{50,10},{50,10},{50,40},{71,40}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(combustion_chamber.inlet1, source_fuel.C_out) annotation (Line(points={{0,-10},{0,-33}}, color={213,213,0}));
  connect(combustion_chamber.outlet, gas_turbine.C_in) annotation (Line(points={{10,0},{30,0}}, color={95,95,95}));
  connect(combustion_chamber.inlet, air_compressor.C_out) annotation (Line(points={{-10,0},{-34,0}}, color={95,95,95}));
  connect(air_compressor.C_W_in, gas_turbine.C_W_shaft) annotation (Line(
      points={{-34,7.5},{-34,22},{50,22},{50,10}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(source_air.C_out, inletGuideVanes.C_in) annotation (Line(points={{-81,0},{-75,0}}, color={95,95,95}));
  connect(inletGuideVanes.C_out, air_compressor.C_in) annotation (Line(points={{-65,0},{-60,0},{-60,0},{-54,0}}, color={95,95,95}));
    annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2})),
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Polygon(
          points={{100,100},{100,-100},{-100,-100},{-100,100},{100,100}},
          lineColor={0,0,255},
          fillColor={244,125,35},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-32,62},{34,46},{-10,30},{46,18},{-6,0},{36,-16},{-10,-32},{42,
              -44},{-32,-60},{-32,62}},
          lineColor={0,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.CrossDiag)}));
end GasTurbine_direct;
