within MetroscopeModelingLibrary.Examples.CCGT;
model GasTurbine_reverse

  import MetroscopeModelingLibrary.Utilities.Units;

  // Boundary conditions
  input Units.Pressure source_P(start=1e5) "Pa";
  input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
  input Units.SpecificEnthalpy source_h(start=0.3e6) "J/kg";

  input Units.Pressure P_fuel(start = 30e5);
  input Units.SpecificEnthalpy h_fuel(start=0.9e6);
  input Units.NegativeMassFlowRate Q_fuel(start=-15);

  // Parameters
  input Units.SpecificEnthalpy LHV_plant( start=48130e3) "Directly assigned in combustion chamber modifiers";
  parameter Units.FrictionCoefficient combustion_chamber_Kfr = 0.1;
  parameter Real eta_mech = 0.99;
  parameter Real combustionChamber_eta = 0.9999;

  // Inputs for calibration
  input Real compressor_P_out(start = 16) "barA";
  input Real compressor_T_out(start = 406) "degC";
  input Real W(start = 200) "MW";
  input Real turbine_P_out(start=1) "barA";

  // Parameters for calibration
  output Real compression_rate;
  output Real compressor_eta_is;
  output Real turbine_compression_rate;
  output Real turbine_eta_is;

  // Initialisation parameters
  parameter Units.SpecificEnthalpy h_out_compressor_0 = 7e5; // Model won't initialize correctly without a guess value for the outlet enthalpy

  FlueGases.BoundaryConditions.Source source_air annotation (Placement(transformation(extent={{-108,-10},{-88,10}})));
  FlueGases.Machines.AirCompressor                           airCompressor(h_out(start=h_out_compressor_0)) annotation (Placement(transformation(extent={{-84,-10},{-64,10}})));
  FlueGases.BoundaryConditions.Sink sink_exhaust annotation (Placement(transformation(extent={{88,-10},{108,10}})));
  FlueGases.Machines.GasTurbine                              gasTurbine(eta_is(
        start=0.73), eta_mech(start=0.9))                                  annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  Power.BoundaryConditions.Sink                           sink_power annotation (Placement(transformation(extent={{88,30},{108,50}})));
  MultiFluid.Machines.CombustionChamber combustionChamber(LHV=LHV_plant) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Fuel.BoundaryConditions.Source                           source_fuel annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-38})));
  Sensors.FlueGases.PressureSensor compressor_P_out_sensor annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
  Sensors.FlueGases.TemperatureSensor compressor_T_out_sensor annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
  Sensors.FlueGases.PressureSensor turbine_P_out_sensor annotation (Placement(transformation(extent={{62,-10},{82,10}})));
  Sensors.Power.PowerSensor W_sensor annotation (Placement(transformation(extent={{64,30},{84,50}})));
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
  combustionChamber.Kfr = combustionChamber_Kfr;
  combustionChamber.eta = combustionChamber_eta;
  gasTurbine.eta_mech = eta_mech;

  // Inputs for calibration
  compressor_T_out_sensor.T_degC = compressor_T_out;
  compressor_P_out_sensor.P_barA = compressor_P_out;
  W_sensor.W_MW = W;
  turbine_P_out_sensor.P_barA = turbine_P_out;

  // Parameters for calibration
  airCompressor.tau = compression_rate;
  airCompressor.eta_is = compressor_eta_is;
  gasTurbine.tau = turbine_compression_rate;
  gasTurbine.eta_is = turbine_eta_is;

  connect(source_air.C_out, airCompressor.C_in) annotation (Line(points={{-93,0},{-84,0}}, color={95,95,95}));
  connect(airCompressor.C_W_in, gasTurbine.C_W_compressor) annotation (Line(
      points={{-64,10},{-64,22},{30,22},{30,10}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(combustionChamber.inlet1,source_fuel. C_out) annotation (Line(points={{0,-10},{0,-33}},                   color={213,213,0}));
  connect(combustionChamber.outlet, gasTurbine.C_in) annotation (Line(points={{10,0},{30,0}}, color={95,95,95}));
  connect(sink_power.C_in, W_sensor.C_out) annotation (Line(points={{93,40},{83.8,40}}, color={244,125,35}));
  connect(W_sensor.C_in, gasTurbine.C_W_out) annotation (Line(
      points={{64,40},{50,40},{50,10}},
      color={244,125,35},
      smooth=Smooth.Bezier));
  connect(turbine_P_out_sensor.C_out, sink_exhaust.C_in) annotation (Line(points={{82,0},{93,0}}, color={95,95,95}));
  connect(turbine_P_out_sensor.C_in, gasTurbine.C_out) annotation (Line(points={{62,0},{50,0}}, color={95,95,95}));
  connect(airCompressor.C_out, compressor_P_out_sensor.C_in) annotation (Line(points={{-64,0},{-58,0}}, color={95,95,95}));
  connect(compressor_P_out_sensor.C_out, compressor_T_out_sensor.C_in) annotation (Line(points={{-38,0},{-34,0}}, color={95,95,95}));
  connect(compressor_T_out_sensor.C_out, combustionChamber.inlet) annotation (Line(points={{-14,0},{-10,0}}, color={95,95,95}));
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
end GasTurbine_reverse;
