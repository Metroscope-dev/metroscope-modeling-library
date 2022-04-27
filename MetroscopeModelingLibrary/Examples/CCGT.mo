within MetroscopeModelingLibrary.Examples;
package CCGT
  extends Modelica.Icons.ExamplesPackage;
  model GasTurbine_direct

    import MetroscopeModelingLibrary.Units;

    // Boundary conditions
    input Units.Pressure source_P(start=1e5) "Pa";
    input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
    input Units.SpecificEnthalpy source_h(start=0.3e6) "J/kg";

    input Units.Pressure P_fuel(start = 30e5);
    input Units.SpecificEnthalpy h_fuel(start=0.9e6);
    input Units.NegativeMassFlowRate Q_fuel(start=15);

    // Parameters
    parameter Units.SpecificEnthalpy LHV = 48130e3;
    parameter Units.DifferentialPressure combustion_chamber_pressure_loss = 0.1e5;
    parameter Real compression_rate = 17;
    parameter Real compressor_eta_is = 0.9;
    parameter Real turbine_compression_rate = 17;
    parameter Real turbine_eta_is = 0.9;
    parameter Real eta_mech = 0.99;


    FlueGases.BoundaryConditions.Source source_air annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
    FlueGases.Machines.AirCompressor                           airCompressor annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
    FlueGases.BoundaryConditions.Sink sink_exhaust annotation (Placement(transformation(extent={{66,-10},{86,10}})));
    FlueGases.Machines.GasTurbine                              gasTurbine    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
    Power.BoundaryConditions.Sink                           sink_power annotation (Placement(transformation(extent={{66,30},{86,50}})));
    MultiFluid.Machines.CombustionChamber combustionChamber annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    Fuel.BoundaryConditions.Source                           source_fuel annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,-38})));
  equation

    // Boundary Conditions
    source_air.P_out = source_P;
    source_air.h_out = source_h;
    source_air.Q_out = source_Q;
    source_air.Xi_out = {0.768,0.232,0.0,0.0,0.0};

    source_fuel.P_out = P_fuel;
    source_fuel.h_out = h_fuel;
    source_fuel.Q_out = - Q_fuel;
    source_fuel.Xi_out = {0.90,0.05,0,0,0.025,0.025};

    // Parameters
    combustionChamber.LHV = LHV;
    combustionChamber.DP = combustion_chamber_pressure_loss;
    airCompressor.tau = compression_rate;
    airCompressor.eta_is = compressor_eta_is;
    gasTurbine.tau = turbine_compression_rate;
    gasTurbine.eta_is = turbine_eta_is;
    gasTurbine.eta_mech = eta_mech;



    connect(source_air.C_out, airCompressor.C_in) annotation (Line(points={{-79,0},{-54,0}}, color={95,95,95}));
    connect(gasTurbine.C_out, sink_exhaust.C_in) annotation (Line(points={{50,0},{71,0}}, color={95,95,95}));
    connect(gasTurbine.C_W_out,sink_power. C_in) annotation (Line(
        points={{50,10},{50,10},{50,40},{71,40}},
        color={244,125,35},
        smooth=Smooth.Bezier));
    connect(airCompressor.C_W_in, gasTurbine.C_W_compressor) annotation (Line(
        points={{-34,10},{-34,26},{30,26},{30,10}},
        color={244,125,35},
        smooth=Smooth.Bezier));
    connect(combustionChamber.inlet1,source_fuel. C_out) annotation (Line(points={{0,-10},{0,-33}},                   color={213,213,0}));
    connect(combustionChamber.outlet, gasTurbine.C_in) annotation (Line(points={{10,0},{30,0}}, color={95,95,95}));
    connect(combustionChamber.inlet, airCompressor.C_out) annotation (Line(points={{-10,0},{-34,0}}, color={95,95,95}));
      annotation (
      Diagram(coordinateSystem(
          preserveAspectRatio=false,
          extent={{-100,-100},{100,100}},
          grid={2,2})),
      Window(
        x=0.03,
        y=0.02,
        width=0.95,
        height=0.95),
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

  model GasTurbine_reverse

    import MetroscopeModelingLibrary.Units;

    // Boundary conditions
    input Units.Pressure source_P(start=1e5) "Pa";
    input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
    input Units.SpecificEnthalpy source_h(start=0.3e6) "J/kg";

    input Units.Pressure P_fuel(start = 30e5);
    input Units.SpecificEnthalpy h_fuel(start=0.9e6);
    input Units.NegativeMassFlowRate Q_fuel(start=15);

    // Parameters
    parameter Units.SpecificEnthalpy LHV = 48130e3;
    parameter Units.DifferentialPressure combustion_chamber_pressure_loss = 0.1e5;
    parameter Real eta_mech = 0.99;

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
    FlueGases.Machines.GasTurbine                              gasTurbine    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
    Power.BoundaryConditions.Sink                           sink_power annotation (Placement(transformation(extent={{88,30},{108,50}})));
    MultiFluid.Machines.CombustionChamber combustionChamber annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
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
    source_fuel.Q_out = - Q_fuel;
    source_fuel.Xi_out = {0.90,0.05,0,0,0.025,0.025};

    // Parameters
    combustionChamber.LHV = LHV;
    combustionChamber.DP = combustion_chamber_pressure_loss;
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
      Window(
        x=0.03,
        y=0.02,
        width=0.95,
        height=0.95),
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

  model Evaporator_noRecirculation_direct
                // Boundary conditions
    input Real P_hot_source(start=1*1e5, min=1*1e5, nominal=1*1e5);
    input Units.MassFlowRate Q_hot_source(start=586);
    input Real hot_source_h(start=494000);

    input Real P_cold_source(start=3.5*1e5, min=1.5*1e5, nominal=3.5*1e5);
    input Units.MassFlowRate Q_cold_source(start=96);
    input Real T_cold_source(start = 132+273.15, min = 130+273.15, nominal = 150+273.15);

     // Parameters
    parameter Units.Area S = 10;
    parameter Units.HeatExchangeCoefficient Kth= 102000;
    parameter Units.FrictionCoefficient Kfr_hot = 0;
    parameter Units.FrictionCoefficient Kfr_cold = 1;


    MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-38,-36},{40,36}})));
    WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-26,38},{-46,58}})));
    WaterSteam.BoundaryConditions.Sink cold_liquid_sink annotation (Placement(transformation(extent={{-78,-62},{-98,-42}})));
    WaterSteam.BoundaryConditions.Sink cold_steam_sink annotation (Placement(transformation(extent={{-80,42},{-100,62}})));
    FlueGases.BoundaryConditions.Source                           hot_source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
    WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{72,30},{52,50}})));
    FlueGases.BoundaryConditions.Sink                           hot_sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  equation

      hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
    hot_source.P_out = P_hot_source;
    hot_source.h_out = hot_source_h;
    hot_source.Q_out = - Q_hot_source;

    cold_source.P_out = P_cold_source;
    cold_source.T_out =  T_cold_source;
    cold_source.Q_out = - Q_cold_source;

    evaporator.S_vaporising = S;
    evaporator.Kth = Kth;
    evaporator.Kfr_hot = Kfr_hot;
    evaporator.Kfr_cold = Kfr_cold;

    connect(flashTank.C_in, evaporator.C_cold_out) annotation (Line(points={{-26,52},{-10.7,52},{-10.7,25.2}}, color={28,108,200}));
    connect(flashTank.C_hot_steam, cold_steam_sink.C_in) annotation (Line(points={{-46,52},{-85,52}}, color={28,108,200}));
    connect(flashTank.C_hot_liquid, cold_liquid_sink.C_in) annotation (Line(points={{-46,44},{-76,44},{-76,-52},{-83,-52}}, color={28,108,200}));
    connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-26.3,-0.72},{-55.65,-0.72},{-55.65,0},{-85,0}}, color={95,95,95}));
    connect(evaporator.C_hot_out, hot_sink.C_in) annotation (Line(points={{28.3,-0.72},{45.65,-0.72},{45.65,0},{59,0}}, color={95,95,95}));
    connect(evaporator.C_cold_in, cold_source.C_out) annotation (Line(points={{12.7,25.2},{12.7,40},{57,40}}, color={28,108,200}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
            extent={{-40,76},{-4,40}},
            lineColor={0,0,0},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-40,54},{-30,42},{-14,42},{-8,50},{-6,56},{-6,58},{-34,60},{-38,58},{-38,54},{-40,54}},
            lineColor={28,108,200},
            lineThickness=1,
            smooth=Smooth.Bezier,
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-36,56},{-34,54}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-32,58},{-28,54}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-32,52},{-30,50}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
            Rectangle(
            extent={{-60,40},{78,-60}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Line(
            points={{40,40},{40,-12},{10,-62},{-22,-10},{-22,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{36,40},{36,-12},{6,-62},{-26,-10},{-26,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{44,40},{44,-12},{14,-62},{-18,-10},{-18,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Evaporator_noRecirculation_direct;

  model Evaporator_noRecirculation_reverse
      input Real P_hot_source(start=1*1e5, min=1*1e5, nominal=1*1e5);
    input Units.MassFlowRate Q_hot_source(start=586);
    input Real hot_source_h(start=494000);

    input Real P_cold_source(start=3.5*1e5, min=1.5*1e5, nominal=3.5*1e5);
    input Units.MassFlowRate Q_cold_source(start=96);
    input Real T_cold_source(start = 132+273.15, min = 130+273.15, nominal = 150+273.15);

    // Parameters
    parameter Units.Area S = 10;


    // Calibrated parameters
    output Units.HeatExchangeCoefficient Kth;
    output Units.FrictionCoefficient Kfr_hot;
    output Units.FrictionCoefficient Kfr_cold;

    // Calibration inputs
    input Real P_cold_out(start = 3.5, min=1.5, nominal=3.5) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
    input Real P_hot_out(start=1, min=1, nominal=1) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
    input Real Q_cold_liq_out(start = 97, min = 80, nominal = 97) "kg/s"; // Outlet temperature on cold side, to calibrate Kth
    MultiFluid.HeatExchangers.Evaporator evaporator annotation (Placement(transformation(extent={{-38,-36},{40,36}})));
    WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-26,38},{-46,58}})));
    WaterSteam.BoundaryConditions.Sink cold_liquid_sink annotation (Placement(transformation(extent={{-78,-62},{-98,-42}})));
    WaterSteam.BoundaryConditions.Sink cold_steam_sink annotation (Placement(transformation(extent={{-80,42},{-100,62}})));
    FlueGases.BoundaryConditions.Source                           hot_source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
    WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{72,30},{52,50}})));
    FlueGases.BoundaryConditions.Sink                           hot_sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
    Sensors.WaterSteam.PressureSensor                                P_cold_out_sensor annotation (Placement(transformation(extent={{-58,60},{-74,44}})));
    Sensors.WaterSteam.FlowSensor                                Q_cold_liquid_out annotation (Placement(transformation(extent={{8,-8},{-8,8}},
          rotation=90,
          origin={-50,-36})));
    Sensors.FlueGases.PressureSensor                                    P_hot_out_sensor  annotation (Placement(transformation(extent={{44,-4},{52,4}})));
  equation

      // Boundary conditions
    hot_source.Xi_out = {0.7481,0.1392,0.0525,0.0601,0.0};
    hot_source.P_out = P_hot_source;
    hot_source.h_out = hot_source_h;
    hot_source.Q_out = - Q_hot_source;

    cold_source.P_out = P_cold_source;
    cold_source.T_out =  T_cold_source;
    cold_source.Q_out = - Q_cold_source;

    // Parameters
    evaporator.S_vaporising = S;

    // Inputs for calibration
    Q_cold_liquid_out.Q = Q_cold_liq_out;
    P_cold_out_sensor.P_barA = P_cold_out;
    P_hot_out_sensor.P_barA = P_hot_out;

    // Calibrated parameters
    evaporator.Kth = Kth;
    evaporator.Kfr_hot = Kfr_hot;
    evaporator.Kfr_cold = Kfr_cold;

    connect(flashTank.C_in, evaporator.C_cold_out) annotation (Line(points={{-26,52},{-10.7,52},{-10.7,25.2}}, color={28,108,200}));
    connect(evaporator.C_hot_in, hot_source.C_out) annotation (Line(points={{-26.3,-0.72},{-55.65,-0.72},{-55.65,0},{-85,0}}, color={95,95,95}));
    connect(evaporator.C_cold_in, cold_source.C_out) annotation (Line(points={{12.7,25.2},{12.7,40},{57,40}}, color={28,108,200}));
    connect(evaporator.C_hot_out, P_hot_out_sensor.C_in) annotation (Line(points={{28.3,-0.72},{36.15,-0.72},{36.15,0},{44,0}}, color={95,95,95}));
    connect(hot_sink.C_in, P_hot_out_sensor.C_out) annotation (Line(points={{59,0},{52,0}}, color={95,95,95}));
    connect(cold_liquid_sink.C_in, Q_cold_liquid_out.C_out) annotation (Line(points={{-83,-52},{-50,-52},{-50,-44}}, color={28,108,200}));
    connect(flashTank.C_hot_liquid, Q_cold_liquid_out.C_in) annotation (Line(points={{-46,44},{-50,44},{-50,-28}}, color={28,108,200}));
    connect(flashTank.C_hot_steam, P_cold_out_sensor.C_in) annotation (Line(points={{-46,52},{-58,52}}, color={28,108,200}));
    connect(cold_steam_sink.C_in, P_cold_out_sensor.C_out) annotation (Line(points={{-85,52},{-74,52}}, color={28,108,200}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
            extent={{-40,76},{-4,40}},
            lineColor={0,0,0},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-40,54},{-30,42},{-14,42},{-8,50},{-6,56},{-6,58},{-34,60},{-38,58},{-38,54},{-40,54}},
            lineColor={28,108,200},
            lineThickness=1,
            smooth=Smooth.Bezier,
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-36,56},{-34,54}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-32,58},{-28,54}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-32,52},{-30,50}},
            lineThickness=1,
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
            Rectangle(
            extent={{-60,40},{78,-60}},
            lineColor={0,0,0},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Line(
            points={{40,40},{40,-12},{10,-62},{-22,-10},{-22,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{36,40},{36,-12},{6,-62},{-26,-10},{-26,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier),
          Line(
            points={{44,40},{44,-12},{14,-62},{-18,-10},{-18,54}},
            color={28,108,200},
            thickness=1,
            smooth=Smooth.Bezier)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Evaporator_noRecirculation_reverse;
end CCGT;
