within MetroscopeModelingLibrary.Tests;
package WaterSteam
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestPackageIcon;

  package BoundaryConditions
    extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestPackageIcon;

    model Source
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;
      import MetroscopeModelingLibrary.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
    equation
      source.P_out = source_P;
      source.h_out = source_h;
      source.Q_out = source_Q;

      connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={28,108,200}));
    end Source;

    model Sink
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;
      import MetroscopeModelingLibrary.Units;

      // Boundary conditinos
      input Units.Pressure sink_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy sink_h(start=1e6) "J/kg";
      input Units.PositiveMassFlowRate sink_Q(start=100) "kg/s";

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
    equation
      sink.P_in = sink_P;
      sink.h_in = sink_h;
      sink.Q_in = sink_Q;
      connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={28,108,200}));
    end Sink;
  end BoundaryConditions;

  package BaseClasses
    extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestPackageIcon;
    model FlowModel
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      import MetroscopeModelingLibrary.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=50e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=2e6) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

      // Parameters
      input Units.DifferentialPressure DP(start=1e5);
      input Units.Power W(start=1e5);

      .MetroscopeModelingLibrary.WaterSteam.BaseClasses.FlowModel flowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;

      // Parameters
      flowModel.DP = DP;
      flowModel.W = W;

      connect(flowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={28,108,200}));
      connect(flowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={28,108,200}));
    end FlowModel;

    model IsoPFlowModel
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      import MetroscopeModelingLibrary.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=50e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=2e6) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

      // Parameters
      input Units.Power W(start=1e5);

      .MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPFlowModel isoPFlowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;

      // Parameters
      isoPFlowModel.W = W;

      connect(isoPFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={28,108,200}));
      connect(isoPFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={28,108,200}));
    end IsoPFlowModel;

    model IsoHFlowModel
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      import MetroscopeModelingLibrary.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=50e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=2e6) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

      // Parameters
      input Units.DifferentialPressure DP(start=1e5);

      .MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoHFlowModel isoHFlowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;

      // Parameters
      isoHFlowModel.DP = DP;

      connect(isoHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={28,108,200}));
      connect(isoHFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={28,108,200}));
    end IsoHFlowModel;

    model IsoPHFlowModel
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      import MetroscopeModelingLibrary.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=50e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=2e6) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

      .MetroscopeModelingLibrary.WaterSteam.BaseClasses.IsoPHFlowModel isoPHFlowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;

      connect(isoPHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={28,108,200}));
      connect(isoPHFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={28,108,200}));
    end IsoPHFlowModel;
  end BaseClasses;

  package Pipes
    extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestPackageIcon;

    model SteamExtractionSplitter_direct
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.SpecificEnthalpy h_source(start=2.65e6);
      input Units.Pressure source_P(start=2.64e5) "Pa";
      input Units.MassFlowRate source_Q(start=2000) "kg/s";
      input Units.MassFlowRate extracted_Q(start=100) "kg/s";

      // Input: Component parameters
      input Units.Fraction alpha(start=0.8);

      // Components
      .MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter steamExtractionSplitter annotation (Placement(transformation(extent={{-27,-26.6667},{27,21.3333}})));

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink main_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={90,-5.55112e-16})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink extraction_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={0,-70})));

    equation
      // Boundary conditions
      source.h_out = h_source;
      source.P_out = source_P;
      source.Q_out = source_Q;
      extraction_sink.Q_in = extracted_Q;

      // Input: Component parameters
      steamExtractionSplitter.alpha = alpha;
      connect(main_sink.C_in, steamExtractionSplitter.C_main_out) annotation (Line(points={{85,0},{56,0},{56,-3.33333e-05},{28.62,-3.33333e-05}}, color={28,108,200}));
      connect(source.C_out, steamExtractionSplitter.C_in) annotation (Line(points={{-85,0},{-56.81,0},{-56.81,-3.33333e-05},{-28.62,-3.33333e-05}}, color={28,108,200}));
      connect(steamExtractionSplitter.C_ext_out, extraction_sink.C_in) annotation (Line(points={{0,-18.1334},{0,-41.5667},{8.88178e-16,-41.5667},{8.88178e-16,-65}}, color={28,108,200}));
    end SteamExtractionSplitter_direct;

    model SteamExtractionSplitter_reverse
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.SpecificEnthalpy h_source(start=2.65e6);
      input Units.Pressure source_P(start=2.64e5) "Pa";
      input Units.MassFlowRate source_Q(start=2000) "kg/s";
      input Units.MassFlowRate extracted_Q(start=100) "kg/s";

      // Input: Observables
      input Units.SpecificEnthalpy main_h_out(start=2.67e6);

      // Output: Component parameters
      output Units.Fraction alpha;

      // Components
      .MetroscopeModelingLibrary.WaterSteam.Pipes.SteamExtractionSplitter steamExtractionSplitter annotation (Placement(transformation(extent={{-27,-26.6667},{27,21.3333}})));

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink main_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={90,-5.55112e-16})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink extraction_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={0,-70})));

    equation
      // Boundary conditions
      source.h_out = h_source;
      source.P_out = source_P;
      source.Q_out = source_Q;
      extraction_sink.Q_in = extracted_Q;

      // Input: Observables
      steamExtractionSplitter.mainFlow.h_out = main_h_out;

      // Output: Component parameters
      steamExtractionSplitter.alpha = alpha;

      connect(main_sink.C_in, steamExtractionSplitter.C_main_out) annotation (Line(points={{85,0},{56,0},{56,-3.33333e-05},{28.62,-3.33333e-05}}, color={28,108,200}));
      connect(steamExtractionSplitter.C_ext_out, extraction_sink.C_in) annotation (Line(points={{0,-18.1334},{0,-41.5667},{8.88178e-16,-41.5667},{8.88178e-16,-65}}, color={28,108,200}));
      connect(steamExtractionSplitter.C_in, source.C_out) annotation (Line(points={{-28.62,-3.33333e-05},{-56.81,-3.33333e-05},{-56.81,0},{-85,0}}, color={28,108,200}));
    end SteamExtractionSplitter_reverse;

    model Pipe_direct
      import MetroscopeModelingLibrary.Units;
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.SpecificEnthalpy source_h(start=1e6);
      input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

      // Parameters
      parameter Units.FrictionCoefficient Kfr=1 "m-4";
      parameter Units.DifferentialHeight delta_z=1 "m";

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={90,-6.10623e-16})));

      .MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-16.5,-16.3333},{16.5,16.3333}})));

    equation

      // Boundary conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;

      // Parameters
      pipe.Kfr = Kfr;
      pipe.delta_z = delta_z;

      connect(sink.C_in, pipe.C_out) annotation (Line(points={{85,0},{16.5,0}}, color={28,108,200}));
      connect(source.C_out, pipe.C_in) annotation (Line(points={{-85,0},{-16.5,0}}, color={28,108,200}));
    end Pipe_direct;

    model Pipe_reverse
      import MetroscopeModelingLibrary.Units;
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.SpecificEnthalpy source_h(start=1e6);
      input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

      // Parameters
      parameter Units.DifferentialHeight delta_z = 1 "m";

      // Input for calibration
      input Units.DifferentialPressure DP(start=0.5e5) "Pa";

      // Calibrated Parameters
      output Units.FrictionCoefficient Kfr;

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={90,-6.10623e-16})));

      .MetroscopeModelingLibrary.WaterSteam.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-16.5,-16.3333},{16.5,16.3333}})));

      MetroscopeModelingLibrary.Sensors.WaterSteam.DeltaPressureSensor DP_sensor annotation (Placement(transformation(extent={{-10,30},{10,50}})));
    equation

      // Boundary conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;

      // Parameters
      pipe.delta_z = delta_z;

      // Inputs for calibration
      DP_sensor.DP = DP;

      // Calibrated parameter
      pipe.Kfr = Kfr;
      connect(sink.C_in, pipe.C_out) annotation (Line(points={{85,0},{16.5,0}}, color={28,108,200}));
      connect(pipe.C_in, source.C_out) annotation (Line(points={{-16.5,0},{-85,0}}, color={28,108,200}));
      connect(DP_sensor.C_out, pipe.C_out) annotation (Line(points={{10,40},{36,40},{36,0},{16.5,0}}, color={28,108,200}));
      connect(DP_sensor.C_in, source.C_out) annotation (Line(points={{-10,40},{-36,40},{-36,0},{-85,0}}, color={28,108,200}));
    end Pipe_reverse;

    model ControlValve_direct
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      /* In direct mode, control valves are usually used as a sort of pressure cut in the system. The pressure
  will be known on both sides (for instance on a NPP, it will come from the condenser through the turbine line and
  reach the high pressure steam valve, and on the other side the pressure will come from the steam generator.
  As the characteristics of the valve is known and the mass flow rate is known, its opening will adapt to match the pressure
  difference. */

      // Boundary conditions
      input Units.SpecificEnthalpy source_h(start=1e6);
      input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Pressure sink_P(start=9e5) "Pa";

      // Parameters
      parameter Units.Cv Cvmax=3e4 "Cvmax";

      // Components
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-68,-9.99996},{-48,9.99996}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={58,-6.10623e-16})));

      .MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve control_valve annotation (Placement(transformation(extent={{-16.5,-5.93938},{16.5,26.7272}})));

    equation

      // Boundary conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      sink.P_in = sink_P;

      // Parameters
      control_valve.Cvmax = Cvmax;

      connect(control_valve.C_out, sink.C_in) annotation (Line(points={{16.5,-1.81818e-06},{34.75,-1.81818e-06},{34.75,0},{53,0}}, color={28,108,200}));
      connect(control_valve.C_in, source.C_out) annotation (Line(points={{-16.5,-1.81818e-06},{-34.75,-1.81818e-06},{-34.75,0},{-53,0}},color={28,108,200}));
    end ControlValve_direct;

    model ControlValve_reverse
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.SpecificEnthalpy source_h(start=1e6);
      input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Pressure sink_P(start=9e5) "Pa";

      // Inputs for calibration
      input Real opening(start=0.35);

      // Calibrated parameter
      output Units.Cv Cvmax "Cvmax";

      // Components
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-68,-9.99996},{-48,9.99996}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={58,-6.10623e-16})));

      .MetroscopeModelingLibrary.WaterSteam.Pipes.ControlValve control_valve annotation (Placement(transformation(extent={{-16.5,-5.93938},{16.5,26.7272}})));

      MetroscopeModelingLibrary.Sensors.Outline.OpeningSensor opening_sensor annotation (Placement(transformation(extent={{-10,50},{10,70}})));
    equation
      // Boundary conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      sink.P_in = sink_P;

      // Inputs for calibration
      opening_sensor.Opening = opening;

      // Calibrated Parameters
      control_valve.Cvmax = Cvmax;

      connect(control_valve.C_out, sink.C_in) annotation (Line(points={{16.5,-1.81818e-06},{34.75,-1.81818e-06},{34.75,0},{53,0}}, color={28,108,200}));
      connect(control_valve.C_in, source.C_out) annotation (Line(points={{-16.5,-1.81818e-06},{-34.75,-1.81818e-06},{-34.75,0},{-53,0}},color={28,108,200}));
      connect(control_valve.Opening, opening_sensor.Opening) annotation (Line(points={{0,23.7575},{0,49.8}}, color={0,0,127}));
    end ControlValve_reverse;

    model HeatLoss
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
      input Units.Pressure source_P(start=2e5, min=0, nominal=2) "Pa";
      input Units.NegativeMassFlowRate source_Q(start=-500) "kg/s";
      input Units.Power W(start=1e5) "W";

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={90,-6.10623e-16})));

      .MetroscopeModelingLibrary.WaterSteam.Pipes.HeatLoss heat_loss annotation (Placement(transformation(extent={{-16.5,-16.3333},{16.5,16.3333}})));

    equation

      // Boundary conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      heat_loss.W = W;

      connect(sink.C_in, heat_loss.C_out) annotation (Line(points={{85,0},{16.5,0}}, color={28,108,200}));
      connect(source.C_out, heat_loss.C_in) annotation (Line(points={{-85,0},{-16.5,0}}, color={28,108,200}));
    end HeatLoss;

    model PressureCut
      import MetroscopeModelingLibrary.Units;
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.SpecificEnthalpy source_h(start=1e6);
      input Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
      input Units.Pressure sink_P(start=1e5, min=0, nominal=10e5) "Pa";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={90,-6.10623e-16})));

      .MetroscopeModelingLibrary.WaterSteam.Pipes.PressureCut pressureCut annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    equation

      // Boundary conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      sink.P_in = sink_P;

      connect(source.C_out, pressureCut.C_in) annotation (Line(points={{-85,0},{-10,0}}, color={28,108,200}));
      connect(pressureCut.C_out, sink.C_in) annotation (Line(points={{10,0},{85,0}}, color={28,108,200}));
    end PressureCut;

    model LoopBreaker

      import MetroscopeModelingLibrary.Units;
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.SpecificEnthalpy h(start=1e6) "J/kg";
      input Units.Pressure P(start=10e5) "Pa";
      input Units.PositiveMassFlowRate Q(start=100) "kg/s";

      .MetroscopeModelingLibrary.WaterSteam.Pipes.LoopBreaker loopBreaker annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BaseClasses.FlowModel flowModel annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={0,20})));
    equation

      // Boundary conditions
      flowModel.Q = Q;
      flowModel.h_in = h;
      flowModel.P_in= P;

      connect(flowModel.C_in, loopBreaker.C_out) annotation (Line(points={{10,20},{20,20},{20,0},{10,0}}, color={28,108,200}));
      connect(loopBreaker.C_in, flowModel.C_out) annotation (Line(points={{-10,0},{-20,0},{-20,20},{-10,20}}, color={28,108,200}));
    end LoopBreaker;
  end Pipes;

  package Machines
    extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestPackageIcon;

    model Pump_direct
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.Pressure source_P(start=2e5);
      input Units.Temperature source_T(start=20 + 273.15);
      input Units.NegativeMassFlowRate source_Q(start=-100);
      input Real pump_VRot(start=1400);

      // Component parameters
      parameter Real pump_VRotn = 1400;
      parameter Real pump_rm = 0.85;
      parameter Real pump_a1 = -88.67;
      parameter Real pump_a2 = 0;
      parameter Real pump_a3 = 43.15;
      parameter Real pump_b1 = -3.7751;
      parameter Real pump_b2 = 3.61;
      parameter Real pump_b3 = -0.0075464;
      parameter Units.Yield pump_rhmin = 0.20;

      .MetroscopeModelingLibrary.WaterSteam.Machines.Pump pump annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{66,-10},{86,10}})));
      MetroscopeModelingLibrary.Sensors.Outline.VRotSensor pump_VRot_sensor annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={0,-48})));
      MetroscopeModelingLibrary.Power.BoundaryConditions.Source powerSource annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,50})));
    equation
      // Boundary conditions
      source.P_out = source_P;
      source.T_out = source_T;
      source.Q_out = source_Q;
      pump_VRot_sensor.VRot = pump_VRot;

      // Component parameters
      pump.VRotn = pump_VRotn;
      pump.rm = pump_rm;
      pump.a1 = pump_a1;
      pump.a2 = pump_a2;
      pump.a3 = pump_a3;
      pump.b1 = pump_b1;
      pump.b2 = pump_b2;
      pump.b3 = pump_b3;
      pump.rhmin = pump_rhmin;

      connect(pump.C_in, source.C_out) annotation (Line(points={{-10,0},{-61,0}}, color={28,108,200}));
      connect(pump.C_out, sink.C_in) annotation (Line(points={{10,0},{71,0}}, color={28,108,200}));
      connect(pump.VRot, pump_VRot_sensor.VRot) annotation (Line(points={{0,-12},{0,-37.8},{1.77636e-15,-37.8}}, color={0,0,127}));
      connect(pump.C_power, powerSource.C_out) annotation (Line(points={{0,10.8},{0,28},{-8.88178e-16,28},{-8.88178e-16,45.2}}, color={244,125,35}));
    end Pump_direct;

    model Pump_reverse
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.Pressure source_P(start=2e5);
      input Units.Temperature source_T(start=20 + 273.15);
      input Units.NegativeMassFlowRate source_Q(start=-100);

      // Component parameters
      parameter Real pump_VRot = 1400;
      parameter Real pump_VRotn = 1400;
      parameter Real pump_rm = 0.85;
      parameter Real pump_a1 = -88.67;
      parameter Real pump_a2 = 0;
      parameter Real pump_b1 = -3.7751;
      parameter Real pump_b2 = 3.61;
      parameter Units.Yield pump_rhmin = 0.20;

      // Calibrated parameters
      output Real pump_a3;
      output Real pump_b3;

      // Calibration inputs
      input Units.Pressure pump_P_out(start=6e5);
      input Units.Temperature pump_T_out(start=20 + 273.15);

      .MetroscopeModelingLibrary.WaterSteam.Machines.Pump pump annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-30,0})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-70,0})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={80,0})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor pump_T_out_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor pump_P_out_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={40,0})));
      MetroscopeModelingLibrary.Sensors.Outline.VRotSensor pump_VRot_sensor annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={-30,-46})));
      MetroscopeModelingLibrary.Power.BoundaryConditions.Source powerSource annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-30,42})));
    equation
      // Boundary conditions
      source.P_out = source_P;
      source.T_out = source_T;
      source.Q_out = source_Q;
      pump_VRot_sensor.VRot = pump_VRot; // Could be replaced by power fed to component

      // Component parameters
      pump.VRotn = pump_VRotn;
      pump.rm = pump_rm;
      pump.a1 = pump_a1;
      pump.a2 = pump_a2;
      pump.b1 = pump_b1;
      pump.b2 = pump_b2;
      pump.rhmin = pump_rhmin;

      // Calibrated parameters
      pump.a3 = pump_a3;
      pump.b3 = pump_b3;

      // Inputs for calibration
      pump_T_out_sensor.T = pump_T_out;
      pump_P_out_sensor.P = pump_P_out;

      connect(pump.C_in, source.C_out) annotation (Line(points={{-40,0},{-65,0}}, color={28,108,200}));
      connect(pump.C_out, pump_T_out_sensor.C_in) annotation (Line(points={{-20,0},{-10,0}}, color={28,108,200}));
      connect(pump_T_out_sensor.C_out, pump_P_out_sensor.C_in) annotation (Line(points={{10,0},{30,0}}, color={28,108,200}));
      connect(pump_P_out_sensor.C_out, sink.C_in) annotation (Line(points={{50,0},{75,0}}, color={28,108,200}));
      connect(pump.VRot, pump_VRot_sensor.VRot) annotation (Line(points={{-30,-12},{-30,-35.8}}, color={0,0,127}));
      connect(pump.C_power, powerSource.C_out) annotation (Line(points={{-30,10.8},{-30,37.2}}, color={244,125,35}));
    end Pump_reverse;

    model StodolaTurbine_direct
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.Pressure source_P(start=20e5);
      input Units.SpecificEnthalpy source_h(start=2.7718e6);
      input Units.NegativeMassFlowRate source_Q(start=-100);

      .MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine stodolaTurbine annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-66,-10},{-46,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{62,-10},{82,10}})));
      MetroscopeModelingLibrary.Power.BoundaryConditions.Sink power_sink annotation (Placement(transformation(extent={{62,20},{82,40}})));
    equation

      // Boundary conditions
      source.P_out = source_P;
      source.h_out = source_h;
      source.Q_out = source_Q;

      // Component parameters
      stodolaTurbine.Cst = 1500;
      stodolaTurbine.eta_is = 0.8;
      stodolaTurbine.area_nz = 1;
      stodolaTurbine.eta_nz = 1;

      connect(sink.C_in, stodolaTurbine.C_out) annotation (Line(points={{67,0},{10,0}}, color={28,108,200}));
      connect(stodolaTurbine.C_in, source.C_out) annotation (Line(points={{-10,0},{-51,0}}, color={28,108,200}));
      connect(stodolaTurbine.C_W_out, power_sink.C_in) annotation (Line(points={{10,8.4},{56,8.4},{56,30},{67,30}}, color={244,125,35}));
    end StodolaTurbine_direct;

    model StodolaTurbine_reverse
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.Pressure source_P(start=20e5);
      input Units.SpecificEnthalpy source_h(start=2.7718e6);
      input Units.NegativeMassFlowRate source_Q(start=-100);

      // Inputs for calibration
      input Real stodolaTurbine_P_out(start=15, unit="bar", nominal=15, min=0) "bar";
      input Real stodolaTurbine_W_out(start=2.6, unit="MW", nominal=100, min=0) "MW";

      // Calibrated parameters
      output Units.Cst stodolaTurbine_Cst;
      output Units.Yield stodolaTurbine_eta_is;

      .MetroscopeModelingLibrary.WaterSteam.Machines.StodolaTurbine stodolaTurbine annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-66,-10},{-46,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{62,-10},{82,10}})));
      MetroscopeModelingLibrary.Power.BoundaryConditions.Sink power_sink annotation (Placement(transformation(extent={{62,20},{82,40}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor stodolaTurbine_P_out_sensor annotation (Placement(transformation(extent={{32,-10},{52,10}})));
      MetroscopeModelingLibrary.Sensors.Power.PowerSensor stodolaTurbine_W_out_sensor annotation (Placement(transformation(extent={{32,20},{52,40}})));
    equation
      // Boundary conditions
      source.P_out = source_P;
      source.h_out = source_h;
      source.Q_out = source_Q;

      // Component parameters
      stodolaTurbine.area_nz = 1;
      stodolaTurbine.eta_nz = 1;

      // Calibrated parameters
      stodolaTurbine.Cst = stodolaTurbine_Cst;
      stodolaTurbine.eta_is = stodolaTurbine_eta_is;

      // Inputs for calibration
      stodolaTurbine_P_out_sensor.P_barA = stodolaTurbine_P_out;
      stodolaTurbine_W_out_sensor.W_MW = stodolaTurbine_W_out;
      connect(stodolaTurbine.C_in, source.C_out) annotation (Line(points={{-10,0},{-51,0}}, color={28,108,200}));
      connect(stodolaTurbine_P_out_sensor.C_in, stodolaTurbine.C_out) annotation (Line(points={{32,0},{10,0}}, color={28,108,200}));
      connect(sink.C_in, stodolaTurbine_P_out_sensor.C_out) annotation (Line(points={{67,0},{52,0}}, color={28,108,200}));
      connect(stodolaTurbine.C_W_out, stodolaTurbine_W_out_sensor.C_in) annotation (Line(points={{10,8.4},{18,8.4},{18,30},{32,30}}, color={244,125,35}));
      connect(stodolaTurbine_W_out_sensor.C_out, power_sink.C_in) annotation (Line(points={{51.8,30},{67,30}}, color={244,125,35}));
    end StodolaTurbine_reverse;
  end Machines;

  package HeatExchangers
    extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestPackageIcon;

    model LiqLiqHX_direct

      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

        // Boundary conditions
      input Real P_hot_source(start=50, min=0, nominal=10) "barA";
      input Units.MassFlowRate Q_hot_source(start=50) "kg/s";
      input Real T_hot_source(start = 100, min = 0, nominal = 50) "degC";

      input Real P_cold_source(start=20, min=0, nominal=10) "barA";
      input Units.MassFlowRate Q_cold_source(start=100) "kg/s";
      input Real T_cold_source(start = 50, min = 0, nominal = 50) "degC";

        // Parameters
      parameter Units.Area S = 100;
      parameter Units.HeatExchangeCoefficient Kth = 500;
      parameter Units.FrictionCoefficient Kfr_hot = 0;
      parameter Units.FrictionCoefficient Kfr_cold = 20;

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.LiqLiqHX liqLiqHX annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,30})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-30})));
    equation

      hot_source.P_out = P_hot_source * 1e5;
      hot_source.T_out = 273.15 + T_hot_source;
      hot_source.Q_out = - Q_hot_source;

      cold_source.P_out = P_cold_source *1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source;

      liqLiqHX.S = S;
      liqLiqHX.Kth = Kth;
      liqLiqHX.Kfr_hot = Kfr_hot;
      liqLiqHX.Kfr_cold = Kfr_cold;

      connect(liqLiqHX.C_cold_out, cold_sink.C_in) annotation (Line(
          points={{16,0},{45,0}},
          color={28,108,200}));
      connect(hot_sink.C_in, liqLiqHX.C_hot_out) annotation (Line(points={{8.88178e-16,
              -25},{8.88178e-16,-20.5},{0,-20.5},{0,-8}}, color={28,108,200}));
      connect(hot_source.C_out, liqLiqHX.C_hot_in) annotation (Line(points={{-8.88178e-16,
              25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
      connect(cold_source.C_out, liqLiqHX.C_cold_in)
        annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,80}})),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}})));
    end LiqLiqHX_direct;

    model LiqLiqHX_reverse

      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

        // Boundary conditions
      input Real P_hot_source(start=50, min=0, nominal=10) "barA";
      input Units.MassFlowRate Q_hot_source(start=50) "kg/s";
      input Real T_hot_source(start = 100, min = 0, nominal = 50) "degC";

      input Real P_cold_source(start=20, min=0, nominal=10) "barA";
      input Units.MassFlowRate Q_cold_source(start=100) "kg/s";
      input Real T_cold_source(start = 50, min = 0, nominal = 50) "degC";

        // Parameters
      parameter Units.Area S = 100;

        // Calibrated parameters
      output Units.HeatExchangeCoefficient Kth;
      output Units.FrictionCoefficient Kfr_hot;
      output Units.FrictionCoefficient Kfr_cold;

        // Calibration inputs
      input Real P_cold_out(start = 19, min= 0, nominal = 10) "barA"; // Outlet pressure on cold side, to calibrate Kfr cold
      input Real P_hot_out(start = 50, min = 0, nominal = 10) "barA"; // Outlet pressure on hot side, to calibrate Kfr hot
      input Real T_cold_out(start = 55, min = 0, nominal = 100) "degC"; // Outlet temperature on cold side, to calibrate Kth

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{52,-10},{72,10}})));
      .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.LiqLiqHX liqLiqHX annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,30})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-36})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_cold_out_sensor annotation (Placement(transformation(extent={{22,-6},{34,6}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cold_out_sensor annotation (Placement(transformation(extent={{38,-6},{50,6}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_hot_out_sensor annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=270,
            origin={0,-18})));
    equation

      // Boundary conditions
      hot_source.P_out = P_hot_source * 1e5;
      hot_source.T_out = 273.15 + T_hot_source;
      hot_source.Q_out = - Q_hot_source;
      cold_source.P_out = P_cold_source *1e5;
      cold_source.T_out = 273.15 + T_cold_source;
      cold_source.Q_out = - Q_cold_source;

      // Parameters
      liqLiqHX.S = S;

      // Inputs for calibration
      T_cold_out_sensor.T_degC = T_cold_out;
      P_cold_out_sensor.P_barA = P_cold_out;
      P_hot_out_sensor.P_barA = P_hot_out;

      // Calibrated parameters
      liqLiqHX.Kth = Kth;
      liqLiqHX.Kfr_hot = Kfr_hot;
      liqLiqHX.Kfr_cold = Kfr_cold;

      connect(hot_source.C_out, liqLiqHX.C_hot_in) annotation (Line(points={{-8.88178e-16,
              25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
      connect(cold_source.C_out, liqLiqHX.C_cold_in)
        annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
      connect(P_cold_out_sensor.C_out, cold_sink.C_in)
        annotation (Line(points={{50,0},{57,0}}, color={28,108,200}));
      connect(P_cold_out_sensor.C_in, T_cold_out_sensor.C_out)
        annotation (Line(points={{38,0},{34,0}}, color={28,108,200}));
      connect(T_cold_out_sensor.C_in, liqLiqHX.C_cold_out)
        annotation (Line(points={{22,0},{16,0}}, color={28,108,200}));
      connect(hot_sink.C_in, P_hot_out_sensor.C_out) annotation (Line(points={{8.88178e-16,
              -31},{8.88178e-16,-27.5},{-1.11022e-15,-27.5},{-1.11022e-15,-24}},
            color={28,108,200}));
      connect(liqLiqHX.C_hot_out, P_hot_out_sensor.C_in) annotation (Line(points={{0,
              -8},{0,-10},{1.11022e-15,-10},{1.11022e-15,-12}}, color={28,108,200}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -80},{100,100}})),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}})));
    end LiqLiqHX_reverse;

    model LiqLiqHX_faulty
      extends LiqLiqHX_direct(
          liqLiqHX(faulty = true));

      Real Failure_fouling(start=0);

    equation

      // Failure input
      Failure_fouling = 0 + 10*time;

      // Failure definition
      liqLiqHX.fouling = Failure_fouling;

    end LiqLiqHX_faulty;

    model SteamGenerator

      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Units.Pressure P_steam(start=70e5) "Pa";
      input Units.PositiveMassFlowRate Q_feedwater(start=500) "kg/s";
      input Units.Pressure P_feedwater(start=80e5) "Pa";
      input Units.Temperature T_feedwater(start=225+273.15) "K";
      input Units.PositiveMassFlowRate Q_purge(start=1) "kg/s";

      // Parameters
      input Real vapor_fraction(start=0.99);

      .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.SteamGenerator steamGenerator annotation (Placement(transformation(extent={{-32,-60},{32,60}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source feedwater_source annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={58,0})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink steam_sink annotation (Placement(transformation(extent={{48,70},{68,90}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink purge_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-78})));
    equation

      // Boundary conditions
      steam_sink.P_in = P_steam;
      feedwater_source.Q_out = - Q_feedwater;
      feedwater_source.P_out = P_feedwater;
      feedwater_source.T_out = T_feedwater;
      purge_sink.Q_in = Q_purge;

      // Parameters
      steamGenerator.vapor_fraction = 0.99;

      // Hypothesis
      steamGenerator.P_purge = P_steam; // Steam generator blowdown is assumed to be at the saturation pressure

      connect(feedwater_source.C_out, steamGenerator.feedwater_inlet) annotation (
          Line(points={{53,5.55112e-17},{34.5,5.55112e-17},{34.5,0},{16,0}},
                                                             color={28,108,200}));
      connect(steamGenerator.steam_outlet, steam_sink.C_in)
        annotation (Line(points={{0,60},{0,80},{53,80}},   color={28,108,200}));
      connect(purge_sink.C_in, steamGenerator.purge_outlet) annotation (Line(points={{8.88178e-16,-73},{8.88178e-16,-59},{0,-59}},
                                                                   color={28,108,200}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end SteamGenerator;

    model DryReheater_direct

      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Real P_hot_source(start=11, min=0, nominal=11) "bar";
      input Real P_cold_source(start=50, min=0, nominal=50) "bar";
      input Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
      input Real T_cold_in(start=50) "degC";
      input Units.SpecificEnthalpy hot_source_h_out(start=2.5e6) "J/kg";

      // Parameters
      parameter Units.Area S = 100;
      parameter Units.HeatExchangeCoefficient Kth = 50e3;
      parameter Units.FrictionCoefficient Kfr_hot = 0;
      parameter Units.FrictionCoefficient Kfr_cold = 500; // About 1 bar of pressure loss in the reheater

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.DryReheater dryReheater annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,30})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-30})));
    equation

      // Boundary conditions
      hot_source.P_out = P_hot_source*1e5;
      hot_source.h_out = hot_source_h_out;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = T_cold_in + 273.15;
      cold_source.Q_out = -Q_cold;

      // Component parameters
      dryReheater.S_condensing = S;
      dryReheater.Kth = Kth;
      dryReheater.Kfr_hot = Kfr_hot;
      dryReheater.Kfr_cold = Kfr_cold;

      connect(dryReheater.C_cold_out, cold_sink.C_in)
        annotation (Line(points={{16,0},{45,0}}, color={28,108,200}));
      connect(hot_sink.C_in, dryReheater.C_hot_out) annotation (Line(points={{8.88178e-16,
              -25},{8.88178e-16,-20.5},{0,-20.5},{0,-8}}, color={28,108,200}));
      connect(hot_source.C_out, dryReheater.C_hot_in) annotation (Line(points={{-8.88178e-16,
              25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
      connect(cold_source.C_out, dryReheater.C_cold_in)
        annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
    end DryReheater_direct;

    model DryReheater_reverse

      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Real P_hot_source(start=11, min=0, nominal=11) "bar";
      input Real P_cold_source(start=50, min=0, nominal=50) "bar";
      input Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
      input Real T_cold_in(start=50) "degC";
      input Units.SpecificEnthalpy hot_source_h_out(start=2.9e6) "J/kg";

      // Component Parameters
      parameter Units.Area S = 100;
      parameter Units.FrictionCoefficient Kfr_hot = 0;

      // Observables for calibration
      input Real P_cold_sink(start=49, min=0, nominal=50) "bar";
      input Real T_cold_sink(start=70, min=0, nominal=200) "degC";

      // Calibrated parameters
      output Units.HeatExchangeCoefficient Kth;
      output Units.FrictionCoefficient Kfr_cold;

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{68,-10},{88,10}})));
      .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.DryReheater dryReheater annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,30})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-36})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_cold_sink_sensor annotation (Placement(transformation(extent={{48,-10},{68,10}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cold_sink_sensor annotation (Placement(transformation(extent={{22,-10},{42,10}})));
    equation

      // Boundary conditions
      hot_source.P_out = P_hot_source*1e5;
      hot_source.h_out = hot_source_h_out;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = T_cold_in + 273.15;
      cold_source.Q_out = -Q_cold;

      // Component parameters
      dryReheater.S_condensing = S;
      dryReheater.Kfr_hot = Kfr_hot;

      // Observables for calibration
      P_cold_sink_sensor.P_barA = P_cold_sink;
      T_cold_sink_sensor.T_degC = T_cold_sink;

      // Calibrated parameters
      dryReheater.Kth = Kth;
      dryReheater.Kfr_cold = Kfr_cold;

      connect(hot_source.C_out, dryReheater.C_hot_in) annotation (Line(points={{-8.88178e-16,
              25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
      connect(cold_source.C_out, dryReheater.C_cold_in)
        annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
      connect(dryReheater.C_cold_out, P_cold_sink_sensor.C_in) annotation (Line(points={{16,0},{22,0}}, color={28,108,200}));
      connect(P_cold_sink_sensor.C_out, T_cold_sink_sensor.C_in) annotation (Line(points={{42,0},{48,0}}, color={28,108,200}));
      connect(T_cold_sink_sensor.C_out, cold_sink.C_in) annotation (Line(points={{68,0},{73,0}}, color={28,108,200}));
      connect(dryReheater.C_hot_out, hot_sink.C_in) annotation (Line(points={{0,-8},
              {0,-19.5},{8.88178e-16,-19.5},{8.88178e-16,-31}}, color={28,108,200}));
    end DryReheater_reverse;

    model DryReheater_faulty
      extends DryReheater_direct(
          dryReheater(faulty = true));

      Real Failure_fouling(start=0);

    equation

      // Failure input
      Failure_fouling = 0 + 10*time;

      // Failure definition
      dryReheater.fouling = Failure_fouling;

    end DryReheater_faulty;

    model Condenser_direct

      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

        // Boundary conditions
      input Units.MassFlowRate Q_turbine(start=150) "kg/s";
      input Units.SpecificEnthalpy h_turbine(start=1500e3);

      input Real P_cold_source(start=5, min=0, nominal=10) "barA";
      input Real T_cold_source(start = 15, min = 0, nominal = 50) "degC";

        // Parameters
      parameter Units.Area S = 100;
      parameter Units.HeatExchangeCoefficient Kth = 50000;
      parameter Units.Height water_height = 2;
      parameter Units.FrictionCoefficient Kfr_cold = 1;
      parameter Units.VolumeFlowRate Qv_cold = 3.82;
      parameter Units.Pressure P_offset = 0;
      parameter Units.Pressure C_incond = 0;

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cooling_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cooling_sink annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_outlet annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,30})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink condensate_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-30})));
    equation

      turbine_outlet.h_out = h_turbine;
      turbine_outlet.Q_out = -Q_turbine;

      cooling_source.P_out = P_cold_source*1e5;
      cooling_source.T_out = 273.15 + T_cold_source;

      condenser.S = S;
      condenser.Kth = Kth;
      condenser.water_height = water_height;
      condenser.Kfr_cold = Kfr_cold;
      condenser.Qv_cold_in = Qv_cold;
      condenser.P_offset = P_offset;
      condenser.C_incond = C_incond;

      connect(condenser.C_cold_out, cooling_sink.C_in) annotation (Line(points={{16,
              -1.42222},{30,-1.42222},{30,0},{45,0}}, color={28,108,200}));
      connect(condensate_sink.C_in, condenser.C_hot_out) annotation (Line(points={{8.88178e-16,
              -25},{8.88178e-16,-20.5},{0,-20.5},{0,-8.35556}}, color={28,108,200}));
      connect(turbine_outlet.C_out, condenser.C_hot_in) annotation (Line(points={{-8.88178e-16,
              25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
      connect(cooling_source.C_out, condenser.C_cold_in) annotation (Line(points={{-43,
              0},{-30,0},{-30,3.55556},{-16.64,3.55556}}, color={28,108,200}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,80}})),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}})));
    end Condenser_direct;

    model Condenser_reverse

      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

        // Boundary conditions
      input Units.MassFlowRate Q_turbine(start=150) "kg/s";
      input Units.SpecificEnthalpy h_turbine(start=1500e3);
      input Real P_cold_source(start=5, min=0, nominal=10) "barA";
      input Real T_cold_source(start = 15, min = 0, nominal = 50) "degC";

        // Parameters
      parameter Units.Area S = 100;
      parameter Units.Height water_height = 1;
      parameter Units.Pressure C_incond = 0;
      parameter Units.Pressure P_offset = 0;
      parameter Units.FrictionCoefficient Kfr_cold = 1;
      parameter Units.VolumeFlowRate Qv_cold = 3.82;

        // Calibrated parameters
      output Units.HeatExchangeCoefficient Kth;

        // Inputs for calibration
      input Real P_cond(start = 0.19e5) "Pa";

      .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Condenser condenser annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source turbine_outlet annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,46})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink condensate_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-44})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cond_sensor annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=270,
            origin={1,25})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor circulating_water_T_out_sensor annotation (Placement(transformation(extent={{24,-6},{34,4}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor condensate_temperature_sensor annotation (Placement(transformation(
            extent={{-5,-5},{5,5}},
            rotation=270,
            origin={-1,-21})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor circulating_water_P_out_sensor annotation (Placement(transformation(extent={{40,-6},{50,4}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cooling_source annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-56,4})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cooling_sink annotation (Placement(transformation(extent={{60,-10},{80,10}})));
    equation

      // Boundary Conditions
      turbine_outlet.h_out = h_turbine;
      turbine_outlet.Q_out = -Q_turbine;

      cooling_source.P_out = P_cold_source*1e5;
      cooling_source.T_out = 273.15 + T_cold_source;

      // Parameters
      condenser.S = S;
      condenser.water_height = water_height;
      condenser.C_incond = C_incond;
      condenser.P_offset = P_offset;
      condenser.Kfr_cold = Kfr_cold;
      condenser.Qv_cold_in = Qv_cold;

      // Calibrated Parameters
      condenser.Kth = Kth;

      // Inputs for calibration
      P_cond_sensor.P = P_cond;

      connect(condenser.C_hot_in, P_cond_sensor.C_out)
        annotation (Line(points={{0,8},{0,20},{1,20}}, color={28,108,200}));
      connect(turbine_outlet.C_out, P_cond_sensor.C_in) annotation (Line(points={{-8.88178e-16,
              41},{0,41},{0,30},{1,30}}, color={28,108,200}));
      connect(condenser.C_cold_out, circulating_water_T_out_sensor.C_in)
        annotation (Line(points={{16,-1.42222},{16,-1},{24,-1}}, color={28,108,200}));
      connect(condensate_temperature_sensor.C_in, condenser.C_hot_out) annotation (
          Line(points={{-1,-16},{0,-16},{0,-8.35556}}, color={28,108,200}));
      connect(condensate_temperature_sensor.C_out, condensate_sink.C_in)
        annotation (Line(points={{-1,-26},{-1,-32.5},{8.88178e-16,-32.5},{8.88178e-16,
              -39}}, color={28,108,200}));
      connect(circulating_water_P_out_sensor.C_in, circulating_water_T_out_sensor.C_out)
        annotation (Line(points={{40,-1},{34,-1}}, color={28,108,200}));
      connect(condenser.C_cold_in, cooling_source.C_out) annotation (Line(points={{-16.64,3.55556},{-34,3.55556},{-34,4},{-51,4}}, color={28,108,200}));
      connect(circulating_water_P_out_sensor.C_out, cooling_sink.C_in) annotation (Line(points={{50,-1},{57.5,-1},{57.5,0},{65,0}}, color={28,108,200}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,80}})),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}})));
    end Condenser_reverse;

    model Condenser_faulty
      extends Condenser_direct(
          condenser(faulty = true));

      Real Failure_fouling(start=0);
      Real Failure_air_intake(start=0);

    equation

      // Failure input
      Failure_fouling = 0 + 10*time;
      Failure_air_intake = 0 + 1e-3 * time;

      // Failure definition
      condenser.fouling = Failure_fouling;
      condenser.air_intake = Failure_air_intake;

    end Condenser_faulty;

    model Reheater_direct

      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Real P_hot_source(start=11, min=0, nominal=11) "bar";
      input Real P_cold_source(start=50, min=0, nominal=50) "bar";
      input Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
      input Real T_cold_in(start=50) "degC";
      input Units.SpecificEnthalpy hot_source_h_out(start=2.5e6) "J/kg";

      // Parameters
      parameter Units.Area S_tot = 100;
      parameter Units.Area level = 0.3;
      parameter Units.HeatExchangeCoefficient Kth_cond = 61e3;
      parameter Units.HeatExchangeCoefficient Kth_subc = 8e3;
      parameter Units.FrictionCoefficient Kfr_hot = 0;
      parameter Units.FrictionCoefficient Kfr_cold = 0;

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater reheater annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,30})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-30})));
    equation

      // Boundary conditions
      hot_source.P_out = P_hot_source*1e5;
      hot_source.h_out = hot_source_h_out;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = T_cold_in + 273.15;
      cold_source.Q_out = -Q_cold;

      // Component parameters
      reheater.S_tot = S_tot;
      reheater.Kth_cond = Kth_cond;
      reheater.Kth_subc = Kth_subc;
      reheater.Kfr_hot = Kfr_hot;
      reheater.Kfr_cold = Kfr_cold;
      reheater.level = level;

      connect(reheater.C_cold_out, cold_sink.C_in)
        annotation (Line(points={{16,0},{45,0}}, color={28,108,200}));
      connect(hot_sink.C_in, reheater.C_hot_out) annotation (Line(points={{8.88178e-16,
              -25},{8.88178e-16,-20.5},{0,-20.5},{0,-8}}, color={28,108,200}));
      connect(hot_source.C_out, reheater.C_hot_in) annotation (Line(points={{-8.88178e-16,
              25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
      connect(cold_source.C_out, reheater.C_cold_in)
        annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
    end Reheater_direct;

    model Reheater_reverse

      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary conditions
      input Real P_hot_source(start=11, min=0, nominal=11) "bar";
      input Real P_cold_source(start=50, min=0, nominal=50) "bar";
      input Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
      input Real T_cold_in(start=50) "degC";
      input Units.SpecificEnthalpy hot_source_h_out(start=2.9e6) "J/kg";

      // Component Parameters
      parameter Units.Area S_tot = 100;
      parameter Units.Area level = 0.3;

      parameter Units.FrictionCoefficient Kfr_hot = 0;

      // Observables for calibration
      input Real P_cold_sink(start=49, min=0, nominal=50) "bar";
      input Real T_cold_sink(start=70, min=0, nominal=200) "degC";
      input Real T_drains(start=80, min=0, nominal=200) "degC";

      // Calibrated parameters
      output Units.HeatExchangeCoefficient Kth_cond;
      output Units.FrictionCoefficient Kfr_cold;
      output Units.HeatExchangeCoefficient Kth_subc;

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink cold_sink annotation (Placement(transformation(extent={{68,-10},{88,10}})));
      .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Reheater reheater annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_source annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,30})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink hot_sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-56})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_cold_sink_sensor annotation (Placement(transformation(extent={{48,-10},{68,10}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor P_cold_sink_sensor annotation (Placement(transformation(extent={{22,-10},{42,10}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor T_drains_sensor annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={0,-30})));
    equation

      // Boundary conditions
      hot_source.P_out = P_hot_source*1e5;
      hot_source.h_out = hot_source_h_out;

      cold_source.P_out = P_cold_source*1e5;
      cold_source.T_out = T_cold_in + 273.15;
      cold_source.Q_out = -Q_cold;

      // Component parameters
      reheater.S_tot = S_tot;
      reheater.Kfr_hot = Kfr_hot;
      reheater.level = level;

      // Observables for calibration
      P_cold_sink_sensor.P_barA = P_cold_sink;
      T_cold_sink_sensor.T_degC = T_cold_sink;
      T_drains_sensor.T_degC = T_drains;

      // Calibrated parameters
      reheater.Kth_cond = Kth_cond;
      reheater.Kfr_cold = Kfr_cold;
      reheater.Kth_subc = Kth_subc;

      connect(hot_source.C_out, reheater.C_hot_in) annotation (Line(points={{-8.88178e-16,
              25},{-8.88178e-16,16.5},{0,16.5},{0,8}}, color={28,108,200}));
      connect(cold_source.C_out, reheater.C_cold_in)
        annotation (Line(points={{-43,0},{-16.2,0}}, color={28,108,200}));
      connect(reheater.C_cold_out, P_cold_sink_sensor.C_in)
        annotation (Line(points={{16,0},{22,0}}, color={28,108,200}));
      connect(P_cold_sink_sensor.C_out, T_cold_sink_sensor.C_in) annotation (Line(points={{42,0},{48,0}}, color={28,108,200}));
      connect(T_cold_sink_sensor.C_out, cold_sink.C_in) annotation (Line(points={{68,0},{73,0}}, color={28,108,200}));
      connect(T_drains_sensor.C_in, reheater.C_hot_out) annotation (Line(points={{1.77636e-15,
              -20},{1.77636e-15,-14},{0,-14},{0,-8}}, color={28,108,200}));
      connect(T_drains_sensor.C_out, hot_sink.C_in) annotation (Line(points={{-1.77636e-15,
              -40},{-1.77636e-15,-45.5},{8.88178e-16,-45.5},{8.88178e-16,-51}},
            color={28,108,200}));
    end Reheater_reverse;

    model Reheater_faulty
      extends Reheater_direct(
          reheater(faulty = true));

      Real Failure_fouling(start=0);
      Real Failure_water_level_rise(start=0);

    equation

      // Failure input
      Failure_fouling = 0 + 10*time;
      Failure_water_level_rise = 0 - 0.1*time;

      // Failure definition
      reheater.fouling = Failure_fouling;
      reheater.water_level_rise = Failure_water_level_rise;

    end Reheater_faulty;

    model Superheater_direct

      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

        // Boundary conditions
      input Real P_hot_steam(start=60, min=0, nominal=11) "bar";
      input Real P_cold_steam(start=11, min=0, nominal=50) "bar";
      input Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
      input Real h_cold_steam(start=2.75e6) "J/kg"; // slightly humid cold steam
      input Real h_hot_steam(start=2.8e6) "J/kg"; // slightly superheated hot steam

      // Parameters
      parameter Units.Area S = 100;
      parameter Units.HeatExchangeCoefficient Kth = 7e3;
      parameter Units.FrictionCoefficient Kfr_cold = 0;
      parameter Units.FrictionCoefficient Kfr_hot = 500; // About 1 bar of pressure loss in the reheater
      parameter Units.PositiveMassFlowRate Q_vent=1;

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_steam_source annotation (Placement(transformation(extent={{-68,-10},{-48,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink drains_sink annotation (Placement(transformation(extent={{48,-10},{68,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_steam_source annotation (Placement(transformation(extent={{-42,-50},{-22,-30}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink superheated_steam_sink annotation (Placement(transformation(extent={{16,30},{36,50}})));
      .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Superheater superheater annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink vent_sink annotation (Placement(transformation(extent={{48,-30},{68,-10}})));
    equation

      // Boundary conditions
      hot_steam_source.P_out = P_hot_steam*1e5;
      hot_steam_source.h_out = h_hot_steam;

      cold_steam_source.P_out = P_cold_steam*1e5;
      cold_steam_source.h_out = h_cold_steam;
      cold_steam_source.Q_out = - Q_cold;

      // Component parameters
      superheater.Kth = Kth;
      superheater.S = S;
      superheater.Kfr_cold=Kfr_cold;
      superheater.Kfr_hot=Kfr_hot;
      superheater.Q_vent = Q_vent;

      connect(superheater.C_cold_out, superheated_steam_sink.C_in) annotation (Line(
            points={{-0.2,8},{0,8},{0,40},{21,40}}, color={28,108,200}));
      connect(superheater.C_hot_out, drains_sink.C_in)
        annotation (Line(points={{16,0},{53,0}}, color={28,108,200}));
      connect(cold_steam_source.C_out,superheater. C_cold_in)
        annotation (Line(points={{-27,-40},{0,-40},{0,-8}}, color={28,108,200}));
      connect(hot_steam_source.C_out,superheater. C_hot_in) annotation (Line(points=
             {{-53,0},{-34.5,0},{-34.5,0.2},{-16,0.2}}, color={28,108,200}));
      connect(vent_sink.C_in, superheater.C_vent) annotation (Line(points={{53,-20},
              {16,-20},{16,-7.8}}, color={28,108,200}));
    end Superheater_direct;

    model Superheater_reverse

      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

        // Boundary conditions
      input Real P_hot_steam(start=60, min=0, nominal=11) "bar";
      input Real P_cold_steam(start=11, min=0, nominal=50) "bar";
      input Units.PositiveMassFlowRate Q_cold(start=500) "kg/s";
      input Real h_cold_steam(start=2.75e6) "J/kg"; // slightly humid cold steam
      input Real h_hot_steam(start=2.8e6) "J/kg"; // slightly superheated hot steam

      // Parameters
      parameter Units.Area S = 100;

      parameter Units.FrictionCoefficient Kfr_cold = 0;

      parameter Units.PositiveMassFlowRate Q_vent=1;

      // Inputs for calibration
      input Real superheated_steam_temperature(start=224, min=0, nominal = 200) "degC";
      input Real drains_pressure(start=59.5, min=0, nominal = 100e5) "barA";

      // Calibrated parameters
      output Units.HeatExchangeCoefficient Kth;
      output Units.FrictionCoefficient Kfr_hot;

      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source hot_steam_source annotation (Placement(transformation(extent={{-70,-2},{-50,18}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink drains_sink annotation (Placement(transformation(extent={{70,-10},{90,10}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source cold_steam_source annotation (Placement(transformation(extent={{-42,-50},{-22,-30}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink superheated_steam_sink annotation (Placement(transformation(extent={{16,30},{36,50}})));
      .MetroscopeModelingLibrary.WaterSteam.HeatExchangers.Superheater superheater annotation (Placement(transformation(extent={{-16,-8},{16,8}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink vent_sink annotation (Placement(transformation(extent={{70,-30},{90,-10}})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.TemperatureSensor superheated_steam_temperature_sensor annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={0,24})));
      MetroscopeModelingLibrary.Sensors.WaterSteam.PressureSensor drains_pressure_sensor annotation (Placement(transformation(extent={{38,-10},{58,10}})));
    equation

      // Boundary conditions
      hot_steam_source.P_out = P_hot_steam*1e5;
      hot_steam_source.h_out = h_hot_steam;
      cold_steam_source.P_out = P_cold_steam*1e5;
      cold_steam_source.h_out = h_cold_steam;
      cold_steam_source.Q_out = - Q_cold;

      // Component parameters
      superheater.S = S;
      superheater.Kfr_cold=Kfr_cold;
      superheater.Q_vent = Q_vent;

      // Inputs for calibration
      drains_pressure_sensor.P_barA = drains_pressure;
      superheated_steam_temperature_sensor.T_degC = superheated_steam_temperature;

      // Calibrated parameters
      superheater.Kth = Kth;
      superheater.Kfr_hot=Kfr_hot;

      connect(cold_steam_source.C_out,superheater. C_cold_in)
        annotation (Line(points={{-27,-40},{0,-40},{0,-8}}, color={28,108,200}));
      connect(hot_steam_source.C_out,superheater. C_hot_in) annotation (Line(points={{-55,8},
              {-34.5,8},{-34.5,0.2},{-16,0.2}},         color={28,108,200}));
      connect(vent_sink.C_in, superheater.C_vent) annotation (Line(points={{75,-20},
              {20,-20},{20,-7.8},{16,-7.8}},
                                   color={28,108,200}));
      connect(superheater.C_cold_out, superheated_steam_temperature_sensor.C_in)
        annotation (Line(points={{-0.2,8},{-5.55112e-16,8},{-5.55112e-16,14}},
            color={28,108,200}));
      connect(superheated_steam_temperature_sensor.C_out, superheated_steam_sink.C_in)
        annotation (Line(points={{5.55112e-16,34},{5.55112e-16,40},{21,40}}, color={
              28,108,200}));
      connect(superheater.C_hot_out, drains_pressure_sensor.C_in)
        annotation (Line(points={{16,0},{38,0}}, color={28,108,200}));
      connect(drains_pressure_sensor.C_out, drains_sink.C_in)
        annotation (Line(points={{58,0},{75,0}}, color={28,108,200}));
    end Superheater_reverse;

    model Superheater_faulty
      extends Superheater_direct(
          superheater(faulty = true));

      Real Failure_fouling(start=0);
      Real Failure_closed_vent(start=0);

    equation

      // Failure input
      Failure_fouling = 0 + 10*time;
      Failure_closed_vent = 0 + 100*time; // Fully closed vent at end of simulation

      // Failure definition
      superheater.fouling = Failure_fouling;
      superheater.closed_vent = Failure_closed_vent;

    end Superheater_faulty;
  end HeatExchangers;

  package Volumes
    extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestPackageIcon;

    model FlashTank
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary Conditions
      input Units.Pressure P_source(start = 10e5) "Pa";
      input Units.NegativeMassFlowRate Q_source(start=-500) "kg/s";
      input Units.SpecificEnthalpy h_source(start=2e6) "J/kg";
      .MetroscopeModelingLibrary.WaterSteam.Volumes.FlashTank flashTank annotation (Placement(transformation(extent={{-32,-30},{28,30}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-80,2},{-60,22}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink steam_sink annotation (Placement(transformation(extent={{56,2},{76,22}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink liquid_sink annotation (Placement(transformation(extent={{56,-22},{76,-2}})));
    equation

      source.P_out = P_source;
      source.Q_out = Q_source;
      source.h_out = h_source;

      connect(flashTank.C_in, source.C_out)
        annotation (Line(points={{-32,12},{-65,12}}, color={28,108,200}));
      connect(flashTank.C_hot_steam, steam_sink.C_in)
        annotation (Line(points={{28,12},{61,12}}, color={28,108,200}));
      connect(flashTank.C_hot_liquid, liquid_sink.C_in)
        annotation (Line(points={{28,-12},{61,-12}}, color={28,108,200}));
    end FlashTank;

    model SteamDryer
      extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

      // Boundary Conditions
      input Units.Pressure P_source(start = 10e5) "Pa";
      input Units.NegativeMassFlowRate Q_source(start=-500) "kg/s";
      input Units.SpecificEnthalpy h_source(start=2e6) "J/kg";

      // Component parameters
      parameter Real x_steam_out = 0.9;

      .MetroscopeModelingLibrary.WaterSteam.Volumes.SteamDryer steamDryer annotation (Placement(transformation(extent={{-32,-26},{28,34}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-80,2},{-60,22}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink steam_sink annotation (Placement(transformation(extent={{56,2},{76,22}})));
      .MetroscopeModelingLibrary.WaterSteam.BoundaryConditions.Sink liquid_sink annotation (Placement(transformation(extent={{56,-50},{76,-30}})));
    equation

      // Boundary conditions
      source.P_out = P_source;
      source.Q_out = Q_source;
      source.h_out = h_source;

      // Component parameters
      steamDryer.x_steam_out = x_steam_out;

      connect(steamDryer.C_in, source.C_out) annotation (Line(points={{-32,12.1818},{-32,12},{-65,12}},
                                                color={28,108,200}));
      connect(steamDryer.C_hot_steam, steam_sink.C_in) annotation (Line(points={{28,12.1818},{28,12},{61,12}},
                                                      color={28,108,200}));
      connect(steamDryer.C_hot_liquid, liquid_sink.C_in) annotation (Line(points={{28,-9.63636},{28,-8},{52,-8},{52,-40},{61,-40}},
                                                          color={28,108,200}));
    end SteamDryer;
  end Volumes;
end WaterSteam;
