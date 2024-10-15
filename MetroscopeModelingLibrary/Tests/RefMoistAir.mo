within MetroscopeModelingLibrary.Tests;
package RefMoistAir
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestPackageIcon;

  package BoundaryConditions
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestPackageIcon;

    model Source
      extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
      import MetroscopeModelingLibrary.Utilities.Units;

      // Boundary conditinos
      input Units.Pressure source_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
    equation
      source.P_out = source_P;
      source.h_out = source_h;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={0,255,128}));
    end Source;

    model Sink
      extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
      import MetroscopeModelingLibrary.Utilities.Units;

      // Boundary conditinos
      input Units.Pressure sink_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy sink_h(start=1e3) "J/kg";
      input Units.PositiveMassFlowRate sink_Q(start=100) "kg/s";
      input Units.Fraction sink_relative_humidity(start=0.5) "1";

      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
    equation
      sink.P_in = sink_P;
      sink.h_in = sink_h;
      sink.Q_in = sink_Q;
      sink.relative_humidity = sink_relative_humidity;

      connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={0,255,128}));
    end Sink;
  end BoundaryConditions;

  package Machines
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestPackageIcon;

    model AirCompressor_reverse
      extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
      import MetroscopeModelingLibrary.Utilities.Units;

      // Boundary conditinos
      input Units.Pressure source_P(start=1e5) "Pa";
      //input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.Temperature source_T(start=293.15);
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      // Inputs for calibration
      input Real compressor_T_out(start = 406) "degC";
      input Real compressor_P_out(start = 17) "barA";

      // Parameters to calibrate
      output Real compression_rate;
      output Real eta_is;

      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{74,-10},{94,10}})));
      MetroscopeModelingLibrary.RefMoistAir.Machines.AirCompressor airCompressor annotation (Placement(transformation(extent={{-40,-8},{-20,8}})));
      MetroscopeModelingLibrary.Power.BoundaryConditions.Source power_source annotation (Placement(transformation(extent={{22,30},{2,50}})));
      MetroscopeModelingLibrary.Sensors.RefMoistAir.TemperatureSensor compressor_T_out_sensor annotation (Placement(transformation(extent={{10,-10},{30,10}})));
      MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor compressor_P_out_sensor annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    equation
      // Boundary conditions
      source.P_out = source_P;
      source.T_out = source_T;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Inputs for calibration
      compressor_T_out_sensor.T_degC = compressor_T_out;
      compressor_P_out_sensor.P_barA = compressor_P_out;

      // Parameters to calibrate
      airCompressor.tau = compression_rate;
      airCompressor.eta_is = eta_is;

      connect(source.C_out, airCompressor.C_in) annotation (Line(points={{-79,0},{-40,0}}, color={0,255,128}));
      connect(airCompressor.C_W_in, power_source.C_out) annotation (Line(points={{-20,6},{-20,40},{7.2,40}}, color={244,125,35}));
      connect(airCompressor.C_out, compressor_T_out_sensor.C_in) annotation (Line(points={{-20,0},{10,0}}, color={0,255,128}));
      connect(compressor_T_out_sensor.C_out, compressor_P_out_sensor.C_in) annotation (Line(points={{30,0},{40,0}}, color={0,255,128}));
      connect(compressor_P_out_sensor.C_out, sink.C_in) annotation (Line(points={{60,0},{79,0}}, color={0,255,128}));
    end AirCompressor_reverse;

    model AirCompressor_direct
      extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
      import MetroscopeModelingLibrary.Utilities.Units;

      // Boundary conditinos
      input Units.Pressure source_P(start=1e5) "Pa";
      //input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.Temperature source_T(start=293.15);
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      // Inputs for calibration
      output Real compressor_T_out "degC";
      output Real compressor_P_out "barA";

      // Parameters to calibrate
      output Real compression_rate = 17;
      output Real eta_is = 0.917589;

      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{74,-10},{94,10}})));
      MetroscopeModelingLibrary.RefMoistAir.Machines.AirCompressor airCompressor annotation (Placement(transformation(extent={{-40,-8},{-20,8}})));
      MetroscopeModelingLibrary.Power.BoundaryConditions.Source power_source annotation (Placement(transformation(extent={{22,30},{2,50}})));
      MetroscopeModelingLibrary.Sensors.RefMoistAir.TemperatureSensor compressor_T_out_sensor annotation (Placement(transformation(extent={{10,-10},{30,10}})));
      MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor compressor_P_out_sensor annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    equation
      // Boundary conditions
      source.P_out = source_P;
      source.T_out = source_T;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Inputs for calibration
      compressor_T_out_sensor.T_degC = compressor_T_out;
      compressor_P_out_sensor.P_barA = compressor_P_out;

      // Parameters to calibrate
      airCompressor.tau = compression_rate;
      airCompressor.eta_is = eta_is;

      connect(source.C_out, airCompressor.C_in) annotation (Line(points={{-79,0},{-40,0}}, color={0,255,128}));
      connect(airCompressor.C_W_in, power_source.C_out) annotation (Line(points={{-20,6},{-20,40},{7.2,40}}, color={244,125,35}));
      connect(airCompressor.C_out, compressor_T_out_sensor.C_in) annotation (Line(points={{-20,0},{10,0}}, color={0,255,128}));
      connect(compressor_T_out_sensor.C_out, compressor_P_out_sensor.C_in) annotation (Line(points={{30,0},{40,0}}, color={0,255,128}));
      connect(compressor_P_out_sensor.C_out, sink.C_in) annotation (Line(points={{60,0},{79,0}}, color={0,255,128}));
    end AirCompressor_direct;
  end Machines;

  package BaseClasses
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestPackageIcon;

    model FlowModel
      extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

      import MetroscopeModelingLibrary.Utilities.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      // Parameters
      input Units.DifferentialPressure DP(start=0.1e5);
      input Units.Power W(start=1e5);

      .MetroscopeModelingLibrary.RefMoistAir.BaseClasses.FlowModel flowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      flowModel.DP = DP;
      flowModel.W = W;

      connect(flowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={0,255,128}));
      connect(flowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={0,255,128}));
    end FlowModel;

    model IsoPFlowModel
      extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

      import MetroscopeModelingLibrary.Utilities.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      // Parameters
      input Units.Power W(start=1e5);

      MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPFlowModel
                                                                 isoPFlowModel
                                                                           annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      isoPFlowModel.W = W;

      connect(isoPFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={0,255,128}));
      connect(isoPFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={0,255,128}));
    end IsoPFlowModel;

    model IsoHFlowModel
      extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

      import MetroscopeModelingLibrary.Utilities.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      // Parameters
      input Units.DifferentialPressure DP(start=0.1e5);

      MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoHFlowModel
                                                                 isoHFlowModel
                                                                           annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      isoHFlowModel.DP = DP;

      connect(isoHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={0,255,128}));
      connect(isoHFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={0,255,128}));
    end IsoHFlowModel;

    model IsoPHFlowModel
      extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

      import MetroscopeModelingLibrary.Utilities.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoPHFlowModel
                                                                 isoPHFlowModel
                                                                           annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      connect(isoPHFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={0,255,128}));
      connect(isoPHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={0,255,128}));
    end IsoPHFlowModel;
  end BaseClasses;

  package Pipes
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestPackageIcon;

    model Pipe_direct
        extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
          // Boundary conditions
      input Utilities.Units.Pressure source_P(start=10e5) "Pa";
      input Utilities.Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
      input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";

        // Parameters
      parameter Utilities.Units.FrictionCoefficient Kfr=100;
      parameter Utilities.Units.Height delta_z=0;

      MetroscopeModelingLibrary.RefMoistAir.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
    equation
      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      pipe.Kfr = Kfr;
      pipe.delta_z = delta_z;

      connect(source.C_out, pipe.C_in) annotation (Line(points={{-37,0},{-10,0}}, color={0,255,128}));
      connect(pipe.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={0,255,128}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Pipe_direct;

    model Pipe_reverse
      extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
          // Boundary conditions
      input Utilities.Units.Pressure source_P(start=10e5) "Pa";
      input Utilities.Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
      input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";

        // Parameters
      parameter Utilities.Units.Height delta_z=1;

      // Inputs for calibration
      input Real P_out(start=9) "barA";

      // Parameters for calibration
      output Utilities.Units.FrictionCoefficient Kfr;

      MetroscopeModelingLibrary.RefMoistAir.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{50,-10},{70,10}})));
      MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor P_out_sensor annotation (Placement(transformation(extent={{24,-10},{44,10}})));
    equation
      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      pipe.delta_z = delta_z;

      // Inputs for calibration
      P_out_sensor.P_barA = P_out;

      // Parameters for calibration
      pipe.Kfr = Kfr;

      connect(source.C_out, pipe.C_in) annotation (Line(points={{-37,0},{-10,0}}, color={0,255,128}));
      connect(pipe.C_out, P_out_sensor.C_in) annotation (Line(points={{10,0},{24,0},{24,0}}, color={0,255,128}));
      connect(P_out_sensor.C_out, sink.C_in) annotation (Line(points={{44,0},{55,0}}, color={0,255,128}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Pipe_reverse;

    model AdmiLouvers_direct
        extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
          // Boundary conditions
      input Utilities.Units.Pressure source_P(start=10e5) "Pa";
      input Utilities.Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
      input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";

        // Parameters
      parameter Utilities.Units.FrictionCoefficient Kfr=100;
      parameter Utilities.Units.Height delta_z=0;

      MetroscopeModelingLibrary.RefMoistAir.Pipes.AdmiLouver
                                                    admiLouver
                                                         annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    equation
      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      admiLouver.Kfr = Kfr;
      admiLouver.delta_z = delta_z;
      connect(source.C_out, admiLouver.C_in) annotation (Line(points={{-31,0},{-10,0}}, color={0,255,128}));
      connect(admiLouver.C_out, sink.C_in) annotation (Line(points={{10,0},{45,0}}, color={0,255,128}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end AdmiLouvers_direct;

    model AdmiLouvers_reverse
      extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
          // Boundary conditions
      input Utilities.Units.Pressure source_P(start=10e5) "Pa";
      input Utilities.Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
      input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";

        // Parameters
      parameter Utilities.Units.Height delta_z=1;

      // Inputs for calibration
      input Real P_out(start=9) "barA";

      // Parameters for calibration
      output Utilities.Units.FrictionCoefficient Kfr;
      MetroscopeModelingLibrary.RefMoistAir.Pipes.AdmiLouver
                                                    admiLouver
                                                         annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{48,-10},{68,10}})));
      MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor P_out_sensor annotation (Placement(transformation(extent={{22,-10},{42,10}})));
    equation
        // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      admiLouver.delta_z = delta_z;

      // Inputs for calibration
      P_out_sensor.P_barA = P_out;

      // Parameters for calibration
      admiLouver.Kfr = Kfr;
      connect(P_out_sensor.C_out, sink.C_in) annotation (Line(points={{42,0},{53,0}}, color={0,255,128}));
      connect(admiLouver.C_out, P_out_sensor.C_in) annotation (Line(points={{8,0},{22,0}}, color={0,255,128}));
      connect(source.C_out, admiLouver.C_in) annotation (Line(points={{-39,0},{-12,0}}, color={0,255,128}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end AdmiLouvers_reverse;

    model HeatLoss
        extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
          // Boundary conditions
      input Utilities.Units.Pressure source_P(start=1e5) "Pa";
      input Utilities.Units.SpecificEnthalpy source_h(start=4.5e4) "J/kg";
      input Utilities.Units.PositiveMassFlowRate source_Q(start=500) "kg/s";
      input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";
      input Utilities.Units.Power W(start=5e6) "W";

      MetroscopeModelingLibrary.RefMoistAir.Pipes.HeatLoss heat_loss annotation (Placement(transformation(extent={{-16,-16},{14,16}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{80,-10},{100,10}})));
    equation
      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = -source_Q;
      source.relative_humidity = source_relative_humidity;
      heat_loss.W = W;

      connect(source.C_out, heat_loss.C_in) annotation (Line(points={{-85,0},{-16,0}}, color={0,255,128}));
      connect(heat_loss.C_out, sink.C_in) annotation (Line(points={{14,0},{85,0}}, color={0,255,128}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end HeatLoss;

    model Leak
        extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;
          // Boundary conditions
      input Utilities.Units.Pressure source_P(start=10e5) "Pa";
      input Utilities.Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
      input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Utilities.Units.Fraction source_relative_humidity(start=0.5) "1";
      input Utilities.Units.Pressure sink_P(start=1e5, min=0, nominal=10e5) "Pa";

      MetroscopeModelingLibrary.RefMoistAir.Pipes.Leak leak annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
      MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
    equation
      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;
      sink.P_in = sink_P;

      connect(leak.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={0,255,128}));
      connect(source.C_out, leak.C_in) annotation (Line(points={{-37,0},{-10,0}}, color={0,255,128}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Leak;

    model PressureCut
      extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

      // Boundary conditions
      input Utilities.Units.SpecificEnthalpy source_h(start=1e3);
      input Utilities.Units.Pressure source_P(start=10e5, min=0, nominal=10e5) "Pa";
      input Utilities.Units.Pressure sink_P(start=1e5, min=0, nominal=10e5) "Pa";
      input Utilities.Units.NegativeMassFlowRate source_Q(start=100) "kg/s";
      input Utilities.Units.Fraction source_relative_humidity(start=0.8) "1";

      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-100,-9.99996},{-80,9.99996}})));
      .MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={90,-6.10623e-16})));
      .MetroscopeModelingLibrary.RefMoistAir.Pipes.PressureCut pressureCut annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    equation

      // Boundary conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = - source_Q;
      source.relative_humidity = source_relative_humidity;
      sink.P_in = sink_P;

      connect(source.C_out, pressureCut.C_in) annotation (Line(points={{-85,0},{-10,0}}, color={0,255,128}));
      connect(pressureCut.C_out, sink.C_in) annotation (Line(points={{10,0},{85,0}}, color={0,255,128}));
    end PressureCut;
  end Pipes;
end RefMoistAir;
