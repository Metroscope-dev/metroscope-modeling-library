within MetroscopeModelingLibrary.Tests;
package MoistAir
  extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestPackageIcon;

  package BoundaryConditions
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestPackageIcon;

    model Source
      extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
      import MetroscopeModelingLibrary.Units;

      // Boundary conditinos
      input Units.Pressure source_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
    equation
      source.P_out = source_P;
      source.h_out = source_h;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={85,170,255}));
    end Source;

    model Sink
      extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
      import MetroscopeModelingLibrary.Units;

      // Boundary conditinos
      input Units.Pressure sink_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy sink_h(start=1e3) "J/kg";
      input Units.PositiveMassFlowRate sink_Q(start=100) "kg/s";
      input Units.Fraction sink_relative_humidity(start=0.5) "1";

      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{18,-10},{38,10}})));
    equation
      sink.P_in = sink_P;
      sink.h_in = sink_h;
      sink.Q_in = sink_Q;
      sink.relative_humidity = sink_relative_humidity;

      connect(source.C_out, sink.C_in) annotation (Line(points={{-23,0},{23,0}}, color={85,170,255}));
    end Sink;
  end BoundaryConditions;

  package BaseClasses
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestPackageIcon;
    model FlowModel
      extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;

      import MetroscopeModelingLibrary.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      // Parameters
      input Units.DifferentialPressure DP(start=0.1e5);
      input Units.Power W(start=1e5);

      .MetroscopeModelingLibrary.MoistAir.BaseClasses.FlowModel flowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      flowModel.DP = DP;
      flowModel.W = W;

      connect(flowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={85,170,255}));
      connect(flowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={85,170,255}));
    end FlowModel;

    model IsoPFlowModel
      extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;

      import MetroscopeModelingLibrary.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      // Parameters
      input Units.Power W(start=1e5);

      MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPFlowModel
                                                                 isoPFlowModel
                                                                           annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      isoPFlowModel.W = W;

      connect(isoPFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={85,170,255}));
      connect(isoPFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={85,170,255}));
    end IsoPFlowModel;

    model IsoHFlowModel
      extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;

      import MetroscopeModelingLibrary.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      // Parameters
      input Units.DifferentialPressure DP(start=0.1e5);

      MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoHFlowModel
                                                                 isoHFlowModel
                                                                           annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      isoHFlowModel.DP = DP;

      connect(isoHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={85,170,255}));
      connect(isoHFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={85,170,255}));
    end IsoHFlowModel;

    model IsoPHFlowModel
      extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;

      import MetroscopeModelingLibrary.Units;

      // Boundary conditions
      input Units.Pressure source_P(start=1e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

      MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoPHFlowModel
                                                                 isoPHFlowModel
                                                                           annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
      .MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
    equation

      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      connect(isoPHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={85,170,255}));
      connect(isoPHFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={85,170,255}));
    end IsoPHFlowModel;
  end BaseClasses;

  package Pipes
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestPackageIcon;
    model Pipe_direct
        extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
          // Boundary conditions
      input Units.Pressure source_P(start=10e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

        // Parameters
      parameter Units.FrictionCoefficient Kfr = 100;
      parameter Units.Height delta_z = 0;

      MetroscopeModelingLibrary.MoistAir.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
      MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
    equation
      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      pipe.Kfr = Kfr;
      pipe.delta_z = delta_z;

      connect(pipe.C_in, source.C_out) annotation (Line(points={{-10,0},{-37,0}}, color={85,170,255}));
      connect(pipe.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={85,170,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Pipe_direct;

    model Pipe_reverse
      extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
          // Boundary conditions
      input Units.Pressure source_P(start=10e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

        // Parameters
      parameter Units.Height delta_z = 1;

      // Inputs for calibration
      input Real P_out(start=9) "barA";

      // Parameters for calibration
      output Units.FrictionCoefficient Kfr;

      MetroscopeModelingLibrary.MoistAir.Pipes.Pipe pipe annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
      MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{50,-10},{70,10}})));
      MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor P_out_sensor annotation (Placement(transformation(extent={{24,-10},{44,10}})));
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


      connect(pipe.C_in,source. C_out) annotation (Line(points={{-10,0},{-37,0}}, color={85,170,255}));
      connect(pipe.C_out, P_out_sensor.C_in) annotation (Line(points={{10,0},{24,0}}, color={85,170,255}));
      connect(sink.C_in, P_out_sensor.C_out) annotation (Line(points={{55,0},{44,0}}, color={85,170,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end Pipe_reverse;

    model AdmiLouvers_direct
        extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
          // Boundary conditions
      input Units.Pressure source_P(start=10e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

        // Parameters
      parameter Units.FrictionCoefficient Kfr = 100;
      parameter Units.Height delta_z = 0;

      MetroscopeModelingLibrary.MoistAir.Pipes.AdmiLouver
                                                    admiLouver
                                                         annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-46,-10},{-26,10}})));
      MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    equation
      // Boundary Conditions
      source.h_out = source_h;
      source.P_out = source_P;
      source.Q_out = source_Q;
      source.relative_humidity = source_relative_humidity;

      // Parameters
      admiLouver.Kfr = Kfr;
      admiLouver.delta_z = delta_z;
      connect(admiLouver.C_in, source.C_out) annotation (Line(points={{-10,0},{-31,0}}, color={85,170,255}));
      connect(admiLouver.C_out, sink.C_in) annotation (Line(points={{10,0},{45,0}}, color={85,170,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end AdmiLouvers_direct;

    model AdmiLouvers_reverse
      extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
          // Boundary conditions
      input Units.Pressure source_P(start=10e5) "Pa";
      input Units.SpecificEnthalpy source_h(start=1e4) "J/kg";
      input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
      input Units.Fraction source_relative_humidity(start=0.5) "1";

        // Parameters
      parameter Units.Height delta_z = 1;

      // Inputs for calibration
      input Real P_out(start=9) "barA";

      // Parameters for calibration
      output Units.FrictionCoefficient Kfr;
      MetroscopeModelingLibrary.MoistAir.Pipes.AdmiLouver
                                                    admiLouver
                                                         annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
      MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
      MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{48,-10},{68,10}})));
      MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor P_out_sensor annotation (Placement(transformation(extent={{22,-10},{42,10}})));
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
      connect(admiLouver.C_in, source.C_out) annotation (Line(points={{-12,0},{-39,0}}, color={85,170,255}));
      connect(admiLouver.C_out, P_out_sensor.C_in) annotation (Line(points={{8,0},{22,0}}, color={85,170,255}));
      connect(sink.C_in,P_out_sensor. C_out) annotation (Line(points={{53,0},{42,0}}, color={85,170,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
    end AdmiLouvers_reverse;
  end Pipes;
end MoistAir;
