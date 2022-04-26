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
end MoistAir;
