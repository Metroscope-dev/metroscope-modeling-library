within MetroscopeModelingLibrary.Tests.Sensors;
package RefMoistAir
  extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestPackageIcon;

  model TemperatureSensor
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

    // Boundary conditions
    input Utilities.Units.Pressure source_P(start=1e5) "Pa";
    input Utilities.Units.SpecificEnthalpy source_h(start=2e4) "J/kg";
    input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

    MetroscopeModelingLibrary.Sensors.RefMoistAir.TemperatureSensor T_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
    MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
  equation
    source.P_out = source_P;
    source.Q_out = source_Q;
    source.relative_humidity = 0.1;
    source.h_out = source_h;

    assert(abs(T_sensor.T_degC - 16.91) < 1e-2, "T_sensor should detect 25 deg C");
    assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
    assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
    assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
    connect(T_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={0,255,128}));
    connect(T_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={0,255,128}));
  end TemperatureSensor;

  model FlowSensor
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

    // Boundary conditions
    input Utilities.Units.Pressure source_P(start=1e5) "Pa";
    input Utilities.Units.SpecificEnthalpy source_h(start=2e4) "J/kg";
    input Utilities.Units.PositiveMassFlowRate source_Q(start=100) "kg/s";

    MetroscopeModelingLibrary.Sensors.RefMoistAir.FlowSensor source_Q_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
    MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
  equation
    source.P_out = source_P;
    source.h_out = source_h;
    source.Q_out = - source_Q;
    source.relative_humidity = 0.1;

    if not source_Q_sensor.faulty then // To avoid breaking FlowSensor_faulty test
      assert(abs(source_Q_sensor.Q - 100) < 1e-5, "Q_sensor should detect 100 kg/s");
    end if;
    assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
    assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
    assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
    connect(source_Q_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={0,255,128}));
    connect(source_Q_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={0,255,128}));
  end FlowSensor;

  model FlowSensor_faulty
    extends FlowSensor(source_Q_sensor(faulty = true));

    Utilities.Units.MassFlowRate Fault_mass_flow_rate_bias(start=0);
  equation
    Fault_mass_flow_rate_bias = 0 + 50 * time; // bias of -50 means that real flow = measured flow - 50
    source_Q_sensor.mass_flow_rate_bias = Fault_mass_flow_rate_bias;

    connect(source_Q_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={28,108,200}));
    connect(source_Q_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={28,108,200}));
  end FlowSensor_faulty;

  model PressureSensor
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

    // Boundary conditions
    input Utilities.Units.Pressure source_P(start=1e5) "Pa";
    input Utilities.Units.SpecificEnthalpy source_h(start=2e4) "J/kg";
    input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

    MetroscopeModelingLibrary.Sensors.RefMoistAir.PressureSensor P_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
    MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{54,-10},{74,10}})));
  equation
    source.h_out = source_h;
    source.Q_out = source_Q;
    source.relative_humidity = 0.1;
    P_sensor.P = 1e5;

    assert(abs(P_sensor.P_barA - 1) < 1e-5, "P_sensor should detect 1 barA");
    assert(abs(P_sensor.P_psiA - 14.50377377) < 1e-3, "P_sensor should detect 14.5 psiA");
    assert(abs(P_sensor.P_kPaA - 100) < 1e-3, "P_sensor should detect 100 kPaA");
    assert(abs(P_sensor.P_MPaA - 0.1) < 1e-3, "P_sensor should detect 0.1 MPaA");

    assert(abs(P_sensor.P_kPaG) < 1e-3, "P_sensor should detect 0 kPaG");
    assert(abs(P_sensor.P_MPaG) < 1e-3, "P_sensor should detect 0 MPaG");
    assert(abs(P_sensor.P_barG) < 1e-5, "P_sensor should detect 0 barG");
    assert(abs(P_sensor.P_psiG) < 1e-3, "P_sensor should detect 0 psiG");

    assert(abs(P_sensor.P_inHg - 29.530058647) < 1e-5, "P_sensor should detect 29.53 inHg");
    assert(abs(P_sensor.P_mbar - 1000) < 1e-5, "P_sensor should detect 1000 mbar");

    assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
    assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
    assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
    connect(P_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={0,255,128}));
    connect(P_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{59,0}}, color={0,255,128}));
  end PressureSensor;

  model DeltaPressureSensor
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

    // Boundary conditions
    input Utilities.Units.Pressure source_P(start=1e5) "Pa";
    input Utilities.Units.SpecificEnthalpy source_h(start=2e4) "J/kg";
    input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

    MetroscopeModelingLibrary.Sensors.RefMoistAir.DeltaPressureSensor DP_sensor annotation (Placement(transformation(extent={{-10,10},{10,30}})));
    MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
    MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{52,-10},{72,10}})));
    MetroscopeModelingLibrary.RefMoistAir.BaseClasses.IsoHFlowModel MoistAirIsoHFlowModel annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  equation
    source.h_out = source_h;
    source.Q_out = source_Q;
    source.P_out = source_P;
    source.relative_humidity = 0.1;
    DP_sensor.DP = 0.5e5;

    assert(abs(DP_sensor.DP_bar - 0.5) < 1e-5, "DP_sensor should detect 1 barA");
    assert(abs(sink.P_in - source.P_out - DP_sensor.DP) < 1e-5, "Pressure difference from source to sink should be the one detected by DP_sensor");
    assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
    assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
    connect(MoistAirIsoHFlowModel.C_in, source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={0,255,128}));
    connect(MoistAirIsoHFlowModel.C_out, sink.C_in) annotation (Line(points={{10,0},{57,0}}, color={0,255,128}));
    connect(DP_sensor.C_out, sink.C_in) annotation (Line(points={{10,20},{40,20},{40,0},{57,0}}, color={0,255,128}));
    connect(DP_sensor.C_in, source.C_out) annotation (Line(points={{-10,20},{-28,20},{-28,0},{-39,0}}, color={0,255,128}));
  end DeltaPressureSensor;

  model RelativeHumiditySensor
    extends MetroscopeModelingLibrary.Utilities.Icons.Tests.RefMoistAirTestIcon;

    // Boundary conditions
    input Utilities.Units.Pressure source_P(start=1e5) "Pa";
    input Utilities.Units.SpecificEnthalpy source_h(start=2e4) "J/kg";
    input Utilities.Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";
    input Real relative_humidity(start=10) "%";

    MetroscopeModelingLibrary.Sensors.RefMoistAir.RelativeHumiditySensor
                                                              H_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-54,-10},{-34,10}})));
    MetroscopeModelingLibrary.RefMoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{34,-10},{54,10}})));
  equation
    source.h_out = source_h;
    source.Q_out = source_Q;
    source.P_out = 1e5;
    H_sensor.relative_humidity_pc = relative_humidity;

    connect(H_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-39,0}}, color={0,255,128}));
    connect(H_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{39,0}}, color={0,255,128}));
  end RelativeHumiditySensor;
end RefMoistAir;
