within MetroscopeModelingLibrary.Tests.Sensors;
package MoistAir
  extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestPackageIcon;

  model MoistAirTemperatureSensor
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;

    // Boundary conditions
    input Units.Pressure source_P(start=1e5) "Pa";
    input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
    input Units.OutletMassFlowRate source_Q(start=-100) "kg/s";

    MetroscopeModelingLibrary.Sensors.MoistAir.MoistAirTemperatureSensor T_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
    MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{38,-10},{58,10}})));
  equation
    source.P_out = source_P;
    source.Q_out = source_Q;
    source.relative_humidity = 0.1;
    T_sensor.T = 298.15;

    assert(abs(T_sensor.T_degC - 25) < 1e-5, "T_sensor should detect 25 deg C");
    assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
    assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
    assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
    connect(T_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-43,0}}, color={28,108,200}));
    connect(T_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{43,0}}, color={28,108,200}));
  end MoistAirTemperatureSensor;

  model MoistAirFlowSensor
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;

    // Boundary conditions
    input Units.Pressure source_P(start=1e5) "Pa";
    input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
    input Units.InletMassFlowRate source_Q(start=100) "kg/s";

    MetroscopeModelingLibrary.Sensors.MoistAir.MoistAirFlowSensor source_Q_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
    MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{38,-10},{58,10}})));
  equation
    source.P_out = source_P;
    source.h_out = source_h;
    source_Q_sensor.Q = source_Q;
    source.relative_humidity = 0.1;

    assert(abs(source_Q_sensor.Q - 100) < 1e-5, "T_sensor should detect 25 deg C");
    assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
    assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
    assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
    connect(source_Q_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-43,0}}, color={28,108,200}));
    connect(source_Q_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{43,0}}, color={28,108,200}));
  end MoistAirFlowSensor;

  model MoistAirPressureSensor
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;

    // Boundary conditions
    input Units.Pressure source_P(start=1e5) "Pa";
    input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
    input Units.OutletMassFlowRate source_Q(start=-100) "kg/s";

    MetroscopeModelingLibrary.Sensors.MoistAir.MoistAirPressureSensor P_sensor annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
    MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{38,-10},{58,10}})));
  equation
    source.h_out = source_h;
    source.Q_out = source_Q;
    source.relative_humidity = 0.1;
    P_sensor.P = 1e5;

    assert(abs(P_sensor.P_barA - 1) < 1e-5, "P_sensor should detect 1 barA");
    assert(abs(P_sensor.P_barG) < 1e-5, "P_sensor should detect 0 barG");
    assert(abs(source.P_out - sink.P_in) < 1e-5, "Pressure should be the same from source to sink");
    assert(abs(source.h_out - sink.h_in) < 1e-5, "Enthalpy should be the same from source to sink");
    assert(abs(source.Q_out + sink.Q_in) < 1e-5, "MassFlowRate should be the same from source to sink");
    connect(P_sensor.C_in, source.C_out) annotation (Line(points={{-10,0},{-43,0}}, color={28,108,200}));
    connect(P_sensor.C_out, sink.C_in) annotation (Line(points={{10,0},{43,0}}, color={28,108,200}));
  end MoistAirPressureSensor;

  model MoistAirDeltaPressureSensor
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;

    // Boundary conditions
    input Units.Pressure source_P(start=1e5) "Pa";
    input Units.SpecificEnthalpy source_h(start=1e3) "J/kg";
    input Units.OutletMassFlowRate source_Q(start=-100) "kg/s";

    MetroscopeModelingLibrary.Sensors.MoistAir.MoistAirDeltaPressureSensor DP_sensor annotation (Placement(transformation(extent={{-10,10},{10,30}})));
    MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-58,-10},{-38,10}})));
    MetroscopeModelingLibrary.MoistAir.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{38,-10},{58,10}})));
    MetroscopeModelingLibrary.MoistAir.BaseClasses.IsoHFlowModel MoistAirIsoHFlowModel annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
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
    connect(DP_sensor.C_out, sink.C_in) annotation (Line(points={{10,20},{20,20},{20,0},{43,0}}, color={28,108,200}));
    connect(source.C_out, MoistAirIsoHFlowModel.C_in) annotation (Line(points={{-43,0},{-10,0}}, color={28,108,200}));
    connect(MoistAirIsoHFlowModel.C_out, sink.C_in) annotation (Line(points={{10,0},{43,0}}, color={28,108,200}));
    connect(DP_sensor.C_in, MoistAirIsoHFlowModel.C_in) annotation (Line(points={{-10,20},{-20,20},{-20,0},{-10,0}}, color={28,108,200}));
  end MoistAirDeltaPressureSensor;
end MoistAir;
