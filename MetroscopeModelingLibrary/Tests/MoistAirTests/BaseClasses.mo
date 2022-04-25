within MetroscopeModelingLibrary.Tests.MoistAirTests;
package BaseClasses
  extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestPackageIcon;
  model FlowModel
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
    MoistAir.BaseClasses.FlowModel moist_air_FlowModel annotation (Placement(transformation(extent={{7,-23},{53,23}})));
    MoistAir.BoundaryConditions.Source moist_air_Source annotation (Placement(transformation(extent={{-108.5,-18.5},{-71.5,18.5}})));
    MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor moist_air_PressureSensor annotation (Placement(transformation(extent={{-72,-10},{-52,10}})));
    MetroscopeModelingLibrary.Sensors.MoistAir.FlowSensor moist_air_FlowSensor annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
    MoistAir.BoundaryConditions.Sink moist_air_Sink annotation (Placement(transformation(extent={{61.5,-20},{102.5,20}})));
  equation
    moist_air_FlowModel.W_input = 0;
    moist_air_FlowModel.DP_input = 0;

    moist_air_Source.relative_humidity = 0.1;
    moist_air_Source.h_out = 1e3;

    moist_air_PressureSensor.P_barA = 0.9;
    moist_air_FlowSensor.Q = 100;

    assert(abs(moist_air_Sink.Q_in + moist_air_Source.Q_out) <= 1e-5, "In flow model, DM should be 0");
    connect(moist_air_Source.C_out, moist_air_PressureSensor.C_in) annotation (Line(points={{-80.75,0},{-72,0},{-72,0}},          color={85,170,255}));
    connect(moist_air_PressureSensor.C_out, moist_air_FlowSensor.C_in) annotation (Line(points={{-52,0},{-50,0},{-50,0.1},{-46,0.1},{-46,0},{-40,0}},
                                                                                                                        color={85,170,255}));
    connect(moist_air_FlowModel.C_in, moist_air_FlowSensor.C_out) annotation (Line(points={{7,0},{4.5,0},{4.5,0},{-20,0}},           color={85,170,255}));
    connect(moist_air_FlowModel.C_out, moist_air_Sink.C_in) annotation (Line(points={{53,0},{62.375,0},{62.375,0},{71.75,0}},         color={85,170,255}));
  end FlowModel;

  model IsoPFlowModel
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
    MoistAir.BaseClasses.IsoPFlowModel moist_air_IsoPFlowModel annotation (Placement(transformation(extent={{5,-23},{51,23}})));
    MoistAir.BoundaryConditions.Source moist_air_Source annotation (Placement(transformation(extent={{-109,-19},{-71,19}})));
    MoistAir.BoundaryConditions.Sink moist_air_Sink annotation (Placement(transformation(extent={{66,-19.5},{106,19.5}})));
    MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor moist_air_PressureSensor annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
    MetroscopeModelingLibrary.Sensors.MoistAir.FlowSensor moist_air_FlowSensor annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  equation
    moist_air_IsoPFlowModel.W_input = 0;

    moist_air_Source.relative_humidity = 0.1;
    moist_air_Source.h_out = 1e3;

    moist_air_PressureSensor.P = 0.9e5;
    moist_air_FlowSensor.Q = 100;

    assert(abs(moist_air_Sink.Q_in + moist_air_Source.Q_out) <= 1e-5, "In IsoPFlowModel, DM should be 0");
    assert(abs(moist_air_Source.P_out - moist_air_Sink.P_in) <= 1e-5, "In IsoPFlowModel, DP should be 0");
    connect(moist_air_IsoPFlowModel.C_out, moist_air_Sink.C_in) annotation (Line(points={{51,0},{57.2,0},{57.2,0},{76,0}},             color={85,170,255}));
    connect(moist_air_PressureSensor.C_out,moist_air_FlowSensor. C_in) annotation (Line(points={{-50,0},{-48,0},{-48,0.1},{-44,0.1},{-44,0},{-38,0}},
                                                                                                                        color={85,170,255}));
    connect(moist_air_PressureSensor.C_in, moist_air_Source.C_out) annotation (Line(points={{-70,0},{-73.82,0},{-73.82,0},{-80.5,0}},        color={85,170,255}));
    connect(moist_air_IsoPFlowModel.C_in, moist_air_FlowSensor.C_out) annotation (Line(points={{5,0},{-11.5,0},{-11.5,0},{-18,0}},               color={85,170,255}));
  end IsoPFlowModel;

  model IsoHFlowModel
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
    MoistAir.BaseClasses.IsoHFlowModel moist_air_IsoHFlowModel annotation (Placement(transformation(extent={{5,-23},{51,23}})));
    MoistAir.BoundaryConditions.Source moist_air_Source annotation (Placement(transformation(extent={{-109,-19},{-71,19}})));
    MoistAir.BoundaryConditions.Sink moist_air_Sink annotation (Placement(transformation(extent={{66,-19.5},{106,19.5}})));
    MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor moist_air_PressureSensor annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
    MetroscopeModelingLibrary.Sensors.MoistAir.FlowSensor moist_air_FlowSensor annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  equation
    moist_air_IsoHFlowModel.DP_input = 0;

    moist_air_Source.relative_humidity = 0.1;
    moist_air_Source.h_out = 1e3;

    moist_air_PressureSensor.P = 1e5;
    moist_air_FlowSensor.Q = 100;

    assert(abs(moist_air_Sink.Q_in + moist_air_Source.Q_out) <= 1e-5, "In IsoHFlowModel, DM should be 0");
    assert(abs(moist_air_Sink.Q_in*moist_air_Sink.h_in + moist_air_Source.Q_out*moist_air_Source.h_out) <= 1e-5, "In IsoHFlowModel, W should be 0");
    connect(moist_air_IsoHFlowModel.C_out, moist_air_Sink.C_in) annotation (Line(points={{51,0},{57.2,0},{57.2,0},{76,0}},             color={85,170,255}));
    connect(moist_air_PressureSensor.C_out,moist_air_FlowSensor. C_in) annotation (Line(points={{-50,0},{-48,0},{-48,0.1},{-44,0.1},{-44,0},{-38,0}},
                                                                                                                        color={85,170,255}));
    connect(moist_air_PressureSensor.C_in, moist_air_Source.C_out) annotation (Line(points={{-70,0},{-73.82,0},{-73.82,0},{-80.5,0}},        color={85,170,255}));
    connect(moist_air_IsoHFlowModel.C_in, moist_air_FlowSensor.C_out) annotation (Line(points={{5,0},{-11.5,0},{-11.5,0},{-18,0}},               color={85,170,255}));
  end IsoHFlowModel;

  model IsoPHFlowModel
    extends MetroscopeModelingLibrary.Icons.Tests.MoistAirTestIcon;
    MoistAir.BaseClasses.IsoPHFlowModel moist_air_IsoPHFlowModel annotation (Placement(transformation(extent={{5,-23},{51,23}})));
    MoistAir.BoundaryConditions.Source moist_air_Source annotation (Placement(transformation(extent={{-109,-19},{-71,19}})));
    MoistAir.BoundaryConditions.Sink moist_air_Sink annotation (Placement(transformation(extent={{66,-19.5},{106,19.5}})));
    MetroscopeModelingLibrary.Sensors.MoistAir.PressureSensor moist_air_PressureSensor annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
    MetroscopeModelingLibrary.Sensors.MoistAir.FlowSensor moist_air_FlowSensor annotation (Placement(transformation(extent={{-38,-10},{-18,10}})));
  equation
    moist_air_Source.relative_humidity = 0.1;
    moist_air_Source.h_out = 1e3;

    moist_air_PressureSensor.P = 1e5;
    moist_air_FlowSensor.Q = 100;

    assert(abs(moist_air_Sink.Q_in + moist_air_Source.Q_out) <= 1e-5, "In IsoPHFlowModel, DM should be 0");
    assert(abs(moist_air_Sink.Qv_in + moist_air_Source.Qv_out) <= 1e-5, "In IsoPHFlowModel, DV should be 0");
    assert(abs(moist_air_Source.P_out - moist_air_Sink.P_in) <= 1e-5, "In IsoPHFlowModel, DP should be 0");
    assert(abs(moist_air_Sink.Q_in*moist_air_Sink.h_in + moist_air_Source.Q_out*moist_air_Source.h_out) <= 1e-5, "In IsoPHFlowModel, W should be 0");
    connect(moist_air_IsoPHFlowModel.C_out, moist_air_Sink.C_in) annotation (Line(points={{51,0},{57.2,0},{57.2,0},{76,0}},             color={85,170,255}));
    connect(moist_air_PressureSensor.C_out,moist_air_FlowSensor. C_in) annotation (Line(points={{-50,0},{-48,0},{-48,0.1},{-44,0.1},{-44,0},{-38,0}},
                                                                                                                        color={85,170,255}));
    connect(moist_air_PressureSensor.C_in, moist_air_Source.C_out) annotation (Line(points={{-70,0},{-73.82,0},{-73.82,0},{-80.5,0}},        color={85,170,255}));
    connect(moist_air_IsoPHFlowModel.C_in, moist_air_FlowSensor.C_out) annotation (Line(points={{5,0},{-11.5,0},{-11.5,0},{-18,0}},               color={85,170,255}));
  end IsoPHFlowModel;
end BaseClasses;
