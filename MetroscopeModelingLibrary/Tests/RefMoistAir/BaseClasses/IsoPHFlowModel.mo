within MetroscopeModelingLibrary.Tests.RefMoistAir.BaseClasses;
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

  connect(isoPHFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={0,127,127}));
  connect(isoPHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={0,127,127}));
end IsoPHFlowModel;
