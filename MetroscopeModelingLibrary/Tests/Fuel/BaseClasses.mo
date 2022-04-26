within MetroscopeModelingLibrary.Tests.Fuel;
package BaseClasses
  extends MetroscopeModelingLibrary.Icons.Tests.FuelTestPackageIcon;
  model FlowModel
    extends MetroscopeModelingLibrary.Icons.Tests.FuelTestIcon;

    import MetroscopeModelingLibrary.Units;

    // Boundary conditions
    input Units.Pressure source_P(start=1e5) "Pa";
    input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
    input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

    // Parameters
    input Units.DifferentialPressure DP(start=1e5);
    input Units.Power W(start=1e5);

    .MetroscopeModelingLibrary.Fuel.BaseClasses.FlowModel flowModel annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
    .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
    .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
  equation

    // Boundary Conditions
    source.h_out = source_h;
    source.P_out = source_P;
    source.Q_out = source_Q;
    source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

    // Parameters
    flowModel.DP = DP;
    flowModel.W = W;

    connect(source.C_out, flowModel.C_in) annotation (Line(points={{-70.5,0},{-23,0}}, color={213,213,0}));
    connect(flowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={213,213,0}));
  end FlowModel;

  model IsoPFlowModel
    extends MetroscopeModelingLibrary.Icons.Tests.FuelTestIcon;

    import MetroscopeModelingLibrary.Units;

    // Boundary conditions
    input Units.Pressure source_P(start=1e5) "Pa";
    input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
    input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

    // Parameters
    input Units.Power W(start=1e5);

    MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPFlowModel   isoPFlowModel
                                                                         annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
    .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
    .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
  equation

    // Boundary Conditions
    source.h_out = source_h;
    source.P_out = source_P;
    source.Q_out = source_Q;
    source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

    // Parameters
    isoPFlowModel.W = W;

    connect(isoPFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={213,213,0}));
    connect(isoPFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={213,213,0}));
  end IsoPFlowModel;

  model IsoHFlowModel
    extends MetroscopeModelingLibrary.Icons.Tests.FuelTestIcon;

    import MetroscopeModelingLibrary.Units;

    // Boundary conditions
    input Units.Pressure source_P(start=1e5) "Pa";
    input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
    input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

    // Parameters
    input Units.DifferentialPressure DP(start=1e5);

    MetroscopeModelingLibrary.Fuel.BaseClasses.IsoHFlowModel   isoHFlowModel
                                                                         annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
    .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
    .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
  equation

    // Boundary Conditions
    source.h_out = source_h;
    source.P_out = source_P;
    source.Q_out = source_Q;
    source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

    // Parameters
    isoHFlowModel.DP = DP;

    connect(isoHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={213,213,0}));
    connect(isoHFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={213,213,0}));
  end IsoHFlowModel;

  model IsoPHFlowModel
    extends MetroscopeModelingLibrary.Icons.Tests.FuelTestIcon;

    import MetroscopeModelingLibrary.Units;

    // Boundary conditions
    input Units.Pressure source_P(start=10e5) "Pa";
    input Units.SpecificEnthalpy source_h(start=1e6) "J/kg";
    input Units.NegativeMassFlowRate source_Q(start=-100) "kg/s";

    MetroscopeModelingLibrary.Fuel.BaseClasses.IsoPHFlowModel  isoPHFlowModel
                                                                         annotation (Placement(transformation(extent={{-23,-23},{23,23}})));
    .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Source source annotation (Placement(transformation(extent={{-99,-19},{-61,19}})));
    .MetroscopeModelingLibrary.Fuel.BoundaryConditions.Sink sink annotation (Placement(transformation(extent={{59,-20},{101,20}})));
  equation

    // Boundary Conditions
    source.h_out = source_h;
    source.P_out = source_P;
    source.Q_out = source_Q;
    source.Xi_out = {0.92,0.048,0.005,0.002,0.015,0.01};

    connect(isoPHFlowModel.C_out, sink.C_in) annotation (Line(points={{23,0},{69.5,0}}, color={95,95,95}));
    connect(isoPHFlowModel.C_in, source.C_out) annotation (Line(points={{-23,0},{-70.5,0}}, color={95,95,95}));
  end IsoPHFlowModel;
end BaseClasses;
