within MetroscopeModelingLibrary.Tests.WaterSteamTests.Pipes;
model LoopBreaker

  import MetroscopeModelingLibrary.Units;
  extends MetroscopeModelingLibrary.Icons.Tests.WaterSteamTestIcon;

  // Boundary conditions
  input Units.SpecificEnthalpy h(start=1e6) "J/kg";
  input Units.Pressure P(start=10e5) "Pa";
  input Units.PositiveMassFlowRate Q(start=100) "kg/s";

  WaterSteam.Pipes.LoopBreaker loopBreaker annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  WaterSteam.BaseClasses.FlowModel flowModel annotation (Placement(transformation(
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
