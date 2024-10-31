within MetroscopeModelingLibrary.Tests.RefMoistAir.Pipes;
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

  connect(source.C_out, heat_loss.C_in) annotation (Line(points={{-85,0},{-16,0}}, color={0,127,127}));
  connect(heat_loss.C_out, sink.C_in) annotation (Line(points={{14,0},{85,0}}, color={0,127,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end HeatLoss;
