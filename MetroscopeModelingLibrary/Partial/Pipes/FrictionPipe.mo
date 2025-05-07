within MetroscopeModelingLibrary.Partial.Pipes;
partial model FrictionPipe
  extends MetroscopeModelingLibrary.Partial.BaseClasses.IsoHFlowModel annotation(IconMap(primitivesVisible=false));
  import MetroscopeModelingLibrary.Utilities.Units;
  import MetroscopeModelingLibrary.Utilities.Units.Inputs;
  import MetroscopeModelingLibrary.Utilities.Constants;

  // Failure modes
  parameter Boolean faulty = false;
  Units.Percentage fouling; // Fouling coefficient

  parameter Units.FrictionCoefficient Kfr_constant = -DP_0*rho_0/(Q_0*Q_0);
  Utilities.Interfaces.GenericReal Kfr(start=Kfr_constant)        annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,40}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,40})));
equation
  // Failure modes
  if not faulty then
    fouling = 0;
  end if;

  DP = - (1+ fouling/100)*Kfr*Q*abs(Q)/rho_in; // homotopy((1+ fouling/100)*Kfr*Q*abs(Q)/rho_in, DP_0/Q_0*Q);
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,20},{-76,18},{-72,20}}, color={28,108,200}),
        Line(points={{-72,20},{-68,18},{-64,20}}, color={28,108,200}),
        Line(points={{-56,20},{-52,18},{-48,20}}, color={28,108,200}),
        Line(points={{-64,20},{-60,18},{-56,20}}, color={28,108,200}),
        Line(points={{-24,20},{-20,18},{-16,20}}, color={28,108,200}),
        Line(points={{-32,20},{-28,18},{-24,20}}, color={28,108,200}),
        Line(points={{-40,20},{-36,18},{-32,20}}, color={28,108,200}),
        Line(points={{-48,20},{-44,18},{-40,20}}, color={28,108,200}),
        Line(points={{-16,20},{-12,18},{-8,20}}, color={28,108,200}),
        Line(points={{-8,20},{-4,18},{0,20}}, color={28,108,200}),
        Line(points={{8,20},{12,18},{16,20}}, color={28,108,200}),
        Line(points={{0,20},{4,18},{8,20}}, color={28,108,200}),
        Line(points={{40,20},{44,18},{48,20}}, color={28,108,200}),
        Line(points={{32,20},{36,18},{40,20}}, color={28,108,200}),
        Line(points={{24,20},{28,18},{32,20}}, color={28,108,200}),
        Line(points={{16,20},{20,18},{24,20}}, color={28,108,200}),
        Line(points={{72,20},{76,18},{80,20}}, color={28,108,200}),
        Line(points={{64,20},{68,18},{72,20}}, color={28,108,200}),
        Line(points={{56,20},{60,18},{64,20}}, color={28,108,200}),
        Line(points={{48,20},{52,18},{56,20}}, color={28,108,200}),
        Line(points={{-80,-20},{-76,-18},{-72,-20}}, color={28,108,200}),
        Line(points={{-72,-20},{-68,-18},{-64,-20}}, color={28,108,200}),
        Line(points={{-56,-20},{-52,-18},{-48,-20}}, color={28,108,200}),
        Line(points={{-64,-20},{-60,-18},{-56,-20}}, color={28,108,200}),
        Line(points={{-24,-20},{-20,-18},{-16,-20}}, color={28,108,200}),
        Line(points={{-32,-20},{-28,-18},{-24,-20}}, color={28,108,200}),
        Line(points={{-40,-20},{-36,-18},{-32,-20}}, color={28,108,200}),
        Line(points={{-48,-20},{-44,-18},{-40,-20}}, color={28,108,200}),
        Line(points={{-16,-20},{-12,-18},{-8,-20}}, color={28,108,200}),
        Line(points={{-8,-20},{-4,-18},{0,-20}}, color={28,108,200}),
        Line(points={{8,-20},{12,-18},{16,-20}}, color={28,108,200}),
        Line(points={{0,-20},{4,-18},{8,-20}}, color={28,108,200}),
        Line(points={{40,-20},{44,-18},{48,-20}}, color={28,108,200}),
        Line(points={{32,-20},{36,-18},{40,-20}}, color={28,108,200}),
        Line(points={{24,-20},{28,-18},{32,-20}}, color={28,108,200}),
        Line(points={{16,-20},{20,-18},{24,-20}}, color={28,108,200}),
        Line(points={{72,-20},{76,-18},{80,-20}}, color={28,108,200}),
        Line(points={{64,-20},{68,-18},{72,-20}}, color={28,108,200}),
        Line(points={{56,-20},{60,-18},{64,-20}}, color={28,108,200}),
        Line(points={{48,-20},{52,-18},{56,-20}}, color={28,108,200})}),
    Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid), Text(
          extent={{-12,14},{16,-14}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid)}));
end FrictionPipe;
