within MetroscopeModelingLibrary.Partial.Pipes;
partial model Pipe
  extends MetroscopeModelingLibrary.Partial.BaseClasses.IsoHFlowModel annotation(IconMap(primitivesVisible=false));
  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;
  import MetroscopeModelingLibrary.Constants;

  // Initialization parameters
  parameter Units.Height z1_0 = 0;
  parameter Units.Height z2_0 = 0;
  parameter Units.DifferentialPressure DP_f_0 = 1e5;
  parameter Units.DifferentialPressure DP_z_0 = 0.001e5;

  Inputs.InputFrictionCoefficient Kfr(start=10) "Friction pressure loss coefficient";
  Inputs.InputHeight z1(start=z1_0) "Inlet altitude";
  Inputs.InputHeight z2(start=z2_0) "Outlet altitude";
  Units.DifferentialPressure DP_f(start=DP_f_0) "Singular pressure loss";
  Units.DifferentialPressure DP_z(start=DP_z_0) "Singular pressure loss";
equation
  DP_f = -Kfr * Q*abs(Q) / rhom;
  DP_z = -rhom * Constants.g * (z2 - z1);

  DP = DP_f + DP_z;
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid), Text(
          extent={{-12,14},{16,-14}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid,
          textString=
               "K")}),
    Icon(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid), Text(
          extent={{-12,14},{16,-14}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-46,37},{46,-37}},
          textColor={0,0,0},
          textString="DP"),
        Line(
          points={{-80,72},{-48,54},{0,48},{46,54},{80,72}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier),
        Line(
          points={{-80,-72},{-48,-54},{0,-48},{46,-54},{80,-72}},
          color={28,108,200},
          thickness=1,
          smooth=Smooth.Bezier)}));
end Pipe;
