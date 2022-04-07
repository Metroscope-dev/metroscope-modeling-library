within MetroscopeModelingLibrary.Partial.Pipes;
partial model Pipe
  extends MetroscopeModelingLibrary.Partial.BaseClasses.IsoHFlowModel annotation(IconMap(primitivesVisible=false));
  import MetroscopeModelingLibrary.Units;
  import MetroscopeModelingLibrary.Units.Inputs;
  import MetroscopeModelingLibrary.Constants;

  // Initialization parameters
  parameter Units.DifferentialHeight delta_z_0 = 0; // delta_z = z_out - z_in, delta_z > 0 means DP_z < 0
  parameter Units.DifferentialPressure DP_f_0 = 1e5;
  parameter Units.DifferentialPressure DP_z_0 = 0.001e5;

  Inputs.InputFrictionCoefficient Kfr(start=10) "Friction pressure loss coefficient";
  Inputs.InputDifferentialHeight delta_z(start=delta_z_0, nominal=5) "Height difference between outlet and inlet";
  Units.DifferentialPressure DP_f(start=DP_f_0) "Singular pressure loss";
  Units.DifferentialPressure DP_z(start=DP_z_0) "Singular pressure loss";
equation
  DP_f = -Kfr * Q*abs(Q) / rhom;
  DP_z = -rhom * Constants.g * delta_z;

  DP = DP_f + DP_z;
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=true,
        extent={{-100,-100},{100,100}},
        grid={2,2}), graphics={Rectangle(
          extent={{-100,20},{100,-20}},
          lineColor={0,0,255},
          fillColor={85,255,85},
          fillPattern=FillPattern.Solid)}),
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
end Pipe;
